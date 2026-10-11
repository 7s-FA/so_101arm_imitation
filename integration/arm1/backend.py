"""Model/hardware adapter. Only Controller's worker may call hardware methods."""
import json
import os
from pathlib import Path
import sys
import time

from engine import MotionBlocked

WORK = Path(__file__).resolve().parents[2]


def load_map(path, *, require_assets=True):
    config = json.loads(Path(path).read_text())
    if config.get('version') != 1 or set(config['models']) != {'SELECT_A','SELECT_B'}:
        raise ValueError('Expected SELECT_A/SELECT_B model map version 1')
    if not 30 <= config['job_timeout_seconds'] <= 3600:
        raise ValueError('Invalid job watchdog')
    for command, spec in config['models'].items():
        project = (WORK/'projects'/spec['project']).resolve()
        if project.parent != (WORK/'projects').resolve() or (require_assets and not (project/'project.json').is_file()):
            raise ValueError(f'{command}: invalid project')
        for key in ('model','completion'):
            path = (WORK/spec[key]).resolve()
            if not path.is_relative_to(project): raise ValueError(f'{command}: {key} outside project')
            if require_assets and not path.exists(): raise FileNotFoundError(path)
    return config


class HardwareBackend:
    def __init__(self, config, device='cuda'):
        # Model checks only; no SerialBus, Cameras, or motor writes until begin().
        sys.path.insert(0,str(WORK/'arm3-current/arm3'))
        import numpy as np
        import run_policy as policy
        from start_pose import load, aligned
        from task_completion import CompletionDetector
        from gripper_contact import GripperContactGuard
        self.np,self.p,self.aligned,self.Contact = np,policy,aligned,GripperContactGuard
        self.models = {}
        previous = os.environ.get('ARM3_PROJECT')
        try:
            for command,spec in config['models'].items():
                os.environ['ARM3_PROJECT']=str(WORK/'projects'/spec['project'])
                model=WORK/spec['model']
                cfg,cal=policy.load_context(model)
                detector=CompletionDetector(WORK/spec['completion'])
                task=json.loads((WORK/'projects'/spec['project']/'task.json').read_text())
                detector.validate_identity(policy.identity(cfg,task))
                print(f"{command} 완료 기준: version={detector.config['version']} "
                      f"method={detector.config.get('method','appearance')} | {detector.profile}",flush=True)
                pose=load(policy.model_dataset(model)/'start_pose.json',cal)
                if pose is None: raise ValueError(f'{command}: missing start pose')
                predictor=policy.Predictor(model,device)
                frames={n:(np.zeros((480,640,3),dtype=np.uint8),time.monotonic()) for n in policy.CAMERAS}
                predictor.predict(np.zeros(6,dtype=np.float32),frames);predictor.reset()
                self.models[command]=(cfg,cal,pose,predictor,detector)
                print(f'{command}: model/identity/completion/start pose validated: {model}',flush=True)
        finally:
            if previous is None: os.environ.pop('ARM3_PROJECT',None)
            else: os.environ['ARM3_PROJECT']=previous
        first,second=self.models.values()
        if first[:2] != second[:2]: raise ValueError('A/B must use the same hardware and calibration')
        self.cfg,self.cal=first[:2]
        self.bus=self.cameras=None
        self.armed=False
        self.permit=lambda:False
        self.guard=policy.control.TemperatureGuard(policy.control.TEMPERATURE_DURATION_S)
        self.phase=None;self.hold_pose=None;self.hold_since=None
        self.camera_since=self.camera_stable=self.camera_previous=None

    def _allow(self):
        if not self.permit(): raise MotionBlocked()

    def _deadline(self,tick):
        if time.monotonic()-tick > self.p.MAX_INFERENCE_SECONDS:
            raise RuntimeError('Arm1 observation/control deadline exceeded (250ms)')

    def _sample(self):
        values=self.p.control.feedback(self.bus,self.cal,self.guard)
        self.current={j:v['raw'] for j,v in values.items()}
        if self.armed:
            for j in self.p.control.JOINTS:
                if self.bus.read(j,'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost')
        return self.current

    def monitor(self):
        if self.bus is not None and self.armed:
            tick=time.monotonic();self._sample();self._deadline(tick)

    def _write(self,goals,tick,*,holding=False,speed=None):
        self._deadline(tick)
        if speed is None:speed=self.p.control.SERVO_PROFILE_SPEED
        for j in self.p.control.JOINTS:
            if not holding: self._allow()
            self.bus.write_position_profile(j,goals[j],speed)
        self._deadline(tick)

    def _freeze_at(self,raw,tick):
        self.hold_pose=dict(raw);self.hold_since=None
        self.return_profile_verified=False
        if self.armed: self._write(self.hold_pose,tick,holding=True)

    def freeze(self):
        if self.bus is not None and self.armed:
            tick=time.monotonic();self._freeze_at(self._sample(),tick)

    def hold_ready(self):
        if not self.armed: return True
        if not self.aligned(self.current,self.hold_pose,self.cal,2):
            self.hold_since=None;return False
        if self.hold_since is None:self.hold_since=time.monotonic()
        return time.monotonic()-self.hold_since >= .2

    def begin(self,command):
        _,_,self.pose,self.predictor,self.detector=self.models[command]
        self.phase=None;self.candidate_pose=None;self.arrival_since=None;self.home_started=None
        self.predictor.reset();self.detector.reset()
        if self.bus is None:
            self.bus=self.p.control.SerialBus(self.p.control.PortConfig(self.cfg['robot']['port'],'5B3D046080','robot'))
            self.bus.open()
            self.p.control.verify(self.bus,self.cal)
            self.cameras=self.p.Cameras(self.cfg,30,reconnect=True)
            self.cameras.start()
        # A second goal reuses the same bus and torque; never tears down between A/B.
        self._allow()
        tick=time.monotonic()
        raw=self._sample()
        frames=self.cameras.snapshot()
        self._deadline(tick)
        self.already_complete=False
        self.planner=self.p.control.TeachingTargets(self.cal,raw,self.p.control.load_motion())
        self.planner.syncing=False
        self.contact=self.Contact(self.cal['gripper'],self.planner.motion['gripper_relative_target_points'])
        return 'PRECHECK'

    def resume(self):
        if hasattr(self,'predictor'):
            self.predictor.reset();self.detector.reset()
        self.candidate_pose=None;self.arrival_since=None;self.home_started=None
        self.return_profile_verified=False
        # Recovering images must be fresh after the resume, not buffered pause frames.
        self.camera_since=time.monotonic() if self.cameras else None
        self.camera_stable=self.camera_previous=None

    def _frames(self,raw,tick):
        try: frames=self.cameras.snapshot()
        except self.p.CameraUnavailable:
            frames=None
        now=time.monotonic()
        if frames is None or self.camera_since is not None:
            if self.camera_since is None:
                self.camera_since=now;self._freeze_at(raw,tick)
            self.predictor.reset();self.detector.reset();self.candidate_pose=None
            if now-self.camera_since > 3: raise RuntimeError('Camera recovery timed out (3s)')
            if frames is None:
                self.camera_stable=self.camera_previous=None
            else:
                stamps=tuple(frames[n][1] for n in self.p.CAMERAS)
                if all(s>self.camera_since and now-s<=.1 for s in stamps):
                    if self.camera_previous is None or all(a>b for a,b in zip(stamps,self.camera_previous)):
                        if self.camera_stable is None:self.camera_stable=now
                        self.camera_previous=stamps
                        if now-self.camera_stable>=.3:
                            self.camera_since=self.camera_stable=self.camera_previous=None
                            return frames
                else:self.camera_stable=self.camera_previous=None
            return None
        return frames

    def step(self,phase):
        self._allow();tick=time.monotonic();raw=self._sample()
        frames=self._frames(raw,tick)
        if frames is None:return False
        if phase=='PRECHECK':
            # Confirm fresh images before moving or enabling torque. A completed
            # destination skips policy execution but still requires safe return.
            self.already_complete=self.detector.observe(*frames['overview'])
            return self.already_complete or not self.detector.last['candidate']
        if not self.armed:
            self._write(raw,tick)
            for j in self.p.control.JOINTS:
                if self.bus.read(j,'Goal_Velocity')!=0 or self.bus.read(j,'Goal_Time')!=0:
                    raise RuntimeError('Position profile readback mismatch')
            self._deadline(tick);self._allow()
            self.armed=True  # Cleanup covers partially enabled motors.
            for j in self.p.control.JOINTS:
                self._allow();self.bus.write(j,'Torque_Enable',1)
        if phase != self.phase:
            self.phase=phase;self.arrival_since=None;self.home_started=time.monotonic()
            self.return_profile_verified=False
            if phase=='RETURNING':print('완료 후 복귀 속도 제한: 약 5도/초 (56 steps/s).',flush=True)
            self.predictor.reset();self.detector.reset();self.candidate_pose=None
        if phase in ('HOMING','RETURNING'):
            if self.home_started is None:self.home_started=time.monotonic()
            timeout=self.p.control.RETURN_TIMEOUT_S if phase=='RETURNING' else 60
            if time.monotonic()-self.home_started>timeout: raise RuntimeError('Start pose movement timed out')
            requested=self.pose['raw']
        else:
            if self.detector.observe(*frames['overview']):
                self._allow();return True
            if self.detector.last['candidate']:
                if self.candidate_pose is None:self.candidate_pose=dict(raw)
                requested=self.candidate_pose
            else:
                if self.candidate_pose is not None:self.predictor.reset()
                self.candidate_pose=None
                requested=self.p.raw_action(self.predictor.predict(self.p.vector(raw,self.cal),frames),self.cal)
        self._allow();self._deadline(tick)
        # Fresh image and feedback checks immediately before bounded goals.
        raw=self._sample()
        try:self.cameras.snapshot()
        except self.p.CameraUnavailable:
            self._freeze_at(raw,tick);self.camera_since=time.monotonic()
            self.camera_stable=self.camera_previous=None
            self.predictor.reset();self.detector.reset();return False
        goals=self.planner.next(requested,raw)
        if phase=='WORKING':goals['gripper']=self.contact.next(goals['gripper'],raw['gripper'],time.monotonic())
        if phase=='RETURNING':
            speed=self.p.control.RETURN_PROFILE_SPEED
            self._write(goals,tick,speed=speed)
            if not self.return_profile_verified:
                for j in self.p.control.JOINTS:
                    if self.bus.read(j,'Goal_Velocity')!=speed or self.bus.read(j,'Goal_Time')!=0:
                        raise RuntimeError(f'{j}: return speed profile readback mismatch')
                self._deadline(tick);self.return_profile_verified=True
        else:self._write(goals,tick)
        if phase in ('HOMING','RETURNING'):
            if self.aligned(raw,requested,self.cal,2) and self.aligned(goals,requested,self.cal,2):
                if self.arrival_since is None:self.arrival_since=time.monotonic()
                return time.monotonic()-self.arrival_since >= .5
            self.arrival_since=None
        return False

    def close(self):
        errors=[]
        if self.bus is not None:
            if self.armed:errors.extend(self.p.release(self.bus))
            try:self.bus.close()
            except Exception as exc:errors.append(repr(exc))
            self.bus=None;self.armed=False
        if self.cameras is not None:
            try:self.cameras.close()
            except Exception as exc:errors.append(repr(exc))
            self.cameras=None
        if errors:raise RuntimeError('Torque-off/cleanup errors: '+repr(errors))
