"""Model/hardware adapter. Only Controller's worker may call hardware methods."""
import json
import os
from pathlib import Path
import sys
import time

from engine import MotionBlocked,JobRejected

WORK = Path(__file__).resolve().parents[2]
# The IK retarget child used ~1.06 GB on the Jetson (2026-10-27). Together with the server (~1.7 GB) it
# exhausted memory and the watchdog reset the system; below this the job is refused instead.
RETARGET_MIN_AVAILABLE_MB = 1500


def available_mb():
    """MemAvailable in MB (excludes swap, which is too slow on the SD card); None if unreadable."""
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                if line.startswith('MemAvailable:'):return int(line.split()[1])//1024
    except (OSError,ValueError):pass
    return None


def load_map(path):
    config = json.loads(Path(path).read_text())
    if config.get('version') != 1 or set(config['models']) != {'SELECT_A','SELECT_B'}:
        raise ValueError('Expected SELECT_A/SELECT_B model map version 1')
    if not 30 <= config['job_timeout_seconds'] <= 3600:
        raise ValueError('Invalid job watchdog')
    for command, spec in config['models'].items():
        project = (WORK/'projects'/spec['project']).resolve()
        if project.parent != (WORK/'projects').resolve() or not (project/'project.json').is_file():
            raise ValueError(f'{command}: invalid project')
        for key in ('model','completion'):
            path = (WORK/spec[key]).resolve()
            if not path.is_relative_to(project): raise ValueError(f'{command}: {key} outside project')
            if not path.exists(): raise FileNotFoundError(path)
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
        # D435 set selection (vision-map.json): material check before any motion; ERROR result if missing.
        from vision_judge import VisionJudge
        from plan_runner import PlanRunner
        self.judge=VisionJudge();self.vision_job=None;self.PlanRunner=PlanRunner;self.plan=None
        # Verified real plans per vision set (model-map.json "plans"); other sets keep the imitation model.
        self.plan_paths={key:Path(path) for key,path in config.get('plans',{}).items()}
        for key,path in self.plan_paths.items():
            if key not in self.judge.config['sets']:raise ValueError(f'plans: unknown set {key}')
            runner=PlanRunner(path,self.cal)
            print(f'{key} ({self.judge.config["sets"][key]["label"]}): 정확한 동작 계획 {runner.frames}프레임 {runner.seconds:.0f}초 | {path}',flush=True)
        # Burger (target pallet) correction: measured before each plan job; verified plans of known burgers.
        self.burger_cfg=config.get('burger_correction') or {};self.burger_state=None;self.jig_cells=None
        if self.burger_cfg.get('enabled'):
            from jig_cells import JigCells
            self.jig_cells=JigCells(self.burger_cfg,self.judge.config['cells'])
            for burger,plans in self.burger_cfg.get('anchor_plans',{}).items():
                for key,path in plans.items():
                    runner=PlanRunner(path,self.cal);print(f'{key} {burger} 버거 검증 계획 {runner.frames}프레임 {runner.seconds:.0f}초 | {path}',flush=True)
            print('버거 위치 보정: 켜짐 (작업마다 목적지 팔레트 측정)',flush=True)

    def _burger_plan(self,key,rgb, *, stamp=None, prepare=True):
        """Plan for the selected set at the measured target-pallet pose; None while measuring or retargeting.
        At the m2 reference: the base plan. At a verified burger (anchors): its verified plan. Elsewhere: the base
        plan retargeted per part in a separate process (the 30Hz worker keeps cycling; nothing moves yet)."""
        base=self.plan_paths[key];cfg=getattr(self,'burger_cfg',None) or {}
        if not cfg.get('enabled') or getattr(self,'jig_cells',None) is None:return base
        import cv2
        from jig_cells import JigCells
        st=self.burger_state
        if st is None:st=self.burger_state={'measurements':[],'failures':0,'first_failure':None,'thread':None,'result':None,'error':None,'info':{'mode':'측정 중'},'last_stamp':None}
        if st['thread'] is None:
            if stamp is not None and stamp == st.get('last_stamp'):
                return None  # Never count the same buffered camera frame twice.
            st['last_stamp']=stamp
            m=self.jig_cells.measure(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            if not m['ok']:
                # A shaken camera or a weak cell can pass again a moment later: keep measuring fresh frames for
                # measure_wait_s under the same limits. Only frames after the last failure may form the plan.
                now=time.monotonic();st['first_failure']=st['first_failure'] if st['first_failure'] is not None else now
                st['failures']+=1;st['measurements']=[];st['info']={'mode':'측정 실패','reason':m['reason']}
                if st['failures']>=cfg.get('max_failures',5) and now-st['first_failure']>=cfg.get('measure_wait_s',0):
                    raise JobRejected('JIG_MEASURE_FAILED: '+m['reason'])
                return None
            st['measurements'].append(m)
            if not prepare or len(st['measurements'])<cfg.get('frames',3):return None
            med=JigCells.median(st['measurements'])
            info=st['info']={'translation_mm':med['rigid']['translation_mm'],'rotation_deg':med['rigid']['rotation_deg'],'pivot_xy_m':med['rigid']['pivot_xy_m'],
                              'cells':{p:c['offset_mm'] for p,c in med['cells'].items()},'cells_px':{p:c['shift_px'] for p,c in med['cells'].items()},'camera':med.get('camera')}
            if self.jig_cells.at_reference(med):info['mode']='m2 기준 계획';return base
            anchor=self.jig_cells.match_anchor(med)
            if anchor and key in cfg.get('anchor_plans',{}).get(anchor,{}):info['mode']=f'{anchor} 검증 계획';return Path(cfg['anchor_plans'][anchor][key])
            alpha,extra=self.jig_cells.extra_mm(med);info.update(mode='보정 계획 생성 중',alpha=round(alpha,3),extra_mm=extra)
            need=cfg.get('retarget_min_available_mb',RETARGET_MIN_AVAILABLE_MB);free=available_mb()
            if free is not None and free<need:
                raise JobRejected(f'RETARGET_LOW_MEMORY: 사용 가능 메모리 {free}MB < {need}MB (IK 계산 약 1.1GB 필요). 다른 프로그램을 닫고 다시 요청하세요')
            import json as _json,subprocess,threading
            work=Path(__file__).resolve().parent/'logs'/'burger'/f"{time.strftime('%Y%m%d_%H%M%S')}_{key}";work.mkdir(parents=True,exist_ok=True)
            (work/'offsets.json').write_text(_json.dumps(dict(med,burger='measured'),ensure_ascii=False));(work/'extra.json').write_text(_json.dumps(extra))
            # Per set: {part: [x, y] mm} the opened gripper moves before lifting (real_plan_burger --retreat).
            retreat=cfg.get('release_retreat_mm',{}).get(key)
            if retreat:(work/'retreat.json').write_text(_json.dumps(retreat));info['release_retreat_mm']=retreat
            def build():
                try:
                    r=subprocess.run([cfg.get('retarget_python') or sys.executable,cfg['retarget_script'],str(base),'--set',key,'--offsets',str(work/'offsets.json'),
                                      '--extra',str(work/'extra.json'),'--output',str(work/'plan.json')]+(['--retreat',str(work/'retreat.json')] if retreat else []),
                                     capture_output=True,text=True,timeout=cfg.get('retarget_timeout_s',90))
                    (work/'retarget.log').write_text(r.stdout+r.stderr)
                    if r.returncode or not (work/'plan.json').exists():st['error']=(r.stderr or r.stdout)[-300:]
                    else:st['result']=work/'plan.json'
                except Exception as exc:st['error']=repr(exc)
            st['thread']=threading.Thread(target=build,daemon=True);st['thread'].start();return None
        if st['thread'].is_alive():return None
        if st['error']:raise JobRejected('JIG_RETARGET_FAILED: '+st['error'])
        st['info']['mode']='보정 계획';st['info']['plan']=str(st['result']);return st['result']

    def _allow(self):
        if not self.permit(): raise MotionBlocked()

    def _deadline(self,tick,*,context='control'):
        elapsed=time.monotonic()-tick
        limit=self.p.MAX_INFERENCE_SECONDS
        if elapsed > limit:
            # Keep the watchdog unchanged; distinguish idle serial reads from
            # inference/camera/control work without logging every 30 Hz cycle.
            detail={'context':context,'phase':getattr(self,'phase',None),'elapsed_ms':round(elapsed*1000,2),
                    'limit_ms':round(limit*1000,2),
                    'last_sample':getattr(self,'sample_timing',None)}
            raise RuntimeError(f'Arm1 observation/control deadline exceeded ({limit*1000:g}ms): '
                               +json.dumps(detail,ensure_ascii=False))

    def _sample(self):
        tick=time.monotonic();feedback_timing={}
        values=self.p.control.feedback(self.bus,self.cal,self.guard,timing=feedback_timing)
        feedback_end=time.monotonic();torque_timing={}
        self.current={j:v['raw'] for j,v in values.items()}
        if self.armed:
            for j in self.p.control.JOINTS:
                before=time.monotonic()
                if self.bus.read(j,'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost')
                torque_timing[j]=round((time.monotonic()-before)*1000,2)
        self.sample_timing={'feedback_ms':round((feedback_end-tick)*1000,2),
                            'torque_ms':round((time.monotonic()-feedback_end)*1000,2),
                            'feedback_joints_ms':{j:round((end-start)*1000,2)
                                                 for j,(start,end) in feedback_timing.items()},
                            'torque_joints_ms':torque_timing}
        return self.current

    def monitor(self):
        if self.bus is not None and self.armed:
            tick=time.monotonic();self._sample();self._deadline(tick,context='monitor')

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

    def start_cameras(self):
        """Open the cameras only (no SerialBus, torque or motor writes) so the judge/GUI see the scene before a goal."""
        if self.cameras is None:
            self.cameras=self.p.Cameras(self.cfg,30,reconnect=True)
            self.cameras.start()

    def clear_recovery_state(self):
        """Discard recovery timers only after engine has verified cleanup/hold."""
        self.camera_since=self.camera_stable=self.camera_previous=None

    def begin(self,command):
        # A new goal gets its own camera window. Never inherit a paused/error timer.
        self.clear_recovery_state()
        home_only=command in ('HOME_A','HOME_B')
        model_command='SELECT_'+command[-1] if home_only else command
        _,_,self.pose,self.predictor,self.detector=self.models[model_command]
        self.phase=None;self.candidate_pose=None;self.arrival_since=None;self.home_started=None
        self.predictor.reset();self.detector.reset()
        self.command=command;self.select_votes=[];self.select_stamp=None;self.vision_job=None;self.plan=None;self.done_since=None;self.burger_state=None
        if self.bus is None:
            self.bus=self.p.control.SerialBus(self.p.control.PortConfig(self.cfg['robot']['port'],'5B3D046080','robot'))
            self.bus.open()
            self.p.control.verify(self.bus,self.cal)
        self.start_cameras()
        # Camera startup has its own bounded wait; freshness starts afterwards.
        self.camera_since=time.monotonic()
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
        # Reuse the measured, bounded slow return. No task completion or policy
        # inference is executed for an explicit startup-home request.
        return 'RETURNING' if home_only else 'PRECHECK'

    def resume(self):
        if getattr(self,'plan',None) is not None:self.plan.pause();self.done_since=None
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
            judge=getattr(self,'judge',None)
            if judge is not None:
                # Three consecutive fresh frames must agree on the set before any torque or motion.
                result=judge.evaluate(frames['overview'][0]);selection=judge.select(self.command,result)
                self.vision_job={'command':self.command,'time':time.time(),'selection':selection,'sets':result['sets'],'targets':result['targets']}
                stamp=frames['overview'][1]
                if stamp==getattr(self,'select_stamp',None):return False
                self.select_stamp=stamp
                vote=(selection['set'],selection['error'])
                if self.select_votes and self.select_votes[-1]!=vote:
                    self.select_votes=[]
                    self.burger_state=None  # Discard measurements for the previous selection.
                self.select_votes=(self.select_votes+[vote])[-3:]
                # Measure the same fresh frames while collecting material votes;
                # do not select or generate a plan before three agreeing votes.
                path=None
                if (not selection['error'] and getattr(self,'plan',None) is None
                        and selection['set'] in getattr(self,'plan_paths',{})
                        and getattr(self,'burger_cfg',{}).get('enabled')
                        and getattr(self,'jig_cells',None) is not None):
                    path=self._burger_plan(selection['set'],frames['overview'][0],stamp=stamp,prepare=len(self.select_votes)>=3)
                if len(self.select_votes)<3:return False
                if selection['error']:raise JobRejected(f"{selection['error']}: {selection['reason']}")
                if getattr(self,'plan',None) is None and selection['set'] in getattr(self,'plan_paths',{}):
                    if not (getattr(self,'burger_cfg',{}).get('enabled') and getattr(self,'jig_cells',None) is not None):
                        path=self._burger_plan(selection['set'],frames['overview'][0])
                    if path is None:return False          # still measuring the pallet or building its plan
                    self.plan=self.PlanRunner(path,self.cal)
                if getattr(self,'plan',None) is not None:self.vision_job['plan_path']=str(self.plan.path)   # the GUI draws this plan's path
                if getattr(self,'burger_state',None):self.vision_job['burger']=self.burger_state['info']
                self.vision_job['executor']=('plan: '+self.plan.path.name) if getattr(self,'plan',None) else 'imitation model'
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
            if phase=='RETURNING':print('완료 후 복귀 속도 제한: 약 10도/초 (112 steps/s).',flush=True)
            self.predictor.reset();self.detector.reset();self.candidate_pose=None
        if phase in ('HOMING','RETURNING'):
            if self.home_started is None:self.home_started=time.monotonic()
            timeout=self.p.control.RETURN_TIMEOUT_S if phase=='RETURNING' else 60
            if time.monotonic()-self.home_started>timeout: raise RuntimeError('Start pose movement timed out')
            requested=self.plan.start if phase=='HOMING' and getattr(self,'plan',None) else self.pose['raw']
        elif getattr(self,'plan',None) is not None:
            requested=self._plan_step(raw,frames)
            if requested is True:self._allow();return True
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
        if phase=='WORKING' and getattr(self,'plan',None) is None:
            goals['gripper']=self.contact.next(goals['gripper'],raw['gripper'],time.monotonic())
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

    def _plan_step(self,raw,frames):
        """Next plan goal; True once the last frame is reached (placement is verified after the return)."""
        requested,frame=self.plan.next(raw,time.monotonic())
        self.vision_job['plan_frame']=[frame,self.plan.frames]
        return True if self.plan.done else requested

    @property
    def verify_after_return(self):
        # The arm hides destination cells at the plan end and the blue fingers look like the blue cylinder,
        # so a plan job is judged only after the return, with the arm away from the jigs.
        return getattr(self,'plan',None) is not None

    def verify_placement(self):
        """After the return: True when 3 consecutive fresh frames show the selected set moved and its destination
        filled, False when that is not seen within 5s, None while still looking. Read only."""
        now=time.monotonic()
        if self.done_since is None:self.done_since=now;self.verify_votes=0;self.verify_last=None
        try:rgb,stamp=self.cameras.snapshot()['overview']
        except self.p.CameraUnavailable:rgb=None
        if rgb is not None and stamp!=self.verify_last and now-stamp<=.1:
            self.verify_last=stamp
            key=self.vision_job['selection']['set'];product=key[0];result=self.judge.evaluate(rgb)
            moved=not any(result['cells'][c]['present'] for c in self.judge.config['sets'][key]['cells'])
            filled=len(result['targets'][product]['filled']);placed=filled==result['targets'][product]['total']
            self.vision_job['verified']={'moved':moved,'filled':filled}
            self.verify_votes=self.verify_votes+1 if moved and placed else 0
            if self.verify_votes>=3:return True
        if now-self.done_since>5:
            v=self.vision_job.get('verified') or {}
            self.verify_failure=f"PLACEMENT_NOT_CONFIRMED: 원래 칸 비움={v.get('moved')}, 목적지 {v.get('filled')}/3"
            return False
        return None

    def reset_resources(self):
        """Worker-only explicit recovery: release owned resources, then verify motors."""
        self.close()
        from resource_cleanup import release_camera_preview
        release_camera_preview(WORK)
        self.recover_cleanup()

    def recover_cleanup(self):
        """Verify restored hardware without disabling/enabling torque or moving."""
        # The cameras stay open for the vision judge/GUI from server start (start_cameras) and hold no torque;
        # only an open motor bus or an armed robot is an unclosed hardware session.
        if self.bus is not None or self.armed:
            raise RuntimeError('Existing hardware session has not closed; inspect locally')
        bus = self.p.control.SerialBus(self.p.control.PortConfig(self.cfg['robot']['port'],'5B3D046080','robot'))
        try:
            bus.open()
            # verify checks every motor's identity, calibration and torque OFF.
            self.p.control.verify(bus, self.cal)
            self.p.control.feedback(bus, self.cal, self.guard)
            self.p.control.verify(bus, self.cal)
        finally:
            bus.close()

    def close(self):
        errors=[]
        if self.bus is not None:
            # Retain failed handles so recovery cannot silently forget ownership.
            released=True
            if self.armed:
                try:
                    failures=self.p.release(self.bus)
                    errors.extend(failures)
                    released=not failures
                except Exception as exc:
                    errors.append(repr(exc));released=False
            if released:
                self.armed=False
                try:self.bus.close()
                except Exception as exc:errors.append(repr(exc))
                else:self.bus=None
        if self.cameras is not None:
            try:
                self.cameras.close()
                if any(t.is_alive() for t in self.cameras.threads):
                    raise RuntimeError('Camera worker still owns device after close; recovery blocked')
            except Exception as exc:errors.append(repr(exc))
            else:self.cameras=None
        self.clear_recovery_state()
        if errors:raise RuntimeError('Torque-off/cleanup errors: '+repr(errors))
