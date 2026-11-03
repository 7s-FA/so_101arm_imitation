"""Arm3 direct teaching trial with synchronization using the same direct teaching limits and torque-off cleanup."""
import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import math
import tomllib
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'raspberry_pi_player'))
from diagnostic_bus import DiagnosticBus as SerialBus, LOW_VOLTAGE_POLICY
from player.program import JOINTS
from player.hardware import RETURN_SPEED_TICKS
from temperature_guard import TemperatureGuard

TEMPERATURE_LIMIT = 100
TEMPERATURE_DURATION_S = 5.0
SYNC_TIMEOUT_S = 60.0  # Arrival watchdog, not an estimate of physical travel time.
POSITION_LIMIT_POLICY = "encoder_0_4095_v1"
# Calibration endpoints remain coordinate references, not software target limits.

@dataclass
class PortConfig:
    port: str
    serial: str
    role: str


def mapped_goal(joint, raw, leader, follower, *, clip=True):
    lo, hi = follower['range_min'], follower['range_max']
    if joint == 'gripper':
        fraction = (raw - leader['range_min']) / (leader['range_max'] - leader['range_min'])
        target = lo + fraction * (hi-lo)
    else:
        target = raw - (leader['range_min'] + leader['range_max']) / 2 + (lo+hi) / 2
    return max(0, min(4095, round(target))) if clip else round(target)


SERVO_PROFILE_SPEED = 0  # STS3215: no fixed servo speed cap; software clips each target.
RETURN_PROFILE_SPEED = RETURN_SPEED_TICKS
RETURN_TIMEOUT_S = 120.0


def load_motion():
    with (ROOT / 'motion.toml').open('rb') as stream:
        values = tomllib.load(stream)['motion']
    maxima = {'body_relative_target_deg': 90, 'gripper_relative_target_points': 100,
              'control_hz': 30}
    if set(values) != set(maxima):
        raise ValueError('Invalid motion.toml fields')
    for key, maximum in maxima.items():
        value = values[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError(f'Invalid motion setting: {key}')
    return values


class TeachingTargets:
    """Use identical measured-position target limits for synchronization and teaching."""
    def __init__(self, calibration, reference, motion):
        self.calibration, self.motion = calibration, motion
        self.reference = dict(reference)
        self.syncing = True

    def scale(self, joint):
        c = self.calibration[joint]
        return (c['range_max'] - c['range_min']) / 100 if joint == 'gripper' else 4095 / 360

    def next(self, requested, measured):
        goals = {}
        for joint in JOINTS:
            scale = self.scale(joint)
            target = requested[joint]
            if self.syncing:
                if abs(target - self.reference[joint]) > 5 * scale:
                    raise RuntimeError('Leader moved during initial sync; hold the leader still and restart')
                target = self.reference[joint]
            relative = self.motion['gripper_relative_target_points' if joint == 'gripper' else 'body_relative_target_deg'] * scale
            candidate = max(math.ceil(measured[joint]-relative),
                            min(math.floor(measured[joint]+relative),round(target)))
            goals[joint] = max(0,min(4095,candidate))
        if self.syncing and all(abs(measured[j]-self.reference[j]) <= 2 * self.scale(j) for j in JOINTS):
            self.syncing = False
        return goals


def verify(bus, calibration):
    if set(calibration) != set(JOINTS):
        raise RuntimeError('Invalid calibration joints')
    for index, joint in enumerate(JOINTS, 1):
        c = calibration[joint]
        if c['id'] != index or not 0 <= c['range_min'] < c['range_max'] <= 4095 or c['drive_mode'] != 0:
            raise RuntimeError(f'{joint}: invalid calibration')
        for field, register in [('homing_offset','Homing_Offset'),('range_min','Min_Position_Limit'),('range_max','Max_Position_Limit')]:
            if bus.read(joint, register) != c[field]:
                raise RuntimeError(f'{joint}: calibration mismatch')
        if bus.read(joint, 'Model_Number') != 777 or bus.read(joint, 'Operating_Mode') != 0:
            raise RuntimeError(f'{joint}: unexpected model or operating mode')
        if bus.read(joint, 'Torque_Enable') != 0:
            raise RuntimeError(f'{joint}: torque already enabled')


def feedback(bus, calibration, temperature_guard, *, timing=None):
    data = {}
    for j in JOINTS:
        before = time.monotonic() if timing is not None else None
        data[j] = bus.feedback(j)
        if timing is not None: timing[j] = [before, time.monotonic()]
    for j, value in data.items():
        if value['status']:
            raise RuntimeError(f'{j}: status fault: {value}')
        temperature_guard.observe(j, value['temperature'], TEMPERATURE_LIMIT, active=True)
        raw = value['raw']
        role = getattr(getattr(bus, 'config', None), 'role', 'robot')
        if type(raw) is not int or not 0 <= raw <= 4095:
            raise RuntimeError(f'{role}/{j}: invalid single-turn position: {raw}')
    return data


class SessionStop(Exception):
    """An explicit session control request, including before activation."""


def run(move, seconds, *, observer=None, poll=None, start_pose=None, observation_reader=None):
    """Run until stopped; seconds=None disables the following time limit."""
    cfg = json.loads((ROOT/'devices.json').read_text())
    motion = load_motion()
    from gripper_contact import GripperContactGuard, POLICY as CONTACT_POLICY
    buses, cal = {}, {}
    temperature_guards = {role: TemperatureGuard(TEMPERATURE_DURATION_S)
                          for role in ('leader', 'robot')}
    for role in ('leader','robot'):
        item=cfg[role]
        path=Path(item['calibration_dir'])
        if not path.is_absolute(): path=ROOT/path
        cal[role]=json.loads((path/(item['id']+'.json')).read_text())
        serial = '5B61035944' if role=='leader' else '5B3D046080'
        buses[role]=SerialBus(PortConfig(item['port'],serial,role))
    contact_guard=GripperContactGuard(cal['robot']['gripper'],motion['gripper_relative_target_points'])
    report={'motion_requested':move,'servo_profile_speed':SERVO_PROFILE_SPEED,'motion':motion,'seconds':seconds,'cleanup_errors':[],
            'temperature_limit':TEMPERATURE_LIMIT,
            'temperature_duration_s':TEMPERATURE_DURATION_S,
            'position_limit_policy':POSITION_LIMIT_POLICY,
            'leader_gripper_low_voltage_policy':dict(LOW_VOLTAGE_POLICY)}
    out=ROOT/'logs'/('slow_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    out.parent.mkdir(exist_ok=True)
    motion_attempted=False
    samples=[]
    def stop(signum, frame): raise KeyboardInterrupt()
    old_sigterm = signal.signal(signal.SIGTERM,stop)
    old_sigint = signal.getsignal(signal.SIGINT)
    try:
        for role,bus in buses.items():
            bus.open()
            verify(bus,cal[role])
        leader,follower=buses['leader'],buses['robot']
        initial={role:feedback(bus,cal[role],temperature_guards[role]) for role,bus in buses.items()}
        goals={j:mapped_goal(j,initial['leader'][j]['raw'],cal['leader'][j],cal['robot'][j]) for j in JOINTS}
        delta={j:goals[j]-initial['robot'][j]['raw'] for j in JOINTS}
        report.update(initial=initial,initial_delta_ticks=delta)
        if start_pose is not None:
            from start_pose import validate
            validate(start_pose,cal['robot'])
            report['start_pose_delta_ticks']={j:start_pose['raw'][j]-initial['robot'][j]['raw'] for j in JOINTS}
            print('팔로워 → 저장된 시작 자세 오차 (ticks):',report['start_pose_delta_ticks'],flush=True)
        else:
            print('Initial target offsets (ticks):',delta,flush=True)
        if not move:
            report['outcome']='preflight_passed'
            return
        if start_pose is not None:
            from start_pose import validate, aligned, LEADER_START_TOLERANCE, relative_leader_goals
            validate(start_pose,cal['robot'])
            initial_mapped={j:mapped_goal(j,initial['leader'][j]['raw'],cal['leader'][j],cal['robot'][j],clip=False) for j in JOINTS}
            if not aligned(initial_mapped,start_pose['raw'],cal['robot'],LEADER_START_TOLERANCE):
                raise RuntimeError('리더 시작 자세가 허용 오차 ±100도(그리퍼 ±100포인트)를 벗어났습니다.')
        limit = 'unlimited' if seconds is None else f'{seconds:g}s'
        print(f'Starting in 5 seconds. Support arm; Ctrl+C stops. Limit: {limit}.',flush=True)
        countdown_end = time.monotonic() + 5
        while time.monotonic() < countdown_end:
            if poll is not None:
                requested_stop = poll()
                if requested_stop and requested_stop != 'resync_requested':
                    raise SessionStop(requested_stop)
            for role, bus in buses.items():
                feedback(bus, cal[role], temperature_guards[role])
            time.sleep(.1)
        # Hold each measured encoder position before direct synchronization.
        current=feedback(follower,cal['robot'],temperature_guards['robot'])
        motion_attempted=True
        for j in JOINTS:
            goal=current[j]['raw']
            follower.write_position_profile(j,goal,SERVO_PROFILE_SPEED)
            if follower.read(j,'Goal_Velocity')!=SERVO_PROFILE_SPEED or follower.read(j,'Goal_Time')!=0:
                raise RuntimeError(f'robot/{j}: speed profile readback mismatch')
        lp = feedback(leader,cal['leader'],temperature_guards['leader'])
        reference = {j:mapped_goal(j,lp[j]['raw'],cal['leader'][j],cal['robot'][j]) for j in JOINTS}
        for j in JOINTS: follower.write(j,'Torque_Enable',1)
        if start_pose is not None:
            from start_pose import move_to_pose
            def home_observe(fp,targets,tick):
                lp=feedback(leader,cal['leader'],temperature_guards['leader'])
                home_sample={'monotonic_s':tick,'follower':fp,'goal':targets}
                report['start_pose_last_sample']=home_sample
                if observer is not None:
                    observer({'t':0,'monotonic_s':tick,'leader':lp,'follower':fp,'goal':targets,'syncing':True})
            def prepare_start():
                move_to_pose(follower,cal['robot'],start_pose,temperature_guards['robot'],poll=poll,on_tick=home_observe)
                current_leader=feedback(leader,cal['leader'],temperature_guards['leader'])
                mapped={j:mapped_goal(j,current_leader[j]['raw'],cal['leader'][j],cal['robot'][j],clip=False) for j in JOINTS}
                if not aligned(mapped,start_pose['raw'],cal['robot'],LEADER_START_TOLERANCE):
                    raise RuntimeError('리더 시작 자세가 허용 오차 ±100도(그리퍼 ±100포인트)를 벗어났습니다.')
                anchor={j:current_leader[j]['raw'] for j in JOINTS}
                report['leader_start_anchor_raw']=anchor
                return anchor
            leader_anchor=prepare_start()
            reference=dict(start_pose['raw'])
            report['start_pose']=start_pose
            report['leader_start_tolerance']=LEADER_START_TOLERANCE
            report['leader_follow_mode']='relative_to_start_anchor'
            print('리더 시작 오차 보정 완료. 이후 리더가 움직인 만큼 팔로워가 따라갑니다.',flush=True)
        start=time.monotonic()
        planner = TeachingTargets(cal['robot'], reference, motion)
        print('SYNCING: hold leader still. Alignment uses the same target limits as direct teaching.',flush=True)
        previous_tick = time.monotonic()
        follow_elapsed = 0.0
        sync_deadline = previous_tick + SYNC_TIMEOUT_S
        report['sync_control'] = 'relative_start' if start_pose is not None else 'direct'
        report['sync_timeout_s'] = SYNC_TIMEOUT_S
        report['resync_count'] = 0
        from record_timing import next_deadline
        scheduled_tick = time.monotonic()
        period = 1 / motion['control_hz']
        while seconds is None or follow_elapsed < seconds:
            resync_requested = False
            if poll is not None:
                requested_stop = poll()
                if requested_stop == 'resync_requested':
                    resync_requested = True
                elif requested_stop:
                    report['outcome'] = requested_stop
                    break
            tick=time.monotonic()
            lp=feedback(leader,cal['leader'],temperature_guards['leader'])
            joint_reads = {}
            fp=feedback(follower,cal['robot'],temperature_guards['robot'],timing=joint_reads)
            goals={j:mapped_goal(j,lp[j]['raw'],cal['leader'][j],cal['robot'][j]) for j in JOINTS}
            if start_pose is not None:
                goals=relative_leader_goals({j:lp[j]['raw'] for j in JOINTS},leader_anchor,start_pose['raw'],cal['leader'],cal['robot'])
            measured = {j:fp[j]['raw'] for j in JOINTS}
            if resync_requested:
                if start_pose is not None:
                    leader_anchor=prepare_start()
                    goals=dict(start_pose['raw'])
                    joint_reads = {}
                    fp=feedback(follower,cal['robot'],temperature_guards['robot'],timing=joint_reads)
                    measured={j:fp[j]['raw'] for j in JOINTS}
                    tick=time.monotonic()
                planner = TeachingTargets(cal['robot'], goals, motion)
                sync_deadline = tick + SYNC_TIMEOUT_S
                previous_tick = tick
                report['resync_count'] += 1
                print('RESYNCING: hold leader still; no recording during synchronization.',flush=True)
            if planner.syncing and tick > sync_deadline:
                raise RuntimeError('Synchronization timed out; check obstruction or tracking failure')
            was_syncing = planner.syncing
            if not was_syncing:
                follow_elapsed += tick - previous_tick
            goals = planner.next(goals, measured)
            goals['gripper']=contact_guard.next(goals['gripper'],measured['gripper'],time.monotonic())
            previous_tick = tick
            if was_syncing and not planner.syncing:
                print('FOLLOWING: initial sync complete. You may move the leader.',flush=True)
            camera_snapshot = observation_reader() if observation_reader is not None else None
            action_sends = {}
            for j in JOINTS:
                if follower.read(j,'Torque_Enable')!=1:
                    raise RuntimeError(f'{j}: torque lost')
                send_start = time.monotonic()
                follower.write_position_profile(j,goals[j],SERVO_PROFILE_SPEED)
                action_sends[j] = [send_start, time.monotonic()]
            sample = {'t':time.monotonic()-start,'monotonic_s':tick,'leader':lp,'follower':fp,'goal':goals,'syncing':planner.syncing,
                      'timing':{'joint_read_monotonic_s':joint_reads,'action_send_monotonic_s':action_sends,
                                'observation_before_action':observation_reader is not None}}
            samples.append(sample)
            if observer is not None:
                observer({**sample,'camera_snapshot':camera_snapshot})
            scheduled_tick = next_deadline(scheduled_tick,time.monotonic(),period)
            time.sleep(max(0,scheduled_tick-time.monotonic()))
        if planner.syncing and not report.get('outcome'):
            raise RuntimeError('Trial ended before initial synchronization completed')
        report['following_elapsed_s'] = follow_elapsed
        report.setdefault('outcome', 'completed')
    except SessionStop as exc:
        report['outcome'] = str(exc)
    except KeyboardInterrupt:
        report['outcome']='interrupted'
    except Exception as exc:
        report['outcome']='error'
        report['error']=repr(exc)
        print('STOP:',repr(exc),flush=True)
    finally:
        if motion_attempted:
            # Ignore repeated interrupts while making a best effort to turn every motor off.
            signal.signal(signal.SIGINT,signal.SIG_IGN)
            signal.signal(signal.SIGTERM,signal.SIG_IGN)
            for j in JOINTS:
                try:
                    buses['robot'].write(j,'Torque_Enable',0)
                    if buses['robot'].read(j,'Torque_Enable')!=0:
                        raise RuntimeError('Torque-off readback failed')
                except Exception as exc: report['cleanup_errors'].append(f'{j}: {exc}')
        for role,bus in buses.items():
            try: bus.close()
            except Exception as exc: report['cleanup_errors'].append(f'{role}: {exc}')
        report['motor_faults'] = {role: list(getattr(bus, 'fault_events', [])) for role, bus in buses.items()}
        report['gripper_contact_policy']=dict(CONTACT_POLICY)
        report['gripper_contact_events']=contact_guard.events
        report['temperature_pending'] = {role: guard.snapshot() for role, guard in temperature_guards.items()}
        report['samples']=samples
        out.write_text(json.dumps(report,indent=2)+'\n')
        print('Outcome:',report.get('outcome'),'cleanup errors:',report['cleanup_errors'], '\nLog:',out,flush=True)
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
    if report.get('outcome')=='error' or report['cleanup_errors']: raise SystemExit(1)
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true',help='Enable torque and run a 45-second trial; default is read-only')
    args=parser.parse_args()
    run(args.run,45)
