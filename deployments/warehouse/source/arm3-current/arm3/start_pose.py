"""Save a read-only follower pose and move to a validated pose under supervision."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import time

import teleoperate_slow as control
from project_paths import project_path

ROOT = Path(__file__).resolve().parent
PROFILE = project_path('start_pose.json', ROOT)
LEADER_START_TOLERANCE = 100.0  # Body degrees; gripper calibrated percentage points.


def relative_leader_goals(raw, anchor, target, leader_cal, follower_cal):
    """Cancel initial pose differences; only subsequent leader motion is followed."""
    goals={}
    for j in control.JOINTS:
        delta=raw[j]-anchor[j]
        if j=='gripper':
            delta *= ((follower_cal[j]['range_max']-follower_cal[j]['range_min']) /
                      (leader_cal[j]['range_max']-leader_cal[j]['range_min']))
        goals[j]=max(0,min(4095,round(target[j]+delta)))
    return goals


def calibration_hash(cal):
    return hashlib.sha256(json.dumps(cal,sort_keys=True).encode()).hexdigest()


def validate(profile, cal):
    if profile.get('version') != 1 or profile.get('calibration_sha256') != calibration_hash(cal):
        raise ValueError('Start pose calibration mismatch; save the pose again after calibration')
    if profile.get('joint_order') != list(control.JOINTS):
        raise ValueError('Start pose joint order mismatch')
    raw = profile.get('raw', {})
    if set(raw) != set(control.JOINTS) or any(type(v) is not int or not 0 <= v <= 4095 for v in raw.values()):
        raise ValueError('Invalid saved start pose encoder values')
    return profile


def load(path, cal):
    path = Path(path)
    return validate(json.loads(path.read_text()),cal) if path.exists() else None


def aligned(raw, target, cal, tolerance=2.0):
    return all(abs(raw[j]-target[j]) <= tolerance * (
        (cal[j]['range_max']-cal[j]['range_min'])/100 if j=='gripper' else 4095/360)
        for j in control.JOINTS)


def move_to_pose(bus, cal, profile, guard, *, poll=None, on_tick=None, returning=False):
    """Caller enables torque at the measured pose and always owns cleanup.

    Initial positioning uses teaching limits. Completion return additionally
    sets a ~5 deg/s servo speed cap, with readback and a 120s watchdog.
    """
    target = validate(profile,cal)['raw']
    motion = dict(control.load_motion())
    speed = control.RETURN_PROFILE_SPEED if returning else control.SERVO_PROFILE_SPEED
    timeout = control.RETURN_TIMEOUT_S if returning else 60.0
    profile_verified = False
    limiter = control.TeachingTargets(cal,target,motion)
    limiter.syncing = False
    start = time.monotonic()
    stable_since = None
    next_status = start
    remaining = {}
    print('저장된 시작 자세로 이동 중. 이동 경로를 비워 주세요. q / Esc / Ctrl+C: 중지.',flush=True)
    if returning: print('완료 후 복귀 속도 제한: 약 10도/초 (112 steps/s).',flush=True)
    while True:
        if poll is not None:
            stop = poll()
            if stop: raise control.SessionStop(stop if isinstance(stop,str) else 'user_stopped')
        tick = time.monotonic()
        if tick-start > timeout:
            raise RuntimeError(f'Start pose movement timed out; remaining degrees/gripper points: {remaining}')
        values = control.feedback(bus,cal,guard)
        current = {j:v['raw'] for j,v in values.items()}
        remaining={j:round((target[j]-current[j])/limiter.scale(j),2) for j in control.JOINTS}
        if tick >= next_status:
            print('시작 자세까지 남은 오차 (도 / 그리퍼 포인트):',remaining,flush=True)
            next_status=tick+2
        goals = limiter.next(target,current)
        for j in control.JOINTS:
            if bus.read(j,'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost during start pose movement')
        if on_tick is not None and on_tick(values,goals,tick) is False:
            stable_since = None
            profile_verified = False  # Recovery hold may have reset servo speed.
            continue  # Callback paused for recovery; discard pre-pause feedback/goals.
        if time.monotonic()-tick > .25: raise RuntimeError('Start pose monitoring deadline exceeded')
        if poll is not None:
            stop = poll()
            if stop: raise control.SessionStop(stop if isinstance(stop,str) else 'user_stopped')
        for j in control.JOINTS: bus.write_position_profile(j,goals[j],speed)
        if returning and not profile_verified:
            for j in control.JOINTS:
                if bus.read(j,'Goal_Velocity') != speed or bus.read(j,'Goal_Time') != 0:
                    raise RuntimeError(f'{j}: return speed profile readback mismatch')
            if time.monotonic()-tick > .25: raise RuntimeError('Return speed verification deadline exceeded')
            profile_verified = True
        if aligned(current,target,cal,2) and aligned(goals,target,cal,2):
            if stable_since is None: stable_since=tick
            if tick-stable_since >= .5:
                print('시작 자세 도착 확인.',flush=True)
                return
        else: stable_since=None
        time.sleep(max(0,1/motion['control_hz']-(time.monotonic()-tick)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['save','show'])
    parser.add_argument('--replace',action='store_true',help='Replace the saved pose only before any new dataset is created')
    args=parser.parse_args()
    cfg=json.loads((ROOT/'devices.json').read_text())
    item=cfg['robot']
    directory=Path(item['calibration_dir'])
    if not directory.is_absolute(): directory=ROOT/directory
    cal=json.loads((directory/(item['id']+'.json')).read_text())
    if args.command=='show':
        profile=load(PROFILE,cal)
        if profile is None: raise SystemExit('저장된 자세가 없습니다. bash start-pose.sh save 로 저장하세요.')
        print(json.dumps(profile,ensure_ascii=False,indent=2))
        return
    task=json.loads((project_path('purple_task.json', ROOT)).read_text())
    dataset=ROOT/task['dataset_directory']
    dataset.parent.mkdir(exist_ok=True)
    with (dataset.parent/(dataset.name+'.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if dataset.exists(): raise SystemExit('시연 데이터가 이미 있습니다. 같은 데이터셋의 시작 자세는 변경할 수 없습니다.')
        if PROFILE.exists() and not args.replace: raise SystemExit('시작 자세가 이미 있습니다. 녹화 전 변경은 --replace 를 사용하세요.')
        bus=control.SerialBus(control.PortConfig(item['port'],'5B3D046080','robot'))
        try:
            bus.open()
            control.verify(bus,cal)  # Includes torque-off verification; no motor writes.
            guard=control.TemperatureGuard(control.TEMPERATURE_DURATION_S)
            samples=[]
            for _ in range(5):
                samples.append({j:v['raw'] for j,v in control.feedback(bus,cal,guard).items()})
                time.sleep(.1)
            if any(not aligned(s,samples[0],cal,1) for s in samples):
                raise RuntimeError('팔이 움직이고 있습니다. 원하는 시작 자세로 받쳐 고정한 뒤 다시 저장하세요.')
            profile={'version':1,'created_at':datetime.now().astimezone().isoformat(),
                     'calibration_sha256':calibration_hash(cal),'joint_order':list(control.JOINTS),
                     'raw':samples[-1]}
            temporary=PROFILE.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
            temporary.replace(PROFILE)
            print('시작 자세 저장 완료 (모터 쓰기 없음):',PROFILE,flush=True)
        finally: bus.close()


if __name__=='__main__': main()
