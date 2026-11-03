"""Local ACT deployment. Default: offline model check; --check: read-only hardware; --run: motion."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import select
import signal
import sys
import time

import numpy as np
import teleoperate_slow as control
from project_paths import project_path
from record_purple import Cameras, CameraUnavailable, CAMERAS, identity, vector, terminal_keys
from roi_mask import load as load_roi, mask_rgb

ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT.parent / 'outputs/model1/checkpoints/100000/pretrained_model'
MAX_INFERENCE_SECONDS = .25


def raw_action(action, calibration):
    action = np.asarray(action)
    if action.shape != (6,) or not np.isfinite(action).all():
        raise ValueError('Policy action must contain six finite values')
    result = {}
    for joint, value in zip(control.JOINTS, action):
        c = calibration[joint]
        raw = (c['range_min'] + float(value) * (c['range_max']-c['range_min']) / 100
               if joint == 'gripper' else
               (c['range_min']+c['range_max']) / 2 + float(value) * 4095 / 360)
        if not -.001 <= raw <= 4095.001:
            raise ValueError(f'{joint}: prediction outside encoder range: {raw}')
        result[joint] = round(raw)
    return result


def model_dataset(model):
    train = json.loads((model/'train_config.json').read_text())
    dataset_map = model/'deployment_dataset.json'
    dataset = Path(json.loads(dataset_map.read_text())['dataset_root'] if dataset_map.exists()
                   else train['dataset']['root'])
    return dataset if dataset.is_absolute() else (model / dataset).resolve()


def load_context(model):
    cfg = json.loads((ROOT/'devices.json').read_text())
    task = json.loads((project_path('purple_task.json', ROOT)).read_text())
    # Archived deployments can retain their original training config verbatim
    # while resolving a dataset that was moved alongside model renaming.
    dataset = model_dataset(model)
    saved = json.loads((dataset/'arm3_identity.json').read_text())
    if saved != identity(cfg, task):
        raise ValueError('Training dataset identity differs from current camera/calibration/task configuration')
    info = json.loads((dataset/'meta/info.json').read_text())
    if info['fps'] != control.load_motion()['control_hz']:
        raise ValueError('Training FPS and control frequency differ')
    for key in ('observation.state', 'action'):
        if info['features'][key]['names'] != [j+'.pos' for j in control.JOINTS]:
            raise ValueError('Training joint order mismatch')
    directory = Path(cfg['robot']['calibration_dir'])
    if not directory.is_absolute(): directory = ROOT/directory
    cal = json.loads((directory/(cfg['robot']['id']+'.json')).read_text())
    return cfg, cal


class Predictor:
    def __init__(self, model, device, n_action_steps=None):
        import torch
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        self.torch = torch
        roi_path = Path(model)/'roi_mask.json'
        self.roi = load_roi(roi_path) if roi_path.is_file() else None
        print('모델 영상 영역:', json.dumps(self.roi, ensure_ascii=False) if self.roi else '전체 영상', flush=True)
        torch.set_num_threads(2)
        cfg = ACTConfig.from_pretrained(str(model), local_files_only=True)
        if n_action_steps is not None:
            if not 1 <= n_action_steps <= cfg.chunk_size:
                raise ValueError(f'--n-action-steps must be between 1 and {cfg.chunk_size}')
            if cfg.temporal_ensemble_coeff is not None:
                raise ValueError('--n-action-steps is only supported without temporal ensembling')
            cfg.n_action_steps = n_action_steps  # Runtime only; checkpoint stays unchanged.
        print(f'동작 재예측: {cfg.n_action_steps}동작마다'
              if cfg.temporal_ensemble_coeff is None else '동작 재예측: 매 동작 (temporal ensemble)', flush=True)
        expected = {'observation.state': (6,), **{f'observation.images.{n}': (3,480,640) for n in CAMERAS}}
        if {k: tuple(v.shape) for k,v in cfg.input_features.items()} != expected:
            raise ValueError('Unexpected model observation schema')
        if tuple(cfg.output_features['action'].shape) != (6,):
            raise ValueError('Unexpected model action schema')
        cfg.device = device
        cfg.pretrained_backbone_weights = None  # Complete backbone already in checkpoint; never download.
        self.policy = ACTPolicy.from_pretrained(str(model), config=cfg, local_files_only=True).to(device).eval()
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=str(model),
            preprocessor_overrides={'device_processor': {'device': device}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'}})

    def reset(self):
        self.policy.reset()

    def predict(self, state, frames):
        torch = self.torch
        obs = {'observation.state': torch.as_tensor(state, dtype=torch.float32)}
        for name in CAMERAS:
            rgb = frames[name][0]
            if self.roi: rgb = mask_rgb(rgb, name, self.roi)
            obs[f'observation.images.{name}'] = torch.from_numpy(rgb.copy()).permute(2,0,1).float()/255
        with torch.inference_mode():
            action = self.post(self.policy.select_action(self.pre(obs)))
        result = action.squeeze(0).cpu().numpy()
        if result.shape != (6,) or not np.isfinite(result).all():
            raise ValueError('Invalid model output')
        return result


def stop_requested():
    if select.select([sys.stdin], [], [], 0)[0]:
        key = sys.stdin.read(1)
        return not key or key.lower() == 'q' or key == '\x1b'
    return False


def release(bus):
    errors = []
    for joint in control.JOINTS:
        try:
            bus.write(joint, 'Torque_Enable', 0)
            if bus.read(joint, 'Torque_Enable') != 0:
                raise RuntimeError('Torque-off readback failed')
        except Exception as exc:
            errors.append(f'{joint}: {exc}')
    return errors


class CompletedControls:
    """Completion-only buttons, with a terminal fallback when no desktop exists."""
    def __init__(self):
        self.restart = False
        self.quit = False
        self.window = None
        try:
            import tkinter as tk
            from gui_text import KoreanLabel, KoreanButton
            self.window = tk.Tk()
            self.window.title('작업 완료 — 토크 유지')
            KoreanLabel(self.window, text='재료를 시작 위치로 되돌린 뒤 다시 실행하세요.').pack(padx=24, pady=16)
            KoreanButton(self.window, text='다시 실행 (R)', command=self.request_restart).pack(pady=8)
            KoreanButton(self.window, text='종료 · 토크 해제 (Q)', command=self.request_quit).pack(pady=8)
            self.window.protocol('WM_DELETE_WINDOW', self.request_quit)
            self.window.bind('<r>', lambda _: self.request_restart())
            self.window.bind('<R>', lambda _: self.request_restart())
            self.window.bind('<q>', lambda _: self.request_quit())
            self.window.bind('<Q>', lambda _: self.request_quit())
            self.window.bind('<Escape>', lambda _: self.request_quit())
        except Exception as exc:
            self.close()
            print(f'버튼 창을 열 수 없습니다. 터미널 R / Q 키를 사용하세요: {exc}', flush=True)

    def request_restart(self): self.restart = True
    def request_quit(self): self.quit = True

    def poll(self):
        if self.window is not None: self.window.update()
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            if key == 'r': self.request_restart()
            elif not key or key in ('q', '\x1b'): self.request_quit()
        return self.quit

    def take_restart(self):
        requested, self.restart = self.restart, False
        return requested

    def close(self):
        if self.window is not None:
            self.window.destroy()
            self.window = None


def hold_completed_pose(bus, cal, guard, poll=stop_requested, restart_requested=None,
                        restart_ready=None):
    """Keep one measured pose with torque on until explicit release or a fault.

    No policy or completion detector is called here, so task motion cannot restart.
    The caller retains ownership of torque-off cleanup.
    """
    if poll(): return 'released_by_user'
    tick = time.monotonic()
    values = control.feedback(bus, cal, guard)
    pose = {j:values[j]['raw'] for j in control.JOINTS}
    for j in control.JOINTS:
        if bus.read(j, 'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost during hold')
    if time.monotonic()-tick > MAX_INFERENCE_SECONDS:
        raise RuntimeError('Hold setup feedback deadline exceeded')
    if poll(): return 'released_by_user'
    for j in control.JOINTS:
        bus.write_position_profile(j, pose[j], control.SERVO_PROFILE_SPEED)
    print('작업 완료 — 토크 ON, 현재 자세 유지 중. 자동 동작은 재시작하지 않습니다.', flush=True)
    print('팔을 받친 뒤 q / Esc / Ctrl+C를 누르면 토크를 해제하고 종료합니다.', flush=True)
    if restart_requested is not None:
        print('재료를 시작 위치로 되돌린 뒤 [다시 실행] 버튼 또는 R 키: 5초 후 재실행.', flush=True)
    restart_at = None
    last_remaining = None
    while True:
        if poll(): return 'released_by_user'
        tick = time.monotonic()
        control.feedback(bus, cal, guard)
        for j in control.JOINTS:
            if bus.read(j, 'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost during hold')
        if time.monotonic()-tick > MAX_INFERENCE_SECONDS:
            raise RuntimeError('Hold monitoring deadline exceeded')
        if restart_requested is not None and restart_requested() and restart_at is None:
            if restart_ready():
                restart_at = time.monotonic()+5
            else:
                print('아직 완료 배치입니다. 재료를 시작 위치로 옮긴 뒤 다시 실행을 누르세요.', flush=True)
        if restart_at is not None:
            remaining = max(0, math.ceil(restart_at-time.monotonic()))
            if remaining != last_remaining:
                print(f'{remaining}초 후 재실행 (q / Esc: 종료)', flush=True)
                last_remaining = remaining
            if remaining == 0:
                if restart_ready(): return 'restart_requested'
                print('완료 배치가 감지되어 재실행을 취소합니다.', flush=True)
                restart_at = None
                last_remaining = None
        # Servos retain the fixed target. Never track a drifting measured position.
        time.sleep(max(0, 1/30-(time.monotonic()-tick)))


def recover_camera(bus, cameras, cal, guard, poll):
    """Freeze one measured pose; require fresh frames from BOTH cameras for 300ms."""
    started = time.monotonic()
    values = control.feedback(bus, cal, guard)
    if time.monotonic()-started > MAX_INFERENCE_SECONDS:
        raise RuntimeError('Camera hold feedback deadline exceeded')
    for j in control.JOINTS:
        if bus.read(j, 'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost')
    if poll(): raise control.SessionStop('user_stopped')
    if time.monotonic()-started > MAX_INFERENCE_SECONDS:
        raise RuntimeError('Camera hold setup deadline exceeded')
    for j in control.JOINTS:
        bus.write_position_profile(j, values[j]['raw'], control.SERVO_PROFILE_SPEED)
    print('카메라 지연: 현재 자세·토크 유지. 최대 3초 동안 영상 복구를 기다립니다.', flush=True)
    stable_since, previous = None, None
    while time.monotonic()-started < 3:
        if poll(): raise control.SessionStop('user_stopped')
        tick = time.monotonic()
        control.feedback(bus, cal, guard)
        for j in control.JOINTS:
            if bus.read(j, 'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost')
        if time.monotonic()-tick > MAX_INFERENCE_SECONDS:
            raise RuntimeError('Camera recovery monitoring deadline exceeded')
        try:
            frames = cameras.snapshot()
        except CameraUnavailable:
            stable_since, previous = None, None
        else:
            stamps = tuple(frames[n][1] for n in CAMERAS)
            now = time.monotonic()
            if all(stamp > started and now-stamp <= .1 for stamp in stamps):
                if previous is None or all(a > b for a,b in zip(stamps, previous)):
                    if stable_since is None: stable_since = now
                    previous = stamps
                    if now-stable_since >= .3 and now-started < 3:
                        print('새 영상 복구 확인: 이전 예측을 버리고 작업을 재개합니다.', flush=True)
                        return
            else:
                stable_since, previous = None, None
        time.sleep(.02)
    raise RuntimeError('Camera recovery timed out (>3s); stopping')


def move_to_saved_start(bus, cameras, cal, start_pose, guard, poll=stop_requested, *, returning=False):
    """Use the supervised homing path for both initial positioning and return."""
    from start_pose import move_to_pose
    def camera_check(*_):
        try: cameras.snapshot()
        except CameraUnavailable:
            recover_camera(bus, cameras, cal, guard, poll)
            return False  # Discard goals computed before the recovery pause.
    move_to_pose(bus, cal, start_pose, guard, poll=poll, on_tick=camera_check, returning=returning)


def drive(bus, cameras, predictor, cal, seconds, poll=stop_requested, completion=None, safety_guard=None, start_pose=None, diagnostics=None):
    """Caller owns torque-off cleanup, including partial activation failures."""
    guard = safety_guard if safety_guard is not None else control.TemperatureGuard(control.TEMPERATURE_DURATION_S)
    from gripper_contact import GripperContactGuard
    def measured():
        return {j: v['raw'] for j,v in control.feedback(bus, cal, guard).items()}
    current = measured()
    for j in control.JOINTS:
        bus.write_position_profile(j, current[j], control.SERVO_PROFILE_SPEED)
        if bus.read(j, 'Goal_Velocity') != control.SERVO_PROFILE_SPEED or bus.read(j, 'Goal_Time') != 0:
            raise RuntimeError('Position profile readback mismatch')
    for j in control.JOINTS: bus.write(j, 'Torque_Enable', 1)
    if start_pose is not None:
        move_to_saved_start(bus, cameras, cal, start_pose, guard, poll=poll)
        current = measured()
    planner = control.TeachingTargets(cal, current, control.load_motion())
    contact_guard=GripperContactGuard(cal['gripper'],planner.motion['gripper_relative_target_points'])
    planner.syncing = False  # Model takes current pose; no leader synchronization.
    predictor.reset()
    if completion is not None: completion.reset()
    holding_for_completion = False
    next_completion_status = 0.0
    start = time.monotonic()
    count = 0
    next_diagnostic = start
    while seconds is None or time.monotonic()-start < seconds:
        if poll(): return 'user_stopped', count
        tick = time.monotonic()
        current = measured()
        try:
            frames = cameras.snapshot()
        except CameraUnavailable:
            recover_camera(bus, cameras, cal, guard, poll)
            predictor.reset()
            if completion is not None: completion.reset()
            holding_for_completion = False
            continue
        candidate = False
        if completion is not None:
            if completion.observe(*frames['overview']):
                print('작업 완료 확인: 지그 위 재료 3개 배치·그리퍼 이탈. 모델 동작을 종료합니다.', flush=True)
                return 'task_completed', count
            candidate = completion.last.get('candidate', False)
            if not candidate and tick >= next_completion_status:
                labels = {'destination_top':'지그 첫 번째 재료', 'destination_middle':'지그 두 번째 재료',
                          'destination_bottom':'지그 세 번째 재료', 'jig_clear':'지그 전체 모습',
                          'orange_parts':'지그 위 주황색 부품 2개', 'blue_parts':'지그 위 파란 원통 1개'}
                labels.update({key+'_color':value+' 색상' for key,value in list(labels.items())})
                missing = [labels.get(k,k) for k,v in completion.last.get('matches',{}).items() if not v]
                if not completion.last.get('gripper_clear',True): missing.append('그리퍼 이탈')
                print('완료 대기 — 확인되지 않은 항목:', ', '.join(missing) or '새 카메라 프레임', flush=True)
                next_completion_status = tick+2
        if candidate:
            # Pause predictions while checking that the completed scene persists.
            # Capture ONE hold pose; never follow drifting feedback while holding.
            if not holding_for_completion:
                hold_pose = dict(current)
                print('완료 후보 감지: 현재 자세를 유지하며 화면 상태를 확인합니다.', flush=True)
            action = vector(hold_pose, cal)
        else:
            if holding_for_completion: predictor.reset()  # Discard old queued actions.
            action = predictor.predict(vector(current, cal), frames)
        holding_for_completion = candidate
        if poll(): return 'user_stopped', count
        if time.monotonic()-tick > MAX_INFERENCE_SECONDS:
            raise RuntimeError('Observation/inference deadline exceeded (250ms); no action sent')
        if seconds is not None and time.monotonic()-start >= seconds: break
        try:
            cameras.snapshot()  # Never send a prediction made before an outage.
        except CameraUnavailable:
            recover_camera(bus, cameras, cal, guard, poll)
            predictor.reset()
            if completion is not None: completion.reset()
            holding_for_completion = False
            continue
        current = measured()  # Limit against fresh feedback, never the previous target.
        goals = planner.next(raw_action(action, cal), current)
        goals['gripper']=contact_guard.next(goals['gripper'],current['gripper'],time.monotonic())
        for j in control.JOINTS:
            if bus.read(j, 'Torque_Enable') != 1: raise RuntimeError(f'{j}: torque lost')
        if poll(): return 'user_stopped', count
        if time.monotonic()-tick > MAX_INFERENCE_SECONDS:
            raise RuntimeError('Control deadline exceeded before writing targets')
        for j in control.JOINTS:
            bus.write_position_profile(j, goals[j], control.SERVO_PROFILE_SPEED)
        count += 1
        if diagnostics is not None and tick >= next_diagnostic:
            # The measured pose is the feedback BEFORE this command, not its result.
            row = {'elapsed_s': round(tick-start, 2),
                   'state_before': vector(current, cal).tolist(),
                   'model_target': action.tolist(),
                   'sent_target': vector(goals, cal).tolist(),
                   'gripper_contact_hold': contact_guard.hold is not None}
            diagnostics.append(row)
            if len(diagnostics) > 600: del diagnostics[0]
            print(f"동작 진단 {row['elapsed_s']:.1f}s | "
                  f"그리퍼 현재={row['state_before'][-1]:.1f}, "
                  f"모델 목표={row['model_target'][-1]:.1f}, "
                  f"전송 목표={row['sent_target'][-1]:.1f}, "
                  f"접촉 유지={row['gripper_contact_hold']} (큰 값=열림)", flush=True)
            next_diagnostic = tick+2
        time.sleep(max(0, 1/planner.motion['control_hz']-(time.monotonic()-tick)))
    return 'time_limit', count


def hardware(cfg, cal, predictor, *, move, seconds, completion=None, release_on_complete=False, start_pose=None, debug_actions=False, return_to_start_on_complete=False):
    if return_to_start_on_complete:
        if completion is None or start_pose is None or release_on_complete:
            raise ValueError('Return to start requires completion, a saved start pose, and torque hold')
        from start_pose import validate
        validate(start_pose, cal)
    bus = control.SerialBus(control.PortConfig(cfg['robot']['port'], '5B3D046080', 'robot'))
    cameras = Cameras(cfg, 30, reconnect=True)
    attempted = False
    report = {'motion_requested': move, 'seconds': seconds, 'cleanup_errors': [],
              'release_on_complete': release_on_complete,
              'return_to_start_on_complete': return_to_start_on_complete}
    diagnostics = [] if debug_actions else None
    if diagnostics is not None: report['action_diagnostics'] = diagnostics
    if start_pose is not None: report['start_pose'] = start_pose
    if completion is not None:
        report['completion_profile'] = str(completion.profile)
        report['completion_version'] = completion.config['version']
    def interrupt(signum, frame): raise KeyboardInterrupt()
    old_term = signal.signal(signal.SIGTERM, interrupt)
    old_int = signal.getsignal(signal.SIGINT)
    try:
        bus.open()
        control.verify(bus, cal)
        cameras.start()
        guard = control.TemperatureGuard(control.TEMPERATURE_DURATION_S)
        current = {j:v['raw'] for j,v in control.feedback(bus, cal, guard).items()}
        action = predictor.predict(vector(current, cal), cameras.snapshot())
        raw_action(action, cal)
        predictor.reset()
        print('카메라·관절·모델 점검 통과. 예측 목표:', action.tolist(), flush=True)
        if start_pose is not None:
            from start_pose import validate
            validate(start_pose,cal)
            print('학습 데이터에 저장된 시작 자세를 사용합니다. START 후 먼저 시작 자세로 이동합니다.',flush=True)
        if completion is not None:
            status = completion.evaluate(cameras.snapshot()['overview'][0])
            print('완료 판정 점검:', status, flush=True)
        if not move:
            report['outcome'] = 'read_only_check_passed'
            return
        if completion is not None and status['candidate']:
            raise RuntimeError('이미 완료 상태처럼 보입니다. 재료를 시작 위치로 되돌린 뒤 실행하세요.')
        if not sys.stdin.isatty(): raise RuntimeError('실제 실행은 대화형 터미널에서 시작하세요.')
        print('학습 시작 자세와 재료·지그 배치를 맞추세요. 카메라 미리보기/record를 종료하세요.')
        print('종료 시 토크가 풀립니다. 팔을 받칠 준비를 하세요. q / Esc / Ctrl+C: 종료.')
        if completion is not None and not release_on_complete:
            print('작업 완료 후에는 토크 ON으로 자세를 유지합니다. 직접 종료하거나 오류가 나면 해제합니다.')
        if return_to_start_on_complete:
            print('완료 확인 후 저장된 시작 자세로 자동 복귀하고 대기합니다. 복귀 경로도 비워 주세요.')
        print('종료 조건:', '화면에서 작업 완료 확인 (시간 제한 없음)' if completion is not None
              else '시간 제한 없음 (직접 종료)' if seconds is None else f'{seconds:g}초')
        if input('움직임을 시작하려면 start 또는 START 입력: ').strip().upper() != 'START':
            report['outcome'] = 'cancelled'
            return
        with terminal_keys():
            for remaining in range(5, 0, -1):
                print(f'{remaining}초 후 시작 (q: 취소)', flush=True)
                deadline = time.monotonic()+1
                while time.monotonic() < deadline:
                    if stop_requested():
                        report['outcome'] = 'cancelled'
                        return
                    cameras.snapshot()
                    time.sleep(.05)
            # Mark BEFORE any motor write to cover partial activation failures.
            attempted = True
            safety_guard = control.TemperatureGuard(control.TEMPERATURE_DURATION_S)
            report['runs'] = []
            while True:
                report['outcome'], report['actions_sent'] = drive(bus, cameras, predictor, cal, seconds,
                    completion=completion, safety_guard=safety_guard, start_pose=start_pose, diagnostics=diagnostics)
                report['runs'].append({'outcome':report['outcome'], 'actions_sent':report['actions_sent']})
                if report['outcome'] != 'task_completed': break
                report['task_completed'] = True
                if release_on_complete: break
                if return_to_start_on_complete:
                    report['post_completion'] = 'returning_to_start'
                    report['runs'][-1]['return_to_start'] = 'in_progress'
                    predictor.reset()
                    print('작업 완료 — 저장된 시작 자세로 복귀합니다. q / Esc / Ctrl+C: 중지.', flush=True)
                    move_to_saved_start(bus, cameras, cal, start_pose, safety_guard, returning=True)
                    report['runs'][-1]['return_to_start'] = 'arrived'
                    print('시작 자세 복귀 완료 — 토크 ON으로 대기합니다.', flush=True)
                controls = CompletedControls()
                try:
                    report['post_completion'] = 'holding'
                    report['post_completion'] = hold_completed_pose(bus, cal, safety_guard,
                        poll=controls.poll, restart_requested=controls.take_restart,
                        restart_ready=lambda: not completion.evaluate(cameras.snapshot()['overview'][0])['candidate'])
                finally:
                    controls.close()
                if report['post_completion'] != 'restart_requested': break
    except control.SessionStop as exc:
        report['outcome'] = str(exc)
    except KeyboardInterrupt:
        report['outcome'] = 'interrupted'
    except Exception as exc:
        report.update(outcome='error', error=repr(exc))
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if attempted: report['cleanup_errors'] = release(bus)
            if completion is not None: report['completion_last'] = completion.last
            for resource in (bus, cameras):
                try: resource.close()
                except Exception as exc: report['cleanup_errors'].append(repr(exc))
            out = ROOT/'logs'/('policy_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
            out.parent.mkdir(exist_ok=True)
            if completion is not None:
                report['completion_best'] = completion.best
                if completion.best_rgb is not None:
                    import cv2
                    image_path = out.with_name(out.stem+'_completion_best.jpg')
                    try:
                        if not cv2.imwrite(str(image_path), cv2.cvtColor(completion.best_rgb,cv2.COLOR_RGB2BGR)):
                            raise RuntimeError('Could not save completion diagnostic image')
                        report['completion_best_image'] = str(image_path)
                    except Exception as exc: report['completion_image_error'] = repr(exc)
            out.write_text(json.dumps(report, indent=2)+'\n')
            print('결과:', report, '\n로그:', out, flush=True)
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)
        if report['cleanup_errors']: raise RuntimeError('Torque-off/cleanup failed; inspect log and support arm')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='Read cameras and follower; never enable torque')
    mode.add_argument('--run', action='store_true', help='Enable motion after interactive START confirmation')
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--completion-profile',type=Path,default=project_path('completion/product_a.json', ROOT),
                        help='Visual completion profile matching the physical jig layout')
    parser.add_argument('--device', choices=['cuda','cpu'], default='cuda')
    parser.add_argument('--n-action-steps', type=int,
                        help='Replan after this many actions; runtime override only, default: checkpoint setting')
    parser.add_argument('--debug-actions', action='store_true',
                        help='Print gripper targets every 2 seconds and save the last 600 diagnostic samples')
    parser.add_argument('--release-on-complete', action='store_true',
                        help='With --until-done, release torque immediately on completion instead of holding')
    parser.add_argument('--return-to-start-on-complete', action='store_true',
                        help='With --until-done, return to the saved start pose, then hold torque and wait')
    duration = parser.add_mutually_exclusive_group()
    duration.add_argument('--seconds', type=float, default=30, help='Motion timeout (default: 30 seconds)')
    duration.add_argument('--no-time-limit', action='store_true', help='Run until manual stop or an error')
    duration.add_argument('--until-done', action='store_true', help='Stop once the selected completion scene is confirmed; no time limit')
    args = parser.parse_args()
    if args.n_action_steps is not None and args.n_action_steps < 1:
        parser.error('--n-action-steps must be positive')
    if args.release_on_complete and not args.until_done: parser.error('--release-on-complete requires --until-done')
    if args.return_to_start_on_complete and (not args.until_done or args.release_on_complete):
        parser.error('--return-to-start-on-complete requires --until-done and cannot use --release-on-complete')
    if not math.isfinite(args.seconds) or args.seconds <= 0: parser.error('--seconds must be positive and finite')
    if args.no_time_limit or args.until_done: args.seconds = None
    cfg, cal = load_context(args.model)
    from start_pose import load as load_start_pose
    start_pose = load_start_pose(model_dataset(args.model)/'start_pose.json',cal)
    if args.return_to_start_on_complete and start_pose is None:
        parser.error('Return to start requires a saved dataset/start_pose.json')
    completion = None
    if args.until_done:
        from task_completion import CompletionDetector
        completion = CompletionDetector(args.completion_profile)
        completion.validate_identity(identity(cfg, json.loads((project_path('purple_task.json', ROOT)).read_text())))
        print(f"완료 기준: version={completion.config['version']} "
              f"method={completion.config.get('method','appearance')} | {completion.profile}", flush=True)
    predictor = Predictor(args.model, args.device, n_action_steps=args.n_action_steps)
    # Warm up without opening hardware. Synthetic predictions are never sent to motors.
    frames = {n:(np.zeros((480,640,3), dtype=np.uint8), time.monotonic()) for n in CAMERAS}
    predictor.predict(np.zeros(6, dtype=np.float32), frames)
    predictor.reset()
    print('모델 로드·정규화·추론 점검 통과:', args.model, flush=True)
    if args.check or args.run:
        hardware(cfg, cal, predictor, move=args.run, seconds=args.seconds, completion=completion,
                 release_on_complete=args.release_on_complete, start_pose=start_pose, debug_actions=args.debug_actions,
                 return_to_start_on_complete=args.return_to_start_on_complete)
    else:
        print('로봇/카메라에 연결하지 않았습니다. --check: 장치 점검 / --run: 실제 실행')


if __name__ == '__main__':
    main()
