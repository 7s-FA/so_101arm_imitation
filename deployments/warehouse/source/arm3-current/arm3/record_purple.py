"""Record purple-group episodes with cancel/restart; export after torque is off. Local only."""
import argparse
from contextlib import contextmanager, nullcontext
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import select
import shutil
import sys
import termios
import threading
import time
import tty

import cv2
import numpy as np

import teleoperate_slow as control
from project_paths import project_path
from capture_queue import pending_sessions

ROOT = Path(__file__).resolve().parent
CAMERAS = ('wrist', 'overview')


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def identity(cfg, task):
    calibration = {}
    for role in ('robot', 'leader'):
        directory = Path(cfg[role]['calibration_dir'])
        if not directory.is_absolute(): directory = ROOT / directory
        path = directory / (cfg[role]['id'] + '.json')
        calibration[role] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {'calibration_sha256': calibration, 'robot_port': cfg['robot']['port'],
            'leader_port': cfg['leader']['port'], 'rgb_path': cfg['cameras']['rgb']['path'],
            'realsense_serial': cfg['cameras']['realsense']['serial_number'],
            'task': task, 'motion': control.load_motion(),
            'position_limit_policy': control.POSITION_LIMIT_POLICY,
            'leader_gripper_low_voltage_policy': dict(control.LOW_VOLTAGE_POLICY),
            'temperature_limit': control.TEMPERATURE_LIMIT,
            'temperature_duration_s': control.TEMPERATURE_DURATION_S,
            'camera_resolution': [640,480], 'action': 'sent absolute target; body degrees, gripper calibrated points (0..100 reference)',
            'joint_order': list(control.JOINTS)}


class CameraUnavailable(RuntimeError):
    """Missing or stale frames; deployment may pause for bounded recovery."""


class Cameras:
    def __init__(self, cfg, fps, *, view=None, reconnect=False):
        if fps not in (15, 30): raise ValueError("Camera FPS must be 15 or 30")
        self.cfg, self.fps, self.view = cfg, fps, view
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest, self.errors = {}, []
        self.listeners = []
        self.threads = []
        self.reconnect = reconnect

    def worker(self, name, capture):
        while not self.stop.is_set():
            try:
                capture()
                return
            except Exception as exc:
                if not self.reconnect:
                    with self.lock: self.errors.append(repr(exc))
                    return
                with self.lock:
                    self.latest.pop(name, None)
                print(f'{name}: 카메라 재연결 대기: {exc}', flush=True)
                self.stop.wait(.1)

    def add_listener(self, callback):
        with self.lock: self.listeners.append(callback)

    def remove_listener(self, callback):
        with self.lock:
            if callback in self.listeners: self.listeners.remove(callback)

    def publish(self, name, frame, *, received_monotonic_s=None, device_metadata=None):
        stamp = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        with self.lock: listeners = tuple(self.listeners)
        # Archive before exposing this image to the control snapshot. Callback
        # only queues work; image encoding never blocks the control thread.
        for callback in listeners: callback(name, rgb, stamp, device_metadata or {})
        with self.lock:
            self.latest[name] = (rgb, stamp)
        if self.view is not None:
            self.view.publish(name, rgb, stamp)

    def rgb(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.cfg['cameras']['rgb']['path'], cv2.CAP_V4L2)
            if not cap.isOpened(): raise RuntimeError('External RGB open failed; close camera preview first')
            for prop, value in [(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG')),
                                (cv2.CAP_PROP_FRAME_WIDTH,640),(cv2.CAP_PROP_FRAME_HEIGHT,480),
                                (cv2.CAP_PROP_FPS,self.fps)]: cap.set(prop,value)
            while not self.stop.is_set():
                ok, frame = cap.read()
                received = time.monotonic()
                if not ok: raise RuntimeError('External RGB read failed')
                self.publish('wrist',frame,received_monotonic_s=received)
        except Exception as exc:
            if self.reconnect: raise
            with self.lock: self.errors.append(repr(exc))
        finally:
            if cap is not None: cap.release()

    def overview(self):
        import pyrealsense2 as rs
        pipeline, started = rs.pipeline(), False
        try:
            config = rs.config()
            config.enable_device(self.cfg['cameras']['realsense']['serial_number'])
            config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,self.fps)
            pipeline.start(config)
            started = True
            while not self.stop.is_set():
                color = pipeline.wait_for_frames(3000).get_color_frame()
                received = time.monotonic()
                if color:
                    metadata = {'device_timestamp_ms': color.get_timestamp(),
                                'device_clock_domain': str(color.get_frame_timestamp_domain()),
                                'device_frame_number': color.get_frame_number()}
                    self.publish('overview',np.asanyarray(color.get_data()).copy(),
                                 received_monotonic_s=received,device_metadata=metadata)
        except Exception as exc:
            if self.reconnect: raise
            with self.lock: self.errors.append(repr(exc))
        finally:
            if started: pipeline.stop()

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            if self.errors: raise RuntimeError('; '.join(self.errors))
            if set(self.latest) != set(CAMERAS): raise CameraUnavailable('Waiting for both cameras')
            for name, (frame, stamp) in self.latest.items():
                if now-stamp > .25: raise CameraUnavailable(f'{name}: camera stale (>250ms)')
                if frame.shape != (480,640,3): raise RuntimeError(f'{name}: unexpected frame size')
            return dict(self.latest)

    def start(self):
        self.threads = [threading.Thread(target=self.worker,args=(name,capture),daemon=True)
                        for name,capture in [('wrist',self.rgb),('overview',self.overview)]]
        for thread in self.threads: thread.start()
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            with self.lock:
                if self.errors: raise RuntimeError('; '.join(self.errors))
                ready = set(self.latest)==set(CAMERAS)
            if ready:
                self.snapshot()
                return
            time.sleep(.1)
        raise RuntimeError('Cameras did not start; close preview and try again')

    def close(self):
        self.stop.set()
        for thread in self.threads: thread.join(timeout=4)


def vector(raw, calibration):
    result=[]
    for joint in control.JOINTS:
        value = raw[joint]
        c = calibration[joint]
        if joint=='gripper': value=(value-c['range_min'])*100/(c['range_max']-c['range_min'])
        else: value=(value-(c['range_min']+c['range_max'])/2)*360/4095
        result.append(value)
    return np.array(result,dtype=np.float32)


class Capture:
    def __init__(self, folder, cameras, calibration, *, auto_record=False, view=None):
        self.auto_record = auto_record
        self.view = view
        self.record_started = None
        self.folder, self.cameras, self.calibration = folder, cameras, calibration
        self.recording = self.ready = False
        self.count, self.error = 0, None
        self.queue = queue.Queue(maxsize=180)
        self.capture_lock = threading.Lock()
        self.closed = False
        self.worker = threading.Thread(target=self.write,daemon=True)
        self.worker.start()
        if hasattr(cameras, 'add_listener'): cameras.add_listener(self.observe_camera)

    def observe_camera(self, name, rgb, stamp, metadata):
        with self.capture_lock:
            if not self.recording or self.closed: return
            try:
                self.queue.put_nowait({'camera_frame': name, 'rgb': rgb,
                                       'stamp': stamp, 'metadata': metadata})
            except queue.Full:
                self.error = 'Camera archive queue full; capture cannot silently drop frames'
                raise RuntimeError(self.error)

    def write(self):
        try:
            with (self.folder/'frames.jsonl').open('w') as stream, \
                 (self.folder/'camera_frames.jsonl').open('w') as camera_stream:
                seen = {}
                counts = dict.fromkeys(CAMERAS, 0)
                def save_image(name, rgb, stamp, metadata=None):
                    key = (name, stamp)
                    if key in seen: return seen[key]
                    filename = f'camera_{name}_{counts[name]:06d}.jpg'
                    if not cv2.imwrite(str(self.folder/filename),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),
                                       [cv2.IMWRITE_JPEG_QUALITY,95]):
                        raise RuntimeError(f'Image write failed: {filename}')
                    camera_stream.write(json.dumps({'camera': name, 'frame_id': counts[name],
                        'host_received_monotonic_s': stamp, 'image': filename,
                        'device_metadata': metadata or {}})+'\n')
                    camera_stream.flush()
                    seen[key] = filename
                    counts[name] += 1
                    return filename
                while True:
                    item = self.queue.get()
                    try:
                        if item is None: return
                        if isinstance(item, dict):
                            save_image(item['camera_frame'],item['rgb'],item['stamp'],item['metadata'])
                            continue
                        record, frames = item
                        for name, (rgb, stamp) in frames.items():
                            stored = save_image(name,rgb,stamp)
                            # Keep legacy per-sample filenames without encoding
                            # or storing the same camera frame again.
                            filename = f'{record["index"]:06d}_{name}.jpg'
                            os.link(self.folder/stored,self.folder/filename)
                            record['images'][name]=filename
                            record['camera_monotonic_s'][name]=stamp
                        stream.write(json.dumps(record)+'\n')
                        stream.flush()
                    finally: self.queue.task_done()
        except Exception as exc: self.error=repr(exc)

    def observe(self, sample):
        if self.error: raise RuntimeError(self.error)
        frames = sample.get('camera_snapshot')
        if frames is None: frames = self.cameras.snapshot()
        self.last_raw = {j:sample['follower'][j]['raw'] for j in control.JOINTS}
        if sample['syncing']:
            if self.view is not None:
                label = ('저장된 시작 자세로 이동 중 · 리더는 가만히 유지하세요 (초기 오차 보정)'
                         if getattr(self,'start_pose',None) is not None else '리더에 동기화 중 · 데이터 수집과 동일한 추종 설정')
                self.view.update('syncing',label,'리더를 가만히 유지하세요. C: 취소 / R: 재시작 / Esc: 종료',force=False)
            return
        if not self.ready:
            self.ready=True
            if self.view is not None:
                self.view.update('ready','추종 준비 완료 · 아직 녹화하지 않음','녹화 버튼 또는 S를 누르고 세 부품을 옮기세요.')
            print('\nREADY: s=record, e=save, c=cancel episode, r=restart episode, q=quit.',flush=True)
            if self.auto_record:
                self.start_recording()
        if not self.recording: return
        if self.count == 0 and getattr(self,'start_pose',None) is not None:
            from start_pose import aligned
            if not aligned(self.last_raw,self.start_pose['raw'],self.calibration):
                raise RuntimeError('녹화 첫 프레임이 시작 자세에서 벗어났습니다. 팔을 되돌리고 다시 녹화하세요.')
        state = vector({j:sample['follower'][j]['raw'] for j in control.JOINTS},self.calibration)
        action = vector(sample['goal'],self.calibration)
        record={'index':self.count,'monotonic_s':sample['monotonic_s'],
                'state':state.tolist(),'action':action.tolist(),'images':{},'camera_monotonic_s':{},
                'timing':sample.get('timing',{})}
        self.queue.put_nowait((record,frames))
        self.count+=1
        if self.view is not None:
            self.view.update('recording',f'● 녹화 중 · {time.monotonic()-self.record_started:.1f}초 · {self.count}개 기록',
                             '세 부품을 놓고 그리퍼를 뺀 뒤 E 저장 / C 취소 / R 재시작',force=False)

    def start_recording(self):
        if getattr(self,'start_pose',None) is not None:
            from start_pose import aligned
            if not getattr(self,'last_raw',None) or not aligned(self.last_raw,self.start_pose['raw'],self.calibration):
                print('팔로워가 시작 자세에 있어야 녹화를 시작할 수 있습니다. 리더를 동기화 완료 당시 자세로 되돌린 뒤 S를 누르세요.',flush=True)
                return
        self.recording=True
        self.record_started=time.monotonic()
        if getattr(self,'view',None) is not None:
            self.view.update('recording','● 녹화 중','세 부품을 놓고 그리퍼를 뺀 뒤 E 저장 / C 취소 / R 재시작')
        print('\nRECORDING: top -> middle -> bottom. Withdraw gripper, then e to save.',flush=True)

    def handle_key(self, key):
        key=key.lower()
        if key=='s' and self.ready and not self.recording:
            self.start_recording()
        elif key=='e':
            if self.recording and self.count>=30: return 'success_requested'
            print('\nNot enough recorded frames yet; press s first.',flush=True)
        elif key=='y' and not self.recording:
            self.ready=False
            if getattr(self,'view',None) is not None:
                self.view.update('syncing','리더에 다시 동기화 중','리더를 가만히 유지하세요. 동기화 동작은 녹화하지 않습니다.')
            return 'resync_requested'
        elif key=='c': return 'cancel_requested'
        elif key=='r': return 'restart_requested'
        elif key in ('q','\x1b'): return 'quit_requested'
        return None

    def poll(self):
        if self.error: raise RuntimeError(self.error)
        if self.view is not None:
            key=self.view.poll()
            if key is not None: return self.handle_key(key)
            self.cameras.snapshot()
            return None
        if not select.select([sys.stdin],[],[],0)[0]: return None
        return self.handle_key(sys.stdin.read(1))

    def close(self):
        with self.capture_lock:
            self.recording = False
            self.closed = True
        if hasattr(self.cameras, 'remove_listener'):
            self.cameras.remove_listener(self.observe_camera)
        deadline=time.monotonic()+10
        while self.worker.is_alive() and not self.error:
            try:
                self.queue.put(None,timeout=.2)
                break
            except queue.Full:
                if time.monotonic()>deadline: raise RuntimeError('Disk writer did not drain')
        self.worker.join(timeout=15)
        if self.worker.is_alive(): raise RuntimeError('Disk writer did not finish')
        if self.error: raise RuntimeError(self.error)


@contextmanager
def terminal_keys():
    if not sys.stdin.isatty(): raise RuntimeError('Run in an interactive terminal')
    old=termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally: termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old)


def features():
    names=[j+'.pos' for j in control.JOINTS]
    result={k:{'dtype':'float32','shape':(6,),'names':names} for k in ('observation.state','action')}
    result.update({f'observation.images.{name}':{'dtype':'video','shape':(480,640,3),'names':['height','width','channels']} for name in CAMERAS})
    return result


def select_frames(records, fps):
    if len(records)<2: raise ValueError('Too few frames')
    times=np.array([r['monotonic_s'] for r in records])
    gaps=np.diff(times)
    if np.any(gaps<=0) or np.max(gaps)>.2: raise ValueError('Control capture timing gap >200ms or invalid timestamps')
    measured_fps = (len(records)-1)/(times[-1]-times[0])
    if measured_fps < .9*fps:
        raise ValueError(f'Capture rate {measured_fps:.2f}Hz is too low for requested {fps}FPS; do not upsample')
    # Regular dataset-FPS grid; use nearest complete state/action/image sample, no interpolated actions.
    targets=np.arange(times[0],times[-1]+1e-9,1/fps)
    right=np.clip(np.searchsorted(times,targets),0,len(times)-1)
    left=np.maximum(0,right-1)
    indices=np.where(abs(times[left]-targets)<=abs(times[right]-targets),left,right)
    return [records[i] for i in indices], float(np.max(abs(times[indices]-targets)))


def export(folder, *, view=None):
    # Stage-weighted estimate: frame conversion 0..90, encoding 90..98,
    # metadata/finalization 98..100. Never report completion before manifest commit.
    episode_index = None
    def progress(percent, detail, *, force=True):
        if view is not None:
            title = ('저장 준비 중 · 토크 OFF' if episode_index is None else
                     f'{episode_index+1}번째 에피소드 저장 중 (번호 {episode_index}) · 토크 OFF')
            view.update('saving', title, detail,
                        progress=percent, force=force, episode_index=episode_index,
                        saved_count=None if episode_index is None else episode_index+(percent==100))

    progress(0, '저장 준비 중')
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.configs import RGBEncoderConfig
    manifest=json.loads((folder/'session.json').read_text())
    if manifest.get('excluded_from_dataset'):
        raise RuntimeError('사용자가 데이터셋에서 제외한 에피소드입니다. 자동으로 다시 저장하지 않습니다.')
    if manifest.get('exported_episode') is not None:
        episode_index = manifest['exported_episode']
        progress(100, '이미 저장이 완료된 에피소드입니다.')
        print('Already exported episode',manifest['exported_episode'])
        return
    if not manifest.get('success'): raise RuntimeError('Session was not marked successful; no export')
    if manifest.get('export_started'): raise RuntimeError('Previous export was interrupted. Inspect dataset before retrying.')
    task=manifest['identity']['task']
    destination=ROOT/task['dataset_directory']
    records=[json.loads(line) for line in (folder/'frames.jsonl').read_text().splitlines()]
    selected,timing_error=select_frames(records,task['dataset_fps'])
    from record_timing import quality
    timing_report = quality(selected,task['dataset_fps'])
    timing_rows = timing_report.pop('rows')
    dump(folder/'timing_quality.json',timing_report)
    with (folder/'selected_frame_timing.jsonl').open('w') as timing_file:
        for row in timing_rows: timing_file.write(json.dumps(row)+'\n')
    print('Timing audit:', json.dumps(timing_report), flush=True)
    destination.parent.mkdir(exist_ok=True)
    with (destination.parent/(destination.name+'.lock')).open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        meta=destination/'arm3_identity.json'
        exists=destination.exists()
        if exists and (not meta.exists() or json.loads(meta.read_text()) != manifest['identity']):
            raise RuntimeError('Dataset identity differs; use a new dataset directory')
        pose_meta=destination/'start_pose.json'
        saved_pose=json.loads(pose_meta.read_text()) if pose_meta.exists() else None
        if exists and saved_pose != manifest.get('start_pose'):
            raise RuntimeError('Dataset start pose differs; do not mix demonstrations with different start poses')
        kwargs=dict(root=destination,rgb_encoder=RGBEncoderConfig(vcodec='h264'),encoder_threads=2)
        manifest['export_started']=True
        dump(folder/'session.json',manifest)
        dataset=None
        try:
            if exists:
                dataset=LeRobotDataset.resume(task['repo_id'],**kwargs)
            else:
                dataset=LeRobotDataset.create(task['repo_id'],fps=task['dataset_fps'],features=features(),
                                               robot_type='so101_follower',**kwargs)
                dump(meta,manifest['identity'])
                if manifest.get('start_pose') is not None: dump(pose_meta,manifest['start_pose'])
            index=dataset.meta.total_episodes
            episode_index=index
            progress(0, f'기존 {index}개 저장 완료 · 이번은 {index+1}번째 (에피소드 번호 {index})')
            for number,record in enumerate(selected,1):
                frame={'observation.state':np.array(record['state'],dtype=np.float32),
                       'action':np.array(record['action'],dtype=np.float32),'task':task['instruction']}
                for name in CAMERAS:
                    bgr=cv2.imread(str(folder/record['images'][name]))
                    if bgr is None: raise RuntimeError('Missing captured image')
                    frame[f'observation.images.{name}']=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
                dataset.add_frame(frame)
                progress(90*number/len(selected),
                         f'1/3 이미지 변환 · {number:,} / {len(selected):,} 프레임', force=False)
            progress(90, '2/3 영상 압축 및 데이터 기록 중 · 이 단계에서는 퍼센트가 잠시 유지됩니다.')
            dataset.save_episode(parallel_encoding=False)
            progress(98, '3/3 저장 마무리 중')
            dataset.finalize()
            manifest.update(exported_episode=index,export_started=False,export_frames=len(selected),
                            resample_max_time_error_s=timing_error,timing_quality=timing_report)
            dump(folder/'session.json',manifest)
            progress(100, f'저장 완료 · 에피소드 {index} · {len(selected):,} 프레임')
            print(f'SAVED: episode {index}, {len(selected)} frames, {destination}',flush=True)
        finally:
            if dataset is not None: dataset.finalize()


def discard_failed_capture(folder, manifest, capture):
    """Delete only this finished unsuccessful attempt; keep a small diagnostic log."""
    if manifest.get('success') or manifest.get('exported_episode') is not None or manifest.get('export_started'):
        return False
    if capture is not None and capture.worker.is_alive():
        print(f'원본 자동 삭제 보류: 기록 스레드가 아직 실행 중입니다. {folder}', flush=True)
        return False
    if folder.is_symlink() or folder.resolve().parent != (project_path('captures', ROOT)).resolve():
        raise RuntimeError('Refusing to delete a path outside the current capture directory')
    try:
        logs=project_path('logs', ROOT)
        logs.mkdir(exist_ok=True)
        dump(logs/f'capture_{folder.name}.json', manifest)
        shutil.rmtree(folder)
    except OSError as exc:
        print(f'원본 자동 삭제 실패: {folder}: {exc}', flush=True)
        return False
    print(f'미저장 에피소드 원본 자동 삭제 완료: {folder.name}', flush=True)
    return True


def collect_attempt(cfg, task, ident, *, auto_record=False, view=None, shared_cameras=None, defer_export=False):
    folder=project_path('captures', ROOT)/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    manifest={'identity':ident,'success':False,'exported_episode':None,'auto_record':auto_record,
              'timing_schema_version':2}
    cameras=shared_cameras
    capture=None
    report=None
    try:
        dump(folder/'session.json',manifest)
        calibration=json.loads((ROOT/'calibration/arm3.json').read_text())
        from start_pose import load as load_start_pose
        pose=load_start_pose(project_path('start_pose.json', ROOT),calibration)
        if pose is not None:
            manifest['start_pose']=pose
            dump(folder/'session.json',manifest)
        if cameras is None: cameras=Cameras(cfg,task['dataset_fps'])
        if shared_cameras is None: cameras.start()
        else: cameras.snapshot()
        if view is not None:
            view.update('syncing','시작 준비 · 잠시 후 동기화','팔을 받치고 리더를 가만히 유지하세요. Esc로 중지할 수 있습니다.')
        capture=Capture(folder,cameras,calibration,auto_record=auto_record,view=view)
        capture.start_pose=pose
        with (terminal_keys() if view is None else nullcontext()):
            options={'start_pose':pose} if pose is not None else {}
            report=control.run(True,task['max_session_seconds'],observer=capture.observe,poll=capture.poll,
                               observation_reader=cameras.snapshot,**options)
        if view is not None:
            view.update('saving','추종 종료 · 토크 OFF · 기록 정리 중')
        capture.close()
        manifest['recorded_frames']=capture.count
        capture=None
        manifest['success']=report.get('outcome')=='success_requested' and not report['cleanup_errors']
        manifest['outcome']=report.get('outcome')
        dump(folder/'session.json',manifest)
    except BaseException as exc:
        manifest.update(success=False,outcome='error',error=repr(exc))
        dump(folder/'session.json',manifest)
        raise
    finally:
        try:
            if shared_cameras is None and cameras is not None: cameras.close()
        finally:
            try:
                if capture is not None: capture.close()
            finally:
                discard_failed_capture(folder, manifest, capture)
    if report['cleanup_errors']:
        raise RuntimeError('Torque-off verification failed; automatic retry blocked')
    if manifest['success']:
        if defer_export:
            manifest['export_deferred'] = True
            dump(folder/'session.json', manifest)
            print(f'원본 저장 완료: {folder.name} · 학습용 변환은 나중에 실행하세요.', flush=True)
            return manifest['outcome']
        if view is not None:
            view.update('saving','저장 중 · 토크 OFF','카메라 영상과 시연을 저장하고 있습니다. 완료 후 다음 에피소드를 준비하세요.')
        print('Torque OFF confirmed. Exporting locally; no upload.',flush=True)
        export(folder, view=view)
    return manifest['outcome']


def collection_loop(cfg, task, ident, *, view=None, shared_cameras=None, defer_export=False):
    auto_record=False
    prompt='Press Enter to start cameras and synchronization, or q to quit: '
    while True:
        if defer_export:
            pending = len(pending_sessions(project_path('captures', ROOT)))
            info = ROOT/task['dataset_directory']/'meta/info.json'
            ready = json.loads(info.read_text())['total_episodes'] if info.exists() else 0
            detail = f'학습용 변환 완료 {ready}개 · 원본 저장/변환 대기 {pending}개 · 다음 시연 {ready+pending+1}번째'
            print(detail, flush=True)
            if view is not None:
                view.update('idle', '빠른 녹화 · 원본 저장 후 다음 시연', detail,
                            saved_count=ready, pending_count=pending)
        # Ignore queued recording hotkeys: reactivation always requires a fresh Enter.
        if view is None:
            termios.tcflush(sys.stdin,termios.TCIFLUSH)
            answer=input(prompt).strip().lower()
        else:
            detail = ('재시작: 세 부품과 두 팔을 원위치로 돌린 뒤 Enter. 동기화 후 자동 녹화합니다.' if auto_record else
                      '작업 공간을 비우고 리더를 원하는 자세에 두세요. 리더에 동기화(Y) 또는 시작(Enter)을 누른 뒤 리더를 가만히 유지하세요.')
            if (project_path('start_pose.json', ROOT)).exists():
                detail='리더 초기 오차 ±100 허용. Enter 후 팔로워는 저장된 시작 자세로 이동합니다. 경로를 비우고 리더를 가만히 유지하세요.'
            if not view.wait_ready(detail): return
            answer='' 
        if answer in ('q','quit'): return
        if answer:
            print('Press Enter when ready, or q to quit.',flush=True)
            continue
        if view is None:
            outcome=collect_attempt(cfg,task,ident,auto_record=auto_record,**({'defer_export':True} if defer_export else {}))
        else:
            outcome=collect_attempt(cfg,task,ident,auto_record=auto_record,view=view,shared_cameras=shared_cameras,
                                    **({'defer_export':True} if defer_export else {}))
        if outcome in ('quit_requested','interrupted'): return
        if outcome=='restart_requested':
            print('RESTART: current attempt excluded. Torque OFF. Reset all three parts and both arms.',flush=True)
            prompt='Press Enter after resetting; synchronization then recording will restart automatically. q=quit: '
            auto_record=True
        elif outcome=='cancel_requested':
            print('CANCELLED: current attempt excluded. Torque OFF. Waiting; saved episodes are unchanged.',flush=True)
            prompt='Reset parts and arms. Enter=prepare a new attempt (s starts recording), q=quit: '
            auto_record=False
        elif outcome=='success_requested':
            print('Episode saved. Torque OFF. Reset parts and arms for the next episode.',flush=True)
            prompt='Enter=prepare next episode (s starts recording), q=quit: '
            auto_record=False
        else:
            print('Time limit reached without saving. Torque OFF. Reset before another attempt.',flush=True)
            prompt='Enter=prepare another attempt (s starts recording), q=quit: '
            auto_record=False


def export_pending():
    """Sequential conversion only; existing export locks and recovery guards apply."""
    sessions = pending_sessions(project_path('captures', ROOT), strict=True)
    task = json.loads(project_path('purple_task.json', ROOT).read_text())
    cfg = json.loads((ROOT/'devices.json').read_text())
    current_identity = identity(cfg, task)
    # Check every queued session before modifying a dataset.
    for folder, manifest in sessions:
        if manifest.get('export_started'):
            raise RuntimeError(f'이전 변환이 중단된 원본입니다. 데이터 상태를 먼저 점검하세요: {folder}')
        if manifest['identity'] != current_identity:
            raise RuntimeError(f'현재 프로젝트와 장비·보정·작업 설정이 다른 원본입니다: {folder}')
    print(f'변환 대기: {len(sessions)}개 · 로봇/카메라를 사용하지 않습니다.', flush=True)
    for number, (folder, _) in enumerate(sessions, 1):
        print(f'변환 {number}/{len(sessions)}: {folder.name}', flush=True)
        export(folder)
    print('대기 원본 변환 완료.', flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gui',action='store_true',help='Use terminal-only controls without a camera window')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--export-session',type=Path,help='Export an already successful local capture; no motor access')
    modes.add_argument('--export-pending',action='store_true',help='Convert successful pending captures; no motor/camera access')
    modes.add_argument('--defer-export',action='store_true',help='Save raw captures immediately; convert later with --export-pending')
    args=parser.parse_args()
    if args.export_pending:
        export_pending()
        return
    if args.export_session:
        export(args.export_session.resolve())
        return
    if not (project_path('start_pose.json', ROOT)).is_file():
        parser.error('시작 자세가 없습니다. 팔로워를 원하는 시작 자세로 받쳐 고정하고 bash start-pose.sh save 를 먼저 실행하세요. 모터는 움직이지 않았습니다.')
    if args.no_gui and not sys.stdin.isatty(): parser.error('Run terminal-only mode in a terminal')
    task=json.loads((project_path('purple_task.json', ROOT)).read_text())
    cfg=json.loads((ROOT/'devices.json').read_text())
    ident=identity(cfg,task)
    destination=ROOT/task['dataset_directory']
    if destination.exists():
        meta=destination/'arm3_identity.json'
        if not meta.exists() or json.loads(meta.read_text()) != ident:
            raise RuntimeError('Existing dataset has different settings; choose a new dataset_directory')
        saved=destination/'start_pose.json'
        current=project_path('start_pose.json', ROOT)
        if (json.loads(saved.read_text()) if saved.exists() else None) != (json.loads(current.read_text()) if current.exists() else None):
            raise RuntimeError('시작 자세가 기존 시연 데이터와 다릅니다. 다른 시작 자세를 같은 데이터셋에 섞을 수 없습니다.')
    if (project_path('start_pose.json', ROOT)).exists():
        print('저장된 시작 자세 사용: 리더 초기 오차 ±100도/그리퍼 ±100포인트 허용·자동 보정. 시작 자세 이동은 녹화에 포함하지 않습니다.')
    print('작업:', task['instruction'], flush=True)
    print('데이터 저장 위치:', destination, flush=True)
    print('Close the camera preview. Clear the motion path and destination pallet; hold the leader at the desired pose for synchronization.')
    print('Keys: s record | e save | c cancel current episode | r restart current episode | q quit')
    view=cameras=None
    try:
        if args.no_gui:
            collection_loop(cfg,task,ident,defer_export=args.defer_export)
        else:
            from record_view import RecordView
            view=RecordView()
            cameras=Cameras(cfg,task['dataset_fps'],view=view)
            cameras.start()
            print('Camera window ready. Motors remain off until Start / Enter is pressed in the window.',flush=True)
            collection_loop(cfg,task,ident,view=view,shared_cameras=cameras,defer_export=args.defer_export)
    except KeyboardInterrupt:
        print('Collection ended.',flush=True)
    finally:
        if cameras is not None: cameras.close()
        if view is not None: view.close()


if __name__=='__main__': main()
