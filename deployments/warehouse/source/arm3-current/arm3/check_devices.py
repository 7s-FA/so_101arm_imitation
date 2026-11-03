"""Read motor calibration without writes; briefly capture both cameras."""
import json
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import pyrealsense2 as rs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

ROOT = Path(__file__).resolve().parent


def check_motor(cfg):
    directory = Path(cfg['calibration_dir'])
    if not directory.is_absolute():
        directory = ROOT / directory
    expected = json.loads((directory / (cfg['id'] + '.json')).read_text())
    bus = FeetechMotorsBus(port=cfg['port'], motors={
        name: Motor(value['id'], 'sts3215', MotorNormMode.RANGE_M100_100)
        for name, value in expected.items()
    })
    try:
        bus.connect()
        actual = {name: asdict(value) for name, value in bus.read_calibration().items()}
        state = {name: {field: bus.read(field, name, normalize=False)
                       for field in ('Present_Position', 'Torque_Enable', 'Present_Temperature')}
                 for name in expected}
        return {'match': actual == expected, 'expected': expected, 'actual': actual, 'state': state}
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def rgb_capture(path, out):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open external RGB camera')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        times = []
        started = time.monotonic()
        while time.monotonic() - started < 8:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('External RGB frame read failed')
            times.append(time.monotonic())
            if len(times) == 15:
                if not cv2.imwrite(str(out / 'external_rgb.jpg'), frame):
                    raise RuntimeError('Snapshot save failed')
        if len(times) < 2:
            raise RuntimeError('Too few RGB frames')
        return {'frames': len(times), 'fps': (len(times)-1)/(times[-1]-times[0]), 'shape': list(frame.shape)}
    finally:
        cap.release()


def realsense_capture(out):
    devices = list(rs.context().query_devices())
    candidates = [d for d in devices if d.get_info(rs.camera_info.name) == 'Intel RealSense D435']
    if len(candidates) != 1:
        raise RuntimeError(f'Expected one D435, found {len(candidates)}')
    serial = candidates[0].get_info(rs.camera_info.serial_number)
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    started_pipeline = False
    try:
        profile = pipe.start(config)
        started_pipeline = True
        times = []
        started = time.monotonic()
        while time.monotonic() - started < 8:
            frames = pipe.wait_for_frames(5000)
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                continue
            times.append(time.monotonic())
            if len(times) == 15:
                color_data = np.asanyarray(color.get_data())
                depth_data = np.asanyarray(depth.get_data())
                if not cv2.imwrite(str(out / 'd435_rgb.jpg'), color_data):
                    raise RuntimeError('Snapshot save failed')
                np.save(out / 'd435_depth.npy', depth_data)
        if len(times) < 2:
            raise RuntimeError('Too few D435 frames')
        return {'serial_number': serial, 'frames': len(times), 'fps': (len(times)-1)/(times[-1]-times[0]),
                'requested_fps': 15, 'resolution': [640,480],
                'depth_scale_m': profile.get_device().first_depth_sensor().get_depth_scale(),
                'depth_nonzero_fraction': float(np.count_nonzero(np.asanyarray(depth.get_data())) / (640*480))}
    finally:
        if started_pipeline:
            pipe.stop()


def main():
    cfg = json.loads((ROOT / 'devices.json').read_text())
    out = ROOT / 'checks' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True)
    results = {'motor_writes': False}
    for role in ('robot', 'leader'):
        try:
            results[role] = check_motor(cfg[role])
            print(role, 'MATCH' if results[role]['match'] else 'MISMATCH', flush=True)
        except Exception as exc:
            results[role] = {'error': repr(exc)}
            print(role, repr(exc), flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs = {'rgb': pool.submit(rgb_capture, cfg['cameras']['rgb']['path'], out),
                'realsense': pool.submit(realsense_capture, out)}
        for role, job in jobs.items():
            try:
                results[role] = job.result()
            except Exception as exc:
                results[role] = {'error': repr(exc)}
            print(role, results[role], flush=True)
    (out / 'report.json').write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')
    print('Report:', out / 'report.json', flush=True)
    if any('error' in results[r] for r in ('robot','leader','rgb','realsense')) or not all(results[r].get('match') for r in ('robot','leader')):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
