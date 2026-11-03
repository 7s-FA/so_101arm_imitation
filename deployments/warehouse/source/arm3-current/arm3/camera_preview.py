"""Live external RGB and D435 RGB/depth preview; no recording or motor access."""
import json
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk
import pyrealsense2 as rs

ROOT = Path(__file__).resolve().parent


def main():
    cfg = json.loads((ROOT / 'devices.json').read_text())
    stop = threading.Event()
    lock = threading.Lock()
    latest, errors = {}, {}
    window = tk.Tk()
    window.title('arm3 — RGB / D435 Live Preview')
    panels = {}
    for index, name in enumerate(('External RGB', 'D435 RGB', 'D435 Depth')):
        frame = ttk.LabelFrame(window, text=name)
        frame.grid(row=0, column=index, padx=5, pady=5)
        panels[name] = ttk.Label(frame, text='Connecting...', width=50)
        panels[name].pack()
    status = tk.StringVar(value='Connecting cameras...')
    ttk.Label(window, textvariable=status).grid(row=1, column=0, columnspan=3, pady=5)

    def publish(name, frame):
        with lock:
            first = name not in latest
            latest[name] = (frame, time.monotonic())
        if first:
            print(f'LIVE: {name} {frame.shape}', flush=True)

    def rgb_worker():
        cap = None
        try:
            cap = cv2.VideoCapture(cfg['cameras']['rgb']['path'], cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError('Camera open failed')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            while not stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('Frame read failed')
                publish('External RGB', frame)
        except Exception as exc:
            with lock:
                errors['External RGB'] = str(exc)
            print('External RGB ERROR:', repr(exc), flush=True)
        finally:
            if cap is not None:
                cap.release()

    def depth_worker():
        pipeline = rs.pipeline()
        started = False
        try:
            config = rs.config()
            config.enable_device(cfg['cameras']['realsense']['serial_number'])
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            profile = pipeline.start(config)
            started = True
            scale = profile.get_device().first_depth_sensor().get_depth_scale()
            while not stop.is_set():
                frames = pipeline.wait_for_frames(3000)
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if color:
                    publish('D435 RGB', np.asanyarray(color.get_data()).copy())
                if depth:
                    raw = np.asanyarray(depth.get_data())
                    # Fixed 0..2 metre visualization; invalid depth is black.
                    mapped = np.clip(raw.astype(np.float32) * scale * 255 / 2, 0, 255).astype(np.uint8)
                    colored = cv2.applyColorMap(mapped, cv2.COLORMAP_TURBO)
                    colored[raw == 0] = 0
                    publish('D435 Depth', colored)
        except Exception as exc:
            with lock:
                errors['D435'] = str(exc)
            print('D435 ERROR:', repr(exc), flush=True)
        finally:
            if started:
                pipeline.stop()

    def refresh():
        with lock:
            frames, faults = dict(latest), dict(errors)
        for name, (frame, stamp) in frames.items():
            display = cv2.cvtColor(cv2.resize(frame, (424,318)), cv2.COLOR_BGR2RGB)
            photo = ImageTk.PhotoImage(Image.fromarray(display))
            panels[name].configure(image=photo, text='')
            panels[name].image = photo
        stale = [name for name, (_,stamp) in frames.items() if time.monotonic()-stamp > 2]
        if faults:
            status.set(' | '.join(f'{name}: {error}' for name,error in faults.items()))
        elif stale:
            status.set('No recent frames: ' + ', '.join(stale))
        else:
            status.set(f'Live {len(frames)}/3 | RGB 30 FPS / D435 15 FPS requested | Depth colors: 0–2 m | No recording')
        window.after(66, refresh)

    def close():
        stop.set()
        window.destroy()

    window.protocol('WM_DELETE_WINDOW', close)
    window.bind('<Escape>', lambda event: close())
    ttk.Button(window, text='Close (Esc)', command=close).grid(row=2,column=0,columnspan=3,pady=5)
    workers = [threading.Thread(target=rgb_worker,daemon=True),threading.Thread(target=depth_worker,daemon=True)]
    for worker in workers:
        worker.start()
    refresh()
    window.lift()
    print('GUI ready',flush=True)
    try:
        window.mainloop()
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=4)


if __name__ == '__main__':
    main()
