"""Live D435 view for fixing and checking the camera mount (camera only, never the motors).

Shows the 640x480 colour stream the server uses and where the ChArUco board sits against the reference, in both
directions (the server's check area touches the left image edge, so its own dx cannot go below 0). Min/max since the
last reset show how far the camera moves when something is touched. Keeps running while the USB cable is unplugged.
g overlays the reference frame, to aim a camera that was moved back to its position.

The D435 opens in one program only: stop the server first. Close with the window button, q or Esc; r resets min/max.
"""
import argparse
import fcntl
import json
from pathlib import Path
import sys
import threading
import time
import tkinter as tk

import cv2
import numpy as np
from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parents[1]
ARM1 = ROOT/'integration/arm1'
sys.path.insert(0, str(ARM1))
from jig_cells import JigCells, gradient

LOCK = ROOT/'.hardware-session.lock'
DEVICES = ROOT/'arm3-current/arm3/devices.json'
SEARCH = 20  # Same search range as JigCells.camera_check.
GOOD, BAD, WAIT = '#1b7f3b', '#c62828', '#555555'


def hardware_lock():
    f = LOCK.open('a')
    try: fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: sys.exit('관제 서버 또는 수동 실행이 D435를 쓰고 있습니다. 서버를 끈 뒤 다시 실행하세요(서버를 끄면 토크가 풀립니다).')
    return f


class Grabber(threading.Thread):
    """Latest colour frame; reopens the D435 whenever it disappears from USB."""
    def __init__(self, serial):
        super().__init__(daemon=True)
        self.serial = serial; self.closing = threading.Event(); self.lock = threading.Lock()
        self.frame = None; self.count = 0; self.usb = None; self.error = '연결 중...'

    def run(self):
        import pyrealsense2 as rs
        while not self.closing.is_set():
            pipe = rs.pipeline(); started = False
            try:
                cfg = rs.config(); cfg.enable_device(self.serial); cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                profile = pipe.start(cfg); started = True
                with self.lock: self.usb = profile.get_device().get_info(rs.camera_info.usb_type_descriptor); self.error = None
                while not self.closing.is_set():
                    color = pipe.wait_for_frames(3000).get_color_frame()
                    if color:
                        frame = np.asanyarray(color.get_data()).copy()
                        with self.lock: self.frame = frame; self.count += 1
            except Exception as exc:
                with self.lock: self.frame = None; self.usb = None; self.error = str(exc)
                self.closing.wait(1)
            finally:
                if started:
                    try: pipe.stop()
                    except Exception: pass

    def latest(self):
        with self.lock: return self.frame, self.count, self.usb, self.error


class Live:
    def __init__(self, grabber, seconds):
        burger = json.loads((ARM1/'model-map.json').read_text())['burger_correction']
        x0, y0, x1, y1 = burger['camera_check_roi']
        self.box = (max(x0, SEARCH), y0, x1, y1)
        self.max_px = burger.get('camera_check_max_px', 1.5); self.min_score = burger.get('camera_check_min_score', .5)
        reference = cv2.imread(str(ARM1/burger['reference']))
        if reference is None: sys.exit('기준 영상을 읽지 못했습니다: '+burger['reference'])
        self.reference = reference; self.G_ref = gradient(reference); self.ghost = False
        self.grabber = grabber; self.seen = 0; self.photo = None; self.reset()
        self.deadline = time.monotonic()+seconds if seconds else None
        self.root = tk.Tk(); self.root.title('D435 실시간 — 카메라 위치 확인')
        self.image = tk.Label(self.root, bg='black'); self.image.pack()
        self.main = tk.Label(self.root, font=('sans', 16, 'bold'), anchor='w', justify='left', fg=WAIT)
        self.main.pack(fill='x', padx=8, pady=(6, 0))
        self.detail = tk.Label(self.root, font=('sans', 12), anchor='w', justify='left')
        self.detail.pack(fill='x', padx=8, pady=(0, 6))
        buttons = tk.Frame(self.root); buttons.pack(fill='x', padx=8, pady=(0, 8))
        tk.Button(buttons, text='흔들림 폭 초기화 (r)', command=self.reset).pack(side='left')
        tk.Button(buttons, text='기준 겹쳐 보기 (g)', command=self.toggle_ghost).pack(side='left', padx=6)
        tk.Button(buttons, text='닫기 (q)', command=self.close).pack(side='right')
        for key in ('q', '<Escape>'): self.root.bind(key, lambda _: self.close())
        self.root.bind('r', lambda _: self.reset())
        self.root.bind('g', lambda _: self.toggle_ghost())
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.rate = (time.monotonic(), 0, 0.)
        self.tick()

    def reset(self):
        self.low = None; self.high = None; self.samples = 0

    def toggle_ghost(self):
        self.ghost = not self.ghost

    def measure(self, frame):
        shift, score = JigCells.locate(gradient(frame), self.G_ref, self.box, SEARCH)
        visible = score >= self.min_score   # Below this the match is noise: the shift means nothing.
        if visible:
            self.low = shift.copy() if self.low is None else np.minimum(self.low, shift)
            self.high = shift.copy() if self.high is None else np.maximum(self.high, shift)
            self.samples += 1
        return shift, score, visible

    def show(self, frame, good):
        x0, y0, x1, y1 = self.box
        view = cv2.addWeighted(frame, .5, self.reference, .5, 0) if self.ghost else frame.copy()
        cv2.rectangle(view, (x0, y0), (x1-1, y1-1), (0, 200, 0) if good else (0, 0, 230), 2)
        self.photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB)))
        self.image.configure(image=self.photo)

    def tick(self):
        frame, count, usb, error = self.grabber.latest()
        now = time.monotonic(); start, first, fps = self.rate
        if now-start >= 1: fps = (count-first)/(now-start); self.rate = (now, count, fps)
        if frame is None:
            self.main.configure(text='D435 연결 대기', fg=WAIT); self.detail.configure(text=error or '')
        elif count != self.seen:
            self.seen = count
            shift, score, visible = self.measure(frame); good = visible and np.abs(shift).max() <= self.max_px
            self.show(frame, good)
            if visible:
                self.main.configure(fg=GOOD if good else BAD,
                                    text=f'{"정상" if good else "허용 초과"}  좌우 {shift[0]:+.2f} px   상하 {shift[1]:+.2f} px   점수 {score:.3f}')
            else:
                self.main.configure(fg=BAD, text=f'차루코 보드 안 보임 (점수 {score:.3f}) — 카메라 방향 확인')
            spread = '흔들림 폭: 측정값 없음'
            if self.low is not None:
                span = self.high-self.low
                spread = (f'흔들림 폭 ({self.samples}프레임): 좌우 {self.low[0]:+.2f} ~ {self.high[0]:+.2f} ({span[0]:.2f} px)'
                          f'   상하 {self.low[1]:+.2f} ~ {self.high[1]:+.2f} ({span[1]:.2f} px)')
            self.detail.configure(text=f'{spread}\n허용 ±{self.max_px:g} px (약 1 px = 1 mm)   USB {usb}   {fps:.0f} fps')
        if self.deadline and now >= self.deadline: return self.close()
        self.root.after(15, self.tick)

    def close(self):
        if self.low is not None:
            print(f'프레임 {self.samples}: 좌우 {self.low[0]:+.2f} ~ {self.high[0]:+.2f} px, 상하 {self.low[1]:+.2f} ~ {self.high[1]:+.2f} px', flush=True)
        self.grabber.closing.set(); self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=0, help='이 시간 뒤 자동 종료 (0: 계속)')
    args = parser.parse_args()
    lock = hardware_lock()
    grabber = Grabber(json.loads(DEVICES.read_text())['cameras']['realsense']['serial_number'])
    live = Live(grabber, args.seconds)
    grabber.start()
    live.root.mainloop()
    grabber.join(timeout=5)
    del lock


if __name__ == '__main__':
    main()
