"""Live target-pallet offset window (D435 only; no robot or motor access). For setting up and checking jig_offset.py.

The Arm1 server holds the D435 while it runs: stop the server first. Buttons:
  기준 저장   save the current frame as the reference (pallet position 0) -> jig-offset.json (+ .reference.png)
  측정 기록   median of the next 15 frames, logged with the label typed beside it -> logs/jig_live_<time>.csv
Close with the window button or Esc.
"""
import argparse,csv,json,statistics,subprocess,sys,threading,time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image,ImageTk

ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
from jig_offset import JigOffsetMeter
DEVICES=ROOT.parents[1]/'arm3-current/arm3/devices.json'


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'jig-offset.json');a=p.parse_args()
    import pyrealsense2 as rs
    serial=json.loads(DEVICES.read_text())['cameras']['realsense']['serial_number']
    pipe=rs.pipeline();cfg=rs.config();cfg.enable_device(serial);cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,15);pipe.start(cfg)
    latest={'frame':None};stop=threading.Event();lock=threading.Lock()
    def grab():
        while not stop.is_set():
            f=pipe.wait_for_frames(3000).get_color_frame()
            if f:
                with lock:latest['frame']=np.asanyarray(f.get_data()).copy()
    threading.Thread(target=grab,daemon=True).start()
    meter={'m':JigOffsetMeter(a.config) if a.config.exists() else None}
    log=ROOT/'logs'/f"jig_live_{datetime.now():%Y%m%d_%H%M%S}.csv";log.parent.mkdir(exist_ok=True);rows=[]
    root=tk.Tk();root.title('목적지 팔레트 어긋남 측정 (카메라만 사용)')
    pic=ttk.Label(root);pic.grid(row=0,column=0,columnspan=4)
    text=tk.StringVar(value='');ttk.Label(root,textvariable=text,font=('TkFixedFont',13,'bold')).grid(row=1,column=0,columnspan=4,sticky='w',padx=6)
    label=tk.StringVar(value='0,0');ttk.Label(root,text='라벨(실제 이동 mm 등):').grid(row=2,column=0,sticky='e');ttk.Entry(root,textvariable=label,width=18).grid(row=2,column=1,sticky='w')
    status=tk.StringVar(value='');ttk.Label(root,textvariable=status,wraplength=640).grid(row=3,column=0,columnspan=4,sticky='w',padx=6)
    def frame():
        with lock:return None if latest['frame'] is None else latest['frame'].copy()
    def save_reference():
        f=frame()
        if f is None:return
        ref=ROOT/'logs'/f"jig_reference_{datetime.now():%Y%m%d_%H%M%S}.png";cv2.imwrite(str(ref),f)
        if a.config.exists():a.config.rename(a.config.with_name(f"jig-offset.{datetime.now():%Y%m%d_%H%M%S}.json"))   # keep the old one
        r=subprocess.run([sys.executable,str(ROOT/'make_jig_reference.py'),'--frame',str(ref),'--output',str(a.config)],capture_output=True,text=True)
        meter['m']=JigOffsetMeter(a.config) if a.config.exists() else None
        status.set('기준 저장 완료: '+str(ref.name) if r.returncode==0 else '기준 저장 실패: '+r.stderr[-300:])
    def record():
        if meter['m'] is None:status.set('먼저 기준을 저장하세요');return
        got=[];t0=time.time()
        while len(got)<15 and time.time()-t0<5:
            f=frame();o=meter['m'].measure(f) if f is not None else {'ok':False}
            if o.get('ok'):got.append(o)
            root.update();time.sleep(.07)
        if len(got)<8:status.set(f'측정 실패: 유효 프레임 {len(got)}/15');return
        med={k:statistics.median(o[k] for o in got) for k in ('dx_mm','dy_mm','rot_deg')}
        rows.append(dict(time=datetime.now().isoformat(timespec='seconds'),label=label.get(),**{k:round(v,3) for k,v in med.items()},frames=len(got)))
        with log.open('w',newline='') as fo:
            w=csv.DictWriter(fo,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        status.set(f"기록 {len(rows)}: [{label.get()}] x {med['dx_mm']:+.2f} mm, y {med['dy_mm']:+.2f} mm, 회전 {med['rot_deg']:+.2f}°  -> {log.name}")
    ttk.Button(root,text='기준 저장',command=save_reference).grid(row=2,column=2);ttk.Button(root,text='측정 기록',command=record).grid(row=2,column=3)
    def refresh():
        f=frame()
        if f is not None:
            v=f.copy();m=meter['m']
            if m is None:text.set('기준 없음: 팔레트를 기준 위치에 두고 [기준 저장]')
            else:
                o=m.measure(f)
                for name,line in m.lines.items():cv2.line(v,tuple(map(int,line[0])),tuple(map(int,line[1])),(0,0,255),1)
                text.set(f"x {o['dx_mm']:+.2f} mm   y {o['dy_mm']:+.2f} mm   회전 {o['rot_deg']:+.2f}°   (화면 오른쪽=+y, 위=-x)" if o['ok'] else '측정 거부: '+o['reason'])
            photo=ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(v,cv2.COLOR_BGR2RGB)));pic.configure(image=photo);pic.image=photo
        root.after(150,refresh)
    def close(*_):
        stop.set();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);root.bind('<Escape>',close);refresh()
    try:root.mainloop()
    finally:
        stop.set();time.sleep(.2);pipe.stop()

if __name__=='__main__':main()
