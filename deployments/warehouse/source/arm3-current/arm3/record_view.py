"""Camera-sharing GUI process. Never opens cameras or robot ports."""
import multiprocessing as mp
import queue
import time

import numpy as np

NAMES = ('wrist', 'overview')
SHAPE = (480, 640, 3)


def preview_frame(frame, name, roi, masked):
    """Display only. Never mutate camera buffers used by recording/control."""
    from PIL import Image, ImageDraw
    from roi_mask import mask_rgb
    image = Image.fromarray(mask_rgb(frame, name, roi) if roi and masked else frame)
    if roi and name == 'overview':
        draw = ImageDraw.Draw(image)
        for x1, y1, x2, y2 in roi['overview_boxes']:
            draw.rectangle((x1, y1, x2-1, y2-1), outline=(0, 255, 112), width=2)
    return image


def viewer_main(buffers, locks, stamps, heartbeat, statuses, commands, quit_event):
    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk
    from gui_text import KoreanLabel, KoreanButton, korean_group

    root=tk.Tk()
    from project_paths import project_root
    project = project_root()
    root.title(f'arm3 — {project.name if project else "model1"} / 시연 녹화')
    root.geometry('1320x740')
    root.minsize(960,650)
    heading=KoreanLabel(root,text='카메라 연결 중 · 로봇은 시작 버튼을 누른 뒤 연결합니다.',size=20)
    heading.pack(fill='x',padx=12,pady=8)
    import json
    from pathlib import Path
    from project_paths import project_path
    base=Path(__file__).resolve().parent
    task=json.loads(project_path('purple_task.json',base).read_text())
    info=base/task['dataset_directory']/'meta/info.json'
    saved_count=json.loads(info.read_text())['total_episodes'] if info.exists() else 0
    episode_label=KoreanLabel(root,text=f'저장 완료: {saved_count}개 · 다음: {saved_count+1}번째 (번호 {saved_count})',size=16)
    episode_label.pack(fill='x',padx=12)
    from project_paths import PROJECTS
    from roi_mask import load as load_roi
    roi_path = (project if project else PROJECTS/'model1')/'roi_mask.json'
    roi_state = {'config': None}
    roi_preview = tk.BooleanVar(value=True)
    roi_bar = ttk.Frame(root); roi_bar.pack(fill='x', padx=12)
    ttk.Checkbutton(roi_bar, variable=roi_preview).pack(side='left')
    KoreanLabel(roi_bar, text='마스크 미리보기 · 저장 영상은 원본 유지').pack(side='left')
    roi_label = KoreanLabel(roi_bar, text=''); roi_label.pack(side='left', padx=12)
    def reload_roi():
        try:
            roi_state['config'] = load_roi(roi_path) if roi_path.is_file() else None
            text = ('ROI 설정 없음 · 프로젝트 관리에서 영역을 저장하세요.' if roi_state['config'] is None else
                    f'박스 {len(roi_state["config"]["overview_boxes"])}개 · 손목 ' +
                    ('전체 유지' if roi_state['config']['wrist'] == 'full' else '가림'))
            roi_label.config(text=text)
        except Exception as exc:
            roi_state['config'] = None
            roi_label.config(text=f'ROI 오류 · 원본 표시: {exc}')
    KoreanButton(roi_bar, text='영역 설정 다시 읽기', command=reload_roi).pack(side='right')
    reload_roi()
    row=ttk.Frame(root)
    row.pack(fill='both',expand=True,padx=8)
    row.grid_propagate(False)  # Video images must not push the controls below the window.
    row.columnconfigure(0,weight=1); row.columnconfigure(1,weight=1); row.rowconfigure(0,weight=1)
    panels, captions={},{}
    for index,name in enumerate(NAMES):
        group=korean_group(row,'외장 RGB · 손끝' if name=='wrist' else 'D435 RGB · 작업대 전체')
        group.grid(row=0,column=index,sticky='nsew',padx=4)
        panels[name]=ttk.Label(group,anchor='center')
        captions[name]=KoreanLabel(group,text='영상 연결 중',anchor='center')
        captions[name].pack(side='bottom',fill='x')
        panels[name].pack(fill='both',expand=True)
    detail=KoreanLabel(root,text='',size=16,wraplength=1250)
    detail.pack(fill='x',padx=12,pady=5)
    progress_label=KoreanLabel(root,text='',size=16)
    progress_label.pack(fill='x',padx=12)
    saving_since=None
    bar=ttk.Frame(root); bar.pack(pady=8)
    state={'stage':'connecting'}
    closing=False
    buttons={}
    labels=[('enter','시작 / 다시 준비 (Enter)'),('y','리더에 동기화 (Y)'),('s','녹화 (S)'),('e','저장 (E)'),
            ('c','현재 취소 (C)'),('r','현재 재시작 (R)'),('q','종료 / 정지 (Esc)')]
    def send(key):
        nonlocal closing
        if key!='q' and str(buttons[key]['state'])=='disabled': return
        try: commands.put_nowait(key)
        except queue.Full: return
        if key=='q':
            closing=True
            heading.config(text='정지 요청 중 · 토크 해제와 파일 정리를 기다리는 중입니다.')
            for button in buttons.values(): button.config(state='disabled')
    for key,label in labels:
        buttons[key]=KoreanButton(bar,text=label,command=lambda k=key:send(k))
        buttons[key].pack(side='left',padx=3)
    for key in ('s','e','c','r','q','y'):
        root.bind(key,lambda event,k=key:send(k))
        root.bind(key.upper(),lambda event,k=key:send(k))
    root.bind('<Return>',lambda event:send('enter'))
    root.bind('<Escape>',lambda event:send('q'))
    root.protocol('WM_DELETE_WINDOW',lambda:send('q'))
    KoreanLabel(root,text='1: 손끝 확대   2: 전체 화면 확대   0: 두 화면   ·   F11: 전체 화면',anchor='center').pack(fill='x',pady=3)
    focus={'name':None,'fullscreen':False}
    groups={name:panels[name].master for name in NAMES}
    def layout(name):
        focus['name']=name
        for index,camera in enumerate(NAMES):
            groups[camera].grid_remove()
            if name is None: groups[camera].grid(row=0,column=index,columnspan=1,sticky='nsew',padx=4)
            elif camera==name: groups[camera].grid(row=0,column=0,columnspan=2,sticky='nsew',padx=4)
    root.bind('1',lambda event:layout('wrist'))
    root.bind('2',lambda event:layout('overview'))
    root.bind('0',lambda event:layout(None))
    def fullscreen(event):
        focus['fullscreen']=not focus['fullscreen']; root.attributes('-fullscreen',focus['fullscreen'])
    root.bind('<F11>',fullscreen)
    last={name:0 for name in NAMES}
    counts={name:[] for name in NAMES}
    while_status={'stage':'connecting','text':'카메라 연결 중','detail':''}

    def refresh():
        nonlocal saving_since
        heartbeat.value=time.monotonic()
        if quit_event.is_set(): root.destroy(); return
        try:
            while True:
                status=statuses.get_nowait()
                while_status.update(status)
                if status.get('saved_count') is not None:
                    count=status['saved_count']
                    index=status['episode_index']
                    pending=status.get('pending_count')
                    text=(f'학습용 변환 완료: {count}개 · 원본 저장/변환 대기: {pending}개 · 다음 시연: {count+pending+1}번째' if pending is not None else
                          f'저장 완료: {count}개 · 다음: {count+1}번째 (번호 {count})' if status.get('progress')==100 else
                          f'저장 완료: {count}개 · 현재 저장 중: {index+1}번째 (번호 {index})')
                    episode_label.config(text=text)
                if status.get('progress')==100:
                    progress_label.config(text='저장 완료 · 100%')
        except queue.Empty: pass
        if not closing:
            state.update(while_status)
            heading.config(text=state['text'])
            detail.config(text=state.get('detail',''))
            allowed={'idle':{'enter','y','q'},'ready':{'s','y','c','r','q'},
                     'recording':{'e','c','r','q'},'syncing':{'c','r','q'},
                     'connecting':{'q'},'saving':{'q'},'error':{'q'}}.get(state['stage'],{'q'})
            for key,button in buttons.items(): button.config(state='normal' if key in allowed else 'disabled')
        now=time.monotonic()
        if while_status['stage']=='saving':
            if saving_since is None: saving_since=now
            elapsed=int(now-saving_since)
            percent=while_status.get('progress')
            value='준비 중' if percent is None else f'{int(percent)}%'
            progress_label.config(text=f'저장 진행률(예상): {value} · 경과 {elapsed//60:02d}:{elapsed%60:02d}')
        else:
            saving_since=None
            # Keep completion visible while waiting for the next episode.
            if while_status['stage']!='idle': progress_label.config(text='')
        for name in NAMES:
            stamp=stamps[name].value
            age=max(0,time.monotonic()-stamp) if stamp else float('inf')
            if stamp!=last[name] and locks[name].acquire(False):
                try: frame=np.frombuffer(buffers[name],dtype=np.uint8).reshape(SHAPE).copy()
                finally: locks[name].release()
                last[name]=stamp
                counts[name].append(now)
                counts[name]=[t for t in counts[name] if now-t<2]
                if focus['name'] in (None,name):
                    width=max(320,panels[name].winfo_width())
                    height=max(240,panels[name].winfo_height())
                    scale=min(width/640,height/480)
                    displayed = preview_frame(frame, name, roi_state['config'], roi_preview.get())
                    photo=ImageTk.PhotoImage(displayed.resize((int(640*scale),int(480*scale))))
                    panels[name].configure(image=photo)
                    panels[name].image=photo
            if age>.25:
                captions[name].configure(text='영상 지연 / 연결 확인 필요',foreground='red')
            else:
                captions[name].configure(text=f'최근 수신 {age*1000:.0f} ms 전 · RGB 640×480',foreground='black')
        root.after(25,refresh)
    refresh()
    root.mainloop()


class RecordView:
    def __init__(self):
        ctx=mp.get_context('spawn')
        self.buffers={name:ctx.RawArray('B',int(np.prod(SHAPE))) for name in NAMES}
        self.locks={name:ctx.Lock() for name in NAMES}
        self.stamps={name:ctx.Value('d',0) for name in NAMES}
        self.heartbeat=ctx.Value('d',time.monotonic())
        self.statuses=ctx.Queue(maxsize=20)
        self.commands=ctx.Queue(maxsize=30)
        self.quit_event=ctx.Event()
        self.process=ctx.Process(target=viewer_main,args=(self.buffers,self.locks,self.stamps,self.heartbeat,
                                                         self.statuses,self.commands,self.quit_event))
        self.process.start()
        self.last_status=0

    def publish(self,name,rgb,stamp):
        if name not in self.buffers or not self.locks[name].acquire(False): return
        try:
            np.copyto(np.frombuffer(self.buffers[name],dtype=np.uint8).reshape(SHAPE),rgb)
            self.stamps[name].value=stamp
        finally: self.locks[name].release()

    def update(self,stage,text,detail='',*,force=True,progress=None,episode_index=None,saved_count=None,pending_count=None):
        now=time.monotonic()
        if not force and now-self.last_status<.2: return
        try: self.statuses.put_nowait({'stage':stage,'text':text,'detail':detail,'progress':progress,
                                      'episode_index':episode_index,'saved_count':saved_count,'pending_count':pending_count})
        except queue.Full: return
        self.last_status=now

    def check(self):
        if not self.process.is_alive(): raise RuntimeError('Camera window closed unexpectedly; stopping robot')
        if time.monotonic()-self.heartbeat.value>3:
            raise RuntimeError('Camera window is unresponsive; stopping robot')

    def poll(self):
        self.check()
        try: return self.commands.get_nowait()
        except queue.Empty: return None

    def clear_commands(self):
        # Keep stop requests; drop buffered hotkeys across state transitions.
        quit_requested=False
        while True:
            try:
                if self.commands.get_nowait()=='q': quit_requested=True
            except queue.Empty: break
        return quit_requested

    def wait_ready(self,text):
        if self.clear_commands(): return False
        self.update('idle','대기 · 토크 OFF',text)
        while True:
            command=self.poll()
            if command=='q': return False
            if command in ('enter','y'): return True
            time.sleep(.03)

    def close(self):
        self.quit_event.set()
        self.process.join(timeout=4)
        if self.process.is_alive():
            self.process.terminate(); self.process.join(timeout=2)
        for q in (self.statuses,self.commands):
            q.cancel_join_thread(); q.close()
