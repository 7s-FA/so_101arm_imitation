"""Project selection and launch; never switches an active recorder's project."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

from project_paths import PROJECTS
from capture_queue import pending_sessions

BUNDLE = Path(__file__).resolve().parents[1]


def load(name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name):
        raise ValueError('이름은 영문·숫자·밑줄·하이픈만 사용하세요.')
    root = PROJECTS/name
    if root.is_symlink(): raise ValueError('Project root cannot be a symlink')
    return root, json.loads((root/'project.json').read_text())


def create(name, instruction):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',name): raise ValueError('잘못된 프로젝트 이름')
    if not instruction.strip(): raise ValueError('작업 설명을 입력하세요.')
    root = PROJECTS/name
    root.mkdir(parents=True, exist_ok=False)
    for sub in ['captures','models','logs','completion','references']: (root/sub).mkdir()
    task = {'name':name, 'instruction':instruction.strip(), 'order':[],
            'dataset_fps':30, 'max_session_seconds':None, 'repo_id':f'local/{name}',
            'dataset_directory':str((root/'dataset').resolve()),
            'success':'User confirms all parts are fully seated and the gripper is withdrawn.'}
    (root/'task.json').write_text(json.dumps(task,ensure_ascii=False,indent=2)+'\n')
    shutil.copy2(BUNDLE/'training_act.json',root/'training_act.json')
    (root/'project.json').write_text(json.dumps({'name':name,'legacy':False},indent=2)+'\n')
    return root


def launch_command(name, action, extra=()):
    root, config = load(name)
    env = dict(os.environ)
    env.pop('PYTHONPATH',None); env.pop('PYTHONHOME',None); env.pop('ARM3_PROJECT',None)
    if not config.get('legacy'): env['ARM3_PROJECT'] = str(root.resolve())
    py = str(BUNDLE/'.venv/bin/python')
    scripts = {'record':BUNDLE/'arm3/record_purple.py', 'pose':BUNDLE/'arm3/start_pose.py',
               'train':BUNDLE/'train_act.py', 'run':BUNDLE/'arm3/run_policy.py',
               'roi':BUNDLE/'arm3/roi_editor.py'}
    command = [py,'-u',str(scripts[action])]
    if action == 'pose': command += ['save']
    if action == 'train':
        command += ['--device','cuda']
        if (root/'roi_mask.json').is_file() and not any(arg=='--roi' or arg.startswith('--roi=') or arg=='--no-roi' for arg in extra):
            command += ['--roi',str(root/'roi_mask.json')]
    if action == 'roi': command += ['--project',name]
    if action == 'run':
        env.update(HF_HUB_OFFLINE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2')
        if '--model' not in extra: raise ValueError('실행할 --model 체크포인트 경로를 지정하세요.')
    return command+list(extra), env


def summary(name):
    root, config = load(name)
    dataset = root/'dataset'
    info = dataset/'meta/info.json'
    count = json.loads(info.read_text())['total_episodes'] if info.exists() else 0
    task = json.loads((root/'task.json').read_text())
    pending = len(pending_sessions(root/'captures'))
    return (f'프로젝트: {name}  |  학습용 변환 완료: {count}개  |  원본 변환 대기: {pending}개\n'
            f'데이터: {dataset}\n학습 모델: {root / "models"}\n'
            f'시작 자세: {"저장됨" if (root/"start_pose.json").exists() else "저장 필요"}\n'
            f'학습 영상: {"지정 영역 사용 (ROI)" if (root/"roi_mask.json").exists() else "전체 영상"}\n'
            f'작업: {task["instruction"]}')


def gui():
    import tkinter as tk
    from tkinter import ttk, simpledialog, messagebox
    from gui_text import KoreanLabel, KoreanButton
    window = tk.Tk(); window.title('SO101 프로젝트 관리')
    selected = tk.StringVar(value='model2')
    picker = ttk.Combobox(window,textvariable=selected,state='readonly',width=40)
    picker.pack(padx=20,pady=12)
    label = KoreanLabel(window,wraplength=900); label.pack(padx=20,pady=8)
    status = KoreanLabel(window,wraplength=900); status.pack(padx=20,pady=8)
    state = {'process':None,'log':None,'handle':None}
    buttons=[]
    def refresh(*_):
        picker['values'] = sorted(p.name for p in PROJECTS.iterdir() if (p/'project.json').is_file())
        try: label.config(text=summary(selected.get()))
        except Exception as exc: status.config(text=str(exc))
    picker.bind('<<ComboboxSelected>>',refresh)
    def start(action, *, defer_export=False, export_pending=False):
        try:
            name = selected.get(); root,_ = load(name)
            extra=[]
            if defer_export: extra.append('--defer-export')
            if export_pending: extra.append('--export-pending')
            if action=='train':
                if not (root/'dataset/meta/info.json').exists(): raise ValueError('먼저 시연을 저장하세요.')
                run_name=simpledialog.askstring('학습 결과 이름','학습 결과 폴더 이름:',initialvalue=name,parent=window)
                if run_name is None: return
                if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',run_name): raise ValueError('잘못된 이름')
                output=root/'models'/run_name
                if output.exists(): raise ValueError('이미 있는 모델 폴더입니다. 다른 이름을 사용하세요.')
                extra=['--output',str(output)]
            command,env=launch_command(name,action,extra)
            path=root/'logs'/f'{action}_{datetime.now():%Y%m%d_%H%M%S_%f}.log'
            handle=path.open('w')
            try: process=subprocess.Popen(command,env=env,cwd=BUNDLE,stdout=handle,stderr=subprocess.STDOUT)
            except Exception:
                handle.close(); raise
            state.update(process=process,log=path,handle=handle)
            picker.config(state='disabled')
            for button in buttons: button.config(state='disabled')
            status.config(text=f'{name}: {action} 실행 중\n로그: {path}')
        except Exception as exc: messagebox.showerror('실행할 수 없음',str(exc),parent=window)
    def new():
        name=simpledialog.askstring('새 프로젝트','프로젝트 이름 (예: model3):',parent=window)
        if name is None: return
        instruction=simpledialog.askstring('작업 설명','옮기는 순서와 도착 위치를 입력하세요:',parent=window)
        if instruction is None: return
        try: create(name,instruction); selected.set(name); refresh()
        except Exception as exc: messagebox.showerror('생성 실패',str(exc),parent=window)
    for title,command in [('새 프로젝트 만들기',new),
                          ('시작 자세 저장 (모터 OFF · 팔을 받쳐 고정)',lambda:start('pose')),
                          ('선택한 프로젝트 녹화',lambda:start('record')),
                          ('빠른 녹화 (원본 저장 · 변환은 나중에)',lambda:start('record',defer_export=True)),
                          ('대기 원본을 학습용 데이터로 변환',lambda:start('record',export_pending=True)),
                          ('학습 영상 영역 설정 (ROI)',lambda:start('roi')),
                          ('선택한 프로젝트 학습',lambda:start('train'))]:
        button=KoreanButton(window,text=title,command=command); button.pack(pady=6); buttons.append(button)
    KoreanLabel(window,text='녹화는 열린 카메라 창에서 시작합니다. 학습 설정은 프로젝트의 training_act.json에서 조절합니다.\n새 작업의 자동 완료 판정은 별도 카메라 기준 등록이 필요합니다.',wraplength=900).pack(padx=20,pady=10)
    def poll():
        proc=state['process']
        if proc is not None and proc.poll() is not None:
            state['handle'].close()
            tail=state['log'].read_text(errors='replace')[-1600:]
            status.config(text=f'종료 코드: {proc.returncode}\n로그: {state["log"]}\n{tail}')
            state['process']=None
            picker.config(state='readonly')
            for button in buttons: button.config(state='normal')
            refresh()
        window.after(500,poll)
    def close():
        if state['process'] is not None:
            messagebox.showinfo('실행 중','녹화 창에서 종료하거나 학습이 끝난 뒤 관리 창을 닫으세요.',parent=window)
        else: window.destroy()
    window.protocol('WM_DELETE_WINDOW',close)
    refresh(); poll(); window.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project',nargs='?')
    parser.add_argument('action',nargs='?',choices=['record','pose','train','run','show','roi'])
    args,extra=parser.parse_known_args()
    if not args.project:
        if extra: parser.error('Unexpected options')
        return gui()
    if not args.action: parser.error('Specify record, pose, train, run, roi or show')
    if args.action=='show': print(summary(args.project)); return
    command,env=launch_command(args.project,args.action,extra)
    os.execve(command[0],command,env)


if __name__=='__main__': main()
