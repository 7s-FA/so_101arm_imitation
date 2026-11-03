"""Draw fixed image rectangles on saved footage; never opens cameras or motors."""
import argparse
from pathlib import Path

import numpy as np

from roi_mask import load, mask_rgb, save, validate


def reference_frame(path):
    from PIL import Image
    if path.suffix.lower() in ('.mp4', '.avi', '.mkv'):
        import cv2
        capture = cv2.VideoCapture(str(path))
        try:
            ok, bgr = capture.read()
            if not ok: raise ValueError('영상의 첫 프레임을 읽을 수 없습니다.')
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        finally:
            capture.release()
    else:
        with Image.open(path) as image:
            rgb = np.asarray(image.convert('RGB')).copy()
    if rgb.shape != (480, 640, 3):
        raise ValueError('학습 카메라와 같은 640×480 이미지/영상을 선택하세요. 휴대폰 사진은 사용할 수 없습니다.')
    return rgb


def main():
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    from PIL import Image, ImageTk
    from gui_text import KoreanButton, KoreanLabel
    from projects import load as load_project

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', required=True)
    parser.add_argument('--image', type=Path, help='640x480 overview image or recorded video')
    args = parser.parse_args()
    root, _ = load_project(args.project)
    target = root/'roi_mask.json'
    config = load(target) if target.is_file() else {
        'version': 1, 'size': [640, 480], 'overview_boxes': [], 'wrist': 'full'}
    window = tk.Tk()
    window.title(f'{args.project} — ROI')
    KoreanLabel(window, text='D435 영상에서 사용할 영역을 마우스로 드래그하세요. 여러 박스의 내부만 학습에 사용합니다.\n'
                '부품이 있는 지그와 목적지 지그를 여유 있게 포함하세요. 고정 영역이며 지그를 자동 추적하지 않습니다.',
                wraplength=680).pack(padx=12, pady=8)
    source_label = KoreanLabel(window, wraplength=680)
    source_label.pack(padx=12)
    canvas = tk.Canvas(window, width=640, height=480, highlightthickness=0, bg='black')
    canvas.pack(padx=12, pady=8)
    state = {'rgb': None, 'path': None, 'anchor': None, 'drag': None, 'photo': None}
    preview = tk.BooleanVar(value=False)
    wrist = tk.BooleanVar(value=config['wrist'] == 'full')
    status = KoreanLabel(window, wraplength=680)

    def redraw():
        canvas.delete('all')
        if state['rgb'] is not None:
            rgb = state['rgb']
            if preview.get() and config['overview_boxes']:
                rgb = mask_rgb(rgb, 'overview', config)
            state['photo'] = ImageTk.PhotoImage(Image.fromarray(rgb), master=window)
            canvas.create_image(0, 0, image=state['photo'], anchor='nw')
        for index, (x1, y1, x2, y2) in enumerate(config['overview_boxes'], 1):
            canvas.create_rectangle(x1, y1, x2-1, y2-1, outline='#00ff70', width=2)
            canvas.create_text(x1+5, y1+5, text=str(index), fill='#00ff70', anchor='nw')
        status.config(text=f'박스 {len(config["overview_boxes"])}개 · 저장 후 다음 학습부터 적용됩니다.\n'
                      '기존 모델은 재학습이 필요합니다. 원본 영상은 변경하지 않습니다.')

    def open_reference(path):
        try:
            rgb = reference_frame(path)
            state.update(rgb=rgb, path=path)
            source_label.config(text=f'저장된 참고 영상 (실시간 아님): {path}')
            redraw()
        except Exception as exc:
            messagebox.showerror('참고 영상 열기 실패', str(exc), parent=window)

    def choose():
        path = filedialog.askopenfilename(parent=window, initialdir=root/'dataset/videos',
                    filetypes=[('Image / video', '*.jpg *.png *.jpeg *.mp4 *.avi *.mkv')])
        if path: open_reference(Path(path))

    def point(event): return max(0, min(640, event.x)), max(0, min(480, event.y))

    def begin(event):
        if state['rgb'] is None: return
        state['anchor'] = point(event)
        state['drag'] = canvas.create_rectangle(*state['anchor'], *state['anchor'], outline='yellow', width=2)

    def drag(event):
        if state['anchor'] is not None: canvas.coords(state['drag'], *state['anchor'], *point(event))

    def end(event):
        if state['anchor'] is None: return
        x1, y1 = state['anchor']; x2, y2 = point(event)
        box = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
        state['anchor'] = None
        if box[2]-box[0] >= 5 and box[3]-box[1] >= 5 and len(config['overview_boxes']) < 16:
            config['overview_boxes'].append(box)
        redraw()

    def undo():
        if config['overview_boxes']: config['overview_boxes'].pop()
        redraw()

    def reset():
        config['overview_boxes'] = []
        redraw()

    def commit():
        try:
            if state['rgb'] is None: raise ValueError('먼저 D435 참고 영상을 여세요.')
            config['wrist'] = 'full' if wrist.get() else 'black'
            validate(config)
            save(target, config)
            status.config(text=f'저장 완료: {target}\n다음 학습에 적용됩니다. 학습 중인 모델/기존 모델에는 영향을 주지 않습니다.')
        except Exception as exc: messagebox.showerror('저장 실패', str(exc), parent=window)

    canvas.bind('<ButtonPress-1>', begin)
    canvas.bind('<B1-Motion>', drag)
    canvas.bind('<ButtonRelease-1>', end)
    controls = ttk.Frame(window); controls.pack(pady=4)
    for title, action in [('참고 영상 열기', choose), ('마지막 박스 취소', undo), ('박스 초기화', reset), ('영역 저장', commit)]:
        KoreanButton(controls, text=title, command=action).pack(side='left', padx=4)
    options = ttk.Frame(window); options.pack(pady=4)
    ttk.Checkbutton(options, variable=preview, command=redraw).pack(side='left')
    KoreanLabel(options, text='마스크 미리보기').pack(side='left')
    ttk.Checkbutton(options, variable=wrist).pack(side='left', padx=(20, 0))
    KoreanLabel(options, text='손목 영상 전체 유지').pack(side='left')
    status.pack(padx=12, pady=8)
    reference = args.image
    if reference is None:
        reference = next(iter(sorted((root/'dataset/videos/observation.images.overview').rglob('*.mp4'))), None)
    if reference: open_reference(reference)
    else:
        source_label.config(text='640×480 D435 참고 영상/이미지를 여세요.')
        redraw()
    window.mainloop()


if __name__ == '__main__': main()
