"""Render Korean with FreeType when bundled Tk cannot see fontconfig fonts."""
from functools import lru_cache
from pathlib import Path
import subprocess
from tkinter import ttk

from PIL import Image, ImageDraw, ImageFont, ImageTk


@lru_cache(maxsize=1)
def font_location():
    result=subprocess.run(['fc-match','-f','%{file}\n%{index}\n','Noto Sans CJK KR'],
                          capture_output=True,text=True,check=True,timeout=3)
    parts=result.stdout.splitlines()
    if len(parts)<2 or not Path(parts[0]).is_file():
        raise RuntimeError('Korean font file not found')
    return parts[0],int(parts[1])


@lru_cache(maxsize=8)
def font(size):
    path,index=font_location()
    return ImageFont.truetype(path,size=size,index=index)


def text_image(text, size=15, color='black', wrap=0):
    face=font(size)
    lines=[]
    for paragraph in str(text).split('\n'):
        line=''
        for char in paragraph:
            if wrap and line and face.getlength(line+char)>wrap:
                lines.append(line); line=''
            line+=char
        lines.append(line)
    ascent,descent=face.getmetrics()
    height=ascent+descent+2
    width=max(1,int(max((face.getlength(line) for line in lines),default=0))+4)
    image=Image.new('RGBA',(width,max(1,height*len(lines))),(0,0,0,0))
    draw=ImageDraw.Draw(image)
    for index,line in enumerate(lines):
        draw.text((2,index*height),line,font=face,fill=color,anchor='lt')
    return image


class KoreanLabel(ttk.Label):
    def __init__(self,master,*,text='',size=15,wraplength=0,foreground='black',**kwargs):
        self.text_size,self.text_wrap=size,wraplength
        self.text_value,self.text_color=str(text),foreground
        self._rendered=None
        super().__init__(master,text=text,**kwargs)
        self._refresh_text()

    def _refresh_text(self):
        key=(self.text_value,self.text_color,self.text_size,self.text_wrap)
        if key==self._rendered: return
        self._photo=ImageTk.PhotoImage(text_image(self.text_value,self.text_size,self.text_color,self.text_wrap),master=self)
        super().configure(image=self._photo,text=self.text_value,compound='none')
        self._rendered=key

    def configure(self,cnf=None,**kwargs):
        if cnf is None and not kwargs: return super().configure()
        if cnf is not None:
            if not isinstance(cnf,dict): return super().configure(cnf,**kwargs)
            kwargs={**cnf,**kwargs}
        changed=False
        if 'text' in kwargs:
            self.text_value=str(kwargs.pop('text')); changed=True
        if 'foreground' in kwargs:
            self.text_color=kwargs.pop('foreground'); changed=True
        if kwargs: super().configure(**kwargs)
        if changed: self._refresh_text()

    config=configure


class KoreanButton(ttk.Button):
    def __init__(self,master,*,text,**kwargs):
        normal=ImageTk.PhotoImage(text_image(text),master=master)
        disabled=ImageTk.PhotoImage(text_image(text,color='#888888'),master=master)
        super().__init__(master,text=text,image=(normal,'disabled',disabled),compound='none',**kwargs)
        self._text_images=(normal,disabled)


def korean_group(master,text):
    group=ttk.LabelFrame(master)
    title=KoreanLabel(group,text=text)
    group.configure(labelwidget=title)
    group._title=title
    return group
