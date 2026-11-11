#!/usr/bin/env python3
"""Show a frame-number marker through RDP; record issued draws, not viewport FPS."""
import argparse
import csv
from pathlib import Path
import time
import tkinter as tk

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds',type=float,default=60.)
parser.add_argument('--rate',type=float,default=60.)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args()
if not 1<=a.rate<=120 or not 1<=a.seconds<=900:parser.error('rate 1..120, seconds 1..900')
a.output.parent.mkdir(parents=True,exist_ok=True)
if a.output.exists():parser.error('output already exists')
root=tk.Tk();root.title('RDP delivery probe — click CYAN square in client browser')
root.attributes('-topmost',True)
canvas=tk.Canvas(root,width=360,height=64,bg='#202020',highlightthickness=0);canvas.pack()
canvas.create_text(180,54,text='Click cyan square with the client bookmarklet armed',fill='white')
cells=[[canvas.create_rectangle(i*20,row*20,(i+1)*20,row*20+20,outline='') for i in range(18)] for row in range(2)]
stream=a.output.open('w');writer=csv.writer(stream);writer.writerow(('monotonic_seconds','frame_id'))
start=time.monotonic();counter=0

def tick():
    global counter
    now=time.monotonic()
    if now-start>=a.seconds:root.destroy();return
    counter+=1
    for row in range(2):
        for i,item in enumerate(cells[row]):
            color='#00ffff' if i==0 else '#ff00ff' if i==17 else '#ffffff' if (((counter>>(i-1))&1)^row) else '#000000'
            canvas.itemconfigure(item,fill=color)
    root.update_idletasks()
    writer.writerow((time.monotonic(),counter));stream.flush()
    root.after(max(1,round(1000/a.rate)),tick)
root.after(0,tick)
try:root.mainloop()
finally:stream.close()
