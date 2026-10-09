#!/usr/bin/env python3
"""Capture only the RViz XWayland window via xwd; decode its native pixel layout."""
import argparse
import ctypes
import re
import struct
import subprocess
import time
from pathlib import Path
import numpy as np
from PIL import Image


def activate_window(window):
    # XWayland xwd reads visible pixels; request activation before capture.
    x=ctypes.CDLL('libX11.so.6')
    pointer=ctypes.c_void_p
    x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=pointer
    x.XDefaultRootWindow.argtypes=[pointer];x.XDefaultRootWindow.restype=ctypes.c_ulong
    x.XInternAtom.argtypes=[pointer,ctypes.c_char_p,ctypes.c_int];x.XInternAtom.restype=ctypes.c_ulong
    x.XSendEvent.argtypes=[pointer,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,pointer]
    window_type=ctypes.c_ulong
    x.XQueryTree.argtypes=[pointer,window_type,ctypes.POINTER(window_type),
                          ctypes.POINTER(window_type),ctypes.POINTER(ctypes.POINTER(window_type)),
                          ctypes.POINTER(ctypes.c_uint)]
    x.XRaiseWindow.argtypes=[pointer,window_type]
    x.XSetInputFocus.argtypes=[pointer,window_type,ctypes.c_int,window_type]
    x.XFree.argtypes=[pointer]
    x.XFlush.argtypes=[pointer];x.XCloseDisplay.argtypes=[pointer]
    class Data(ctypes.Union):
        _fields_=[('l',ctypes.c_long*5)]
    class Message(ctypes.Structure):
        _fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),
                  ('display',pointer),('window',ctypes.c_ulong),('message_type',ctypes.c_ulong),
                  ('format',ctypes.c_int),('data',Data)]
    class Event(ctypes.Union):
        _fields_=[('message',Message),('pad',ctypes.c_long*24)]
    display=x.XOpenDisplay(None)
    if not display:return
    event=Event()
    event.message.type=33
    event.message.send_event=1
    event.message.display=display
    event.message.window=int(window,16)
    event.message.message_type=x.XInternAtom(display,b'_NET_ACTIVE_WINDOW',0)
    event.message.format=32
    event.message.data.l[0]=2
    event.message.data.l[1]=0
    x.XSendEvent(display,x.XDefaultRootWindow(display),0,(1<<19)|(1<<20),ctypes.byref(event))
    root=window_type();parent=window_type();children=ctypes.POINTER(window_type)();count=ctypes.c_uint()
    if x.XQueryTree(display,int(window,16),ctypes.byref(root),ctypes.byref(parent),
                    ctypes.byref(children),ctypes.byref(count)):
        if children:x.XFree(children)
        x.XRaiseWindow(display,parent.value if parent.value!=root.value else int(window,16))
        x.XSetInputFocus(display,int(window,16),1,0)
    x.XFlush(display);x.XCloseDisplay(display)
    time.sleep(0.5)


def capture(output):
    listing=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    line=next((l for l in listing.splitlines() if 'semantic_indoor.rviz' in l and 'RViz' in l),None)
    if not line:
        raise RuntimeError('Semantic RViz window is not running')
    window=re.search(r'0x[0-9a-f]+',line).group(0)
    activate_window(window)
    xwd=output.with_suffix('.xwd')
    subprocess.run(['xwd','-silent','-id',window,'-out',str(xwd)],check=True)
    b=xwd.read_bytes()
    h=struct.unpack('>25I',b[:100])
    header_size,_,_,_,width,height,_,order,_,_,_,bpp,stride,_,red,green,blue,_,_,ncolors,*_=h
    rows=np.frombuffer(b[header_size+ncolors*12:],dtype=np.uint8).reshape(height,stride)
    if bpp==24:
        a=rows[:,:width*3].reshape(height,width,3)
        rgb=a[:,:,::-1] if order==0 else a
    elif bpp==32:
        raw=rows[:,:width*4].copy().view('<u4' if order==0 else '>u4').reshape(height,width)
        rgb=np.empty((height,width,3),dtype=np.uint8)
        for i,mask in enumerate((red,green,blue)):
            shift=(mask&-mask).bit_length()-1
            rgb[:,:,i]=((raw&mask)>>shift)*255//(mask>>shift)
    else:
        raise ValueError(f'Unsupported XWD bits_per_pixel={bpp}')
    Image.fromarray(rgb).save(output)
    print(output)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('output',type=Path)
    capture(p.parse_args().output)
