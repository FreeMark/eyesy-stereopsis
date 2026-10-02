import os
import pygame
import time
import random
import math

#x 128th, 8th, thick

#Knob1 - line rate of rotation & direction
#Knob2 - line length
#Knob3 - line thickness
#Knob4 - foreground color shift rate & direction
#Knob5 - background color

color_rate=0
speed=0

def setup(screen, eyesy):
    global xr, yr, x128, x8th
    xr = eyesy.xres
    yr = eyesy.yres
    x128 = int(xr * 0.1) #int((128*xr)/eyesy.xres)
    x8th = int(xr * 0.00625) #(8*xr)/eyesy.xres
    
_replay = {}  # stereopsis: see _replay_state


def _replay_state(tick, state):
    """stereopsis: state this mode advances element by element and carries from frame to frame. On a frame
    that completes a 30 fps frame (tick) keep a copy of it as the frame starts; on the frame between two, hand
    back that copy (it draws what the tick frame drew) and go on from the tick frame's end state next time"""
    import copy
    if 'end' in _replay:
        state = _replay.pop('end')
    if tick:
        _replay['start'] = copy.deepcopy(state)
    elif 'start' in _replay:
        _replay['end'] = state
        state = copy.deepcopy(_replay['start'])
    return state

def draw(screen, eyesy):
    global xr, yr, x128, x8th, color_rate, speed
    step = getattr(eyesy, 'step', 1.0)  # stereopsis: this frame's share of a 30 fps frame (0.5 at 60 fps)
    tick = getattr(eyesy, 'tick', True)  # stereopsis: True once per 30 fps frame's worth of time
    color_rate = _replay_state(tick, color_rate)
    eyesy.color_picker_bg(eyesy.knob5)
    thick = int(eyesy.knob3*(xr * 0.078))+1 #int(eyesy.knob3*((100*xr)/eyesy.xres))+1
    peak = 0
    lines = 5
    
    # Determine the rotation speed and direction based on eyesy.knob1
    if eyesy.knob1 < 0.48:
        speed = speed-(0.48-eyesy.knob1)*500 * step#((20*xr)/eyesy.xres)
    elif eyesy.knob1 > 0.52:
        speed = speed+(eyesy.knob1-0.52)*500 * step#((20*xr)/eyesy.xres)
    
    for i in range(lines) : 
    
        # Determine the color speed and direction based on eyesy.knob4
        if eyesy.knob4 < 0.48:
            color_rate = (color_rate - ((0.48-eyesy.knob4)*.09))%1.00
        elif eyesy.knob4 > 0.52:
            color_rate = (color_rate + ((eyesy.knob4-0.52)*.09))%1.00
        
        color = eyesy.color_picker((i*0.2+color_rate)%1.00)
        
        if eyesy.audio_in[i*10] > peak:
                peak = eyesy.audio_in[i*10]
                
        R = (4 * eyesy.knob2 * (peak / x128))+20
        x = R * math.cos((speed /  1000.) * 6.28) + (xr/2) + i*xr/lines-2*xr/lines
        y = R * math.sin((speed /  1000.) * 6.28) + (yr/2)
        pygame.draw.line(screen, color, [(i*xr/lines)+xr/10, (yr/2)], [x, y], thick)
