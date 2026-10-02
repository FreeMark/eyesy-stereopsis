import os
import pygame
import random

#Knob1 - number of 'fish'
#Knob2 - fish length 
#Knob3 - line width
#Knob4 - foreground color
#Knob5 - background color

speedList = [random.randrange(-1,1)+.1 for i in range(0,20)]
yList = [random.randrange(-50,770) for i in range(0,20)]
widthList = [random.randrange(20,200) for i in range(0,20)]
countList = [i for i in range(0,20)]
xden = 1
yden = 1
trigger = False
color_direction = 1
color_rate = 0.0

def update_color(sel, eyesy):
    global color_rate, color, color_direction
    if sel < 1:
        color = eyesy.color_picker(eyesy.knob4 * 2)
    elif sel >= 1:
        increment = (abs(sel - 1) * 0.05)
        color_rate += increment * color_direction
        if color_rate >= 1.0:
            color_rate = 1.0
            color_direction = -1  # Start decrementing
        if color_rate <= 0.0:
            color_rate = 0.0
            color_direction = 1  # Start incrementing
        color = eyesy.color_picker(color_rate)
    return color

def setup(screen, eyesy) :
    global xres, yres, widthmax
    xres = eyesy.xres
    yres = eyesy.yres
    widthmax = int(xres * 0.156)

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

def draw(screen, eyesy) :
    global trigger, yList, widthList, countList, speedList, xden, yden, color, color_rate, widthmax, xres, yres
    tick = getattr(eyesy, 'tick', True)  # stereopsis: True once per 30 fps frame's worth of time
    global color_direction  # stereopsis: replayed below (update_color sets it)
    countList, color_rate, color_direction = _replay_state(tick, (countList, color_rate, color_direction))
    eyesy.color_picker_bg(eyesy.knob5)    
    sel = eyesy.knob4*2 #color selector

    if yden != (int(eyesy.knob1 * 19) + 1) :
        yden = (int(eyesy.knob1 * 19) + 1)
        speedList = [random.randrange(-2,2)+.1 for i in range(0,20)]
        yList = [random.randrange(-50,(eyesy.yres+50)) for i in range(0,20)]
        widthList = [random.randrange(20,widthmax) for i in range(0,20)]

    if xden != (int(eyesy.knob2 * 19) + 1) :
        xden = (int(eyesy.knob2 * 19) + 1)
        speedList = [random.randrange(-2,2)+.1 for i in range(0,20)]
        yList = [random.randrange(-50,(eyesy.yres+50)) for i in range(0,20)]
        widthList = [random.randrange(20,widthmax) for i in range(0,20)]
        
    for i in range (0,yden) :

        y0 = yList[i]
        ymod = yres * 0.694 #((500*eyesy.yres)/eyesy.yres)

        for j in range (0,xden) :
            update_color(sel, eyesy)
            width = widthList[i]
            y1 = y0 + (eyesy.audio_in[j+i] / ymod)
            countList[i] = countList[i] + speedList[i]
            modSpeed = countList[i]%(eyesy.xres+width*2)
            x = (j * (width/5)) + (modSpeed-width)
            pygame.draw.line(screen, color, [x, y1], [x, y0], int(eyesy.knob3*(xres*0.078)+1))
    
    if eyesy.trig:
        trigger = True

    if trigger == True:
        speedList = [random.randrange(-2,2)+.1 for i in range(0,20)]
        yList = [random.randrange(-50,int(eyesy.yres+50)) for i in range(0,20)]
        widthList = [random.randrange(20,widthmax) for i in range(0,20)]
        trigger = False
 
