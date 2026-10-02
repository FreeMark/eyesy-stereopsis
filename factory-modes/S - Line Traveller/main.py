import os
import pygame
import time
import math

#Knob1 - size
#Knob2 - y speed
#Knob3 - x speed
#Knob4 - foreground color
#Knob5 - background color

def setup(screen, eyesy):
    global y, x, xr, yr, xhalf, yhalf
    xr = eyesy.xres
    yr = eyesy.yres
    xhalf = xr/2
    yhalf = yr/2
    y = 0
    x = 0

def draw(screen, eyesy):
    global y, x, xr, yr, xhalf, yhalf

    step = getattr(eyesy, 'step', 1.0)  # stereopsis: this frame's share of a 30 fps frame (0.5 at 60 fps)
    eyesy.color_picker_bg(eyesy.knob5)    

    yspeed = int(eyesy.knob2*20)
    xspeed = int(eyesy.knob3*40)
    thick = int(eyesy.knob1*yhalf)+1
    peak = 0
    
    for i in range(0,1) :
        peak = eyesy.audio_in[i*50]
    
    L =  peak/6+1
    x1 = x - L
    x2 = x + L
    
    y = y + yspeed * step
    if y > yr + yspeed * (1 - step) : y = 0  # stereopsis: stock's wrap

    x = x + xspeed * step
    if x > xr + xspeed * (1 - step) : x = 0  # stereopsis: stock's wrap

    color = eyesy.color_picker_lfo(eyesy.knob4)
    pygame.draw.line(screen, color, [x1, y], [x2, y], thick)
