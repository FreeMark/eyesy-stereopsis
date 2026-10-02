import os
import pygame
import glob
import random

#Knob1 - x pos
#Knob2 - y pos
#Knob3 - x scale
#Knob4 - y scale
#Knob5 - background color

image_index = 0

def setup(screen, eyesy) :
    global last_screen, xr, yr
    xr = eyesy.xres
    yr = eyesy.yres
    last_screen = pygame.Surface((xr,yr))
    

_echo = {}  # stereopsis: the echo image, captured once per 30 fps frame

def draw(screen, eyesy) :
    global last_screen, xr, yr
    tick = getattr(eyesy, 'tick', True)  # stereopsis: True once per 30 fps frame's worth of time
    eyesy.color_picker_bg(eyesy.knob5)    
    cscale = int(xr*.04)
   
    
    if eyesy.trig :
        color = eyesy.color_picker((random.randrange(0,100)*.01))
        x = random.randrange(int((cscale/2)*-1),xr)
        y = random.randrange(int((cscale/2)*-1),yr)
        pygame.draw.circle(screen,color,[x,y],cscale)

    if tick or 'img' not in _echo:  # stereopsis: the echo is captured once per 30 fps frame, as on stock
        _echo['img'] = last_screen
        last_screen = screen.copy()
    image = _echo['img']
    thing = pygame.transform.scale(image, (int(eyesy.knob3 * xr), int(eyesy.knob4 * yr) ) )
    thing = pygame.transform.flip(thing, 1,0)
    screen.blit(thing, (int(eyesy.knob1 * xr), int(eyesy.knob2 * yr)))
