import os
import pygame
import pygame.gfxdraw

#Knob1 - rate of change for number of triangles
#Knob2 - feather angle
#Knob3 - y position step amount (bounce speed)
#Knob4 - foreground color
#Knob5 - background color


triangles = 10

class LFO : #uses three arguments: start point, max, and how far each step is.

    def __init__(self, start, max, step):
        self.start = start
        self.max = max
        self.step = step
        self.current = 0
        self.direction = 1

    # stereopsis: call once per frame, before update(), with eyesy.step. Within a frame update() steps
    # exactly as on stock (a mode that calls it for each of its rows keeps its pattern); from frame to
    # frame the LFO moves scale x what the last frame's calls moved it: stock's speed at any frame rate
    def frame(self, scale=1.0):
        s0 = getattr(self, '_s0', None)
        if s0 is not None:
            self.current, self.direction = s0
            k, st = getattr(self, '_n', 0) * scale, self.step
            while k > 1e-9:
                self.step = st * min(k, 1.0)
                self.update()
                k -= 1.0
            self.step = st
        self._s0, self._n = (self.current, self.direction), 0

    def update(self):
        self._n = getattr(self, '_n', 0) + 1  # stereopsis: calls this frame (see frame())
        self.current += self.step * self.direction

    # when it gets to the top, flip direction
        if (self.current >= self.max) :
            self.direction = -1
            self.current = self.max  # in case it steps above max

    # when it gets to the bottom, flip direction
        if (self.current <= self.start) :
            self.direction = 1
            self.current = self.start  # in case it steps below min
        
        return self.current

yposr = LFO(0,500,10)
tris = LFO(2,70,1)

def setup(screen, eyesy) :
    global xr, yr
    xr = eyesy.xres
    yr = eyesy.yres
    yposr.start = int(yr - (yr*1.05))
    yposr.max = int(yr * 1.05)
    

def draw(screen, eyesy) :
    global triangles, xr, yr
    step = getattr(eyesy, 'step', 1.0)  # stereopsis: this frame's share of a 30 fps frame (0.5 at 60 fps)
    
    eyesy.color_picker_bg(eyesy.knob5)    
    color = eyesy.color_picker_lfo(eyesy.knob4) #on knob 4
    tris.max = int(xr * 0.047) #int((60*eyesy.xres)/eyesy.xres)
    tris.step = int(eyesy.knob1*(xr * 0.012)) #int(eyesy.knob1*((15*eyesy.xres)/eyesy.xres))
    tris.frame(step)  # stereopsis: stock's speed frame to frame
    triangles = int(tris.update())+2
    space = int(xr/(triangles-1))
    offset = int((eyesy.knob2*2-1)*space*4)
    yposr.step = int(eyesy.knob3*(yr * 0.1)) #int(eyesy.knob3*((72*eyesy.yres)/eyesy.yres))
    yposr.frame(step)  # stereopsis: stock's speed frame to frame
    y = int(yposr.update())
    
    pygame.draw.line(screen, color, (0, y), (xr,y)) #so you can see something in case no audio input
    
    for i in range (0,triangles) :
        
        auDio = int(eyesy.audio_in[i] / 65)
        ax = i * space
        pygame.gfxdraw.filled_trigon(screen, ax, y, ax+int((space/2)+offset),auDio+y, ax + space, y, color)
