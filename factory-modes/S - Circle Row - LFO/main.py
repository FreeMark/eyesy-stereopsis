import os
import pygame
import pygame.gfxdraw

#Knob1 - number of triangles
#Knob2 - circle size
#Knob3 - y position step amount (bounce speed)
#Knob4 - foreground color
#Knob5 - background color

circles = 10

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

ypos = LFO(0,720,10)

def setup(screen, eyesy) :
    global xr, yr
    xr = eyesy.xres
    yr = eyesy.yres
    ypos.max = yr

def draw(screen, eyesy) :
    global xr, yr
    step = getattr(eyesy, 'step', 1.0)  # stereopsis: this frame's share of a 30 fps frame (0.5 at 60 fps)
    eyesy.color_picker_bg(eyesy.knob5)    
    color = eyesy.color_picker_lfo(eyesy.knob4) #on knob4
    ystepmod = yr * 0.069 #((50*eyesy.yres)/eyesy.yres)
    circmod = xr * 0.020 #((25*eyesy.xres)/eyesy.xres)
    offsetmod = xr * 0.023 #((30*eyesy.xres)/eyesy.xres)
    ypos.step = int(eyesy.knob3*5*ystepmod) #LFO rate of change
    circles = int(eyesy.knob1*circmod)+1
    space = (xr/circles)
    offset = int((eyesy.knob2*7)*offsetmod)
    ypos.frame(step)  # stereopsis: stock's speed frame to frame
    y = int(ypos.update())
    
    for i in range (0, circles) :
        auDio = abs(eyesy.audio_in[i+3] / 100)
        r = int(auDio + offset)+4
        ax = int((i*space)+(space/2))
        pygame.gfxdraw.filled_circle(screen, ax, y, r, color)
