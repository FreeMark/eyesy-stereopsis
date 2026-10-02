import pygame
import socket
import imp
import subprocess
import re

def draw_knob_slider_480(screen, eyesy, offx, offy, index) :
    # color based on knob seq state 
    if eyesy.knob_seq_state == "playing":
        color = eyesy.GREEN
    elif eyesy.knob_seq_state == "recording" :
        color = eyesy.RED
    else :
        color = eyesy.LGRAY
  
    # use knob1,2 etc, same as mode
    if index == 0: knob = eyesy.knob1
    elif index == 1: knob = eyesy.knob2
    elif index == 2: knob = eyesy.knob3
    elif index == 3: knob = eyesy.knob4
    elif index == 4: knob = eyesy.knob5
    pygame.draw.line(screen, color, [offx, offy], [offx + 10, offy], 1)
    pygame.draw.line(screen, color, [offx, offy], [offx, offy + 24], 1)
    pygame.draw.line(screen, color, [offx + 10, offy], [offx + 10, offy + 24], 1)
    pygame.draw.line(screen, color, [offx, offy + 24], [offx + 10, offy + 24], 1)
    pygame.draw.rect(screen, color, (offx, offy + 24 - int(24*knob), 10, int(24*knob)))

def draw_vu_480(screen, eyesy, offx, offy):
    # L
    color = eyesy.LGRAY
    for i in range(0,15) :
        x = offx + 8 * i
        pygame.draw.line(screen, color, [x, offy], [x + 6, offy], 1)
        pygame.draw.line(screen, color, [x, offy], [x, offy + 7], 1)
        pygame.draw.line(screen, color, [x + 6, offy], [x + 6, offy + 7], 1)
        pygame.draw.line(screen, color, [x, offy + 7], [x + 6, offy + 7], 1)
    color = eyesy.GREEN
    for i in range(0, int(eyesy.audio_peak / 2048)):
        if i > 8 : color = (255,255,0)
        if i == 14 : color = eyesy.RED
        x = offx + 8 * i
        if i < 15 : pygame.draw.rect(screen, color, (x + 1, offy + 1, 5, 6))
    
    # R
    offy += 9
    color = eyesy.LGRAY
    for i in range(0,15) :
        x = offx + 8 * i
        pygame.draw.line(screen, color, [x, offy], [x + 6, offy], 1)
        pygame.draw.line(screen, color, [x, offy], [x, offy + 7], 1)
        pygame.draw.line(screen, color, [x + 6, offy], [x + 6, offy + 7], 1)
        pygame.draw.line(screen, color, [x, offy + 7], [x + 6, offy + 7], 1)
    color = eyesy.GREEN
    for i in range(0, int(eyesy.audio_peak_r / 2048)):
        if i > 8 : color = (255,255,0)
        if i == 14 : color = eyesy.RED
        x = offx + 8 * i
        if i < 15 : pygame.draw.rect(screen, color, (x + 1, offy + 1, 5, 6))


def draw_midi(screen, eyesy, offx, offy):
    for i in range(0, 33):
        pygame.draw.line(screen, eyesy.LGRAY, [(i*6)+offx, offy], [(i*6)+offx, 24+offy], 1)
    for i in range(0, 5):
        pygame.draw.line(screen, eyesy.LGRAY, [offx, (i*6)+offy], [offx + 192, (i*6)+offy], 1)
    for i in range(0,128):
        if (eyesy.midi_notes[i] > 0):
            pygame.draw.rect(screen, eyesy.LGRAY, (offx + 6 * (i % 32), offy + 6 * int(i / 32), 6, 6))
 
def draw_gain_bar(screen, eyesy, offx, offy):
    color = eyesy.LGRAY
    pygame.draw.line(screen, color, [offx, offy], [offx + 118, offy], 1)
    pygame.draw.line(screen, color, [offx, offy+5], [offx + 118, offy+5], 1)
    pygame.draw.line(screen, color, [offx, offy], [offx, offy+5], 1)
    pygame.draw.line(screen, color, [offx+118, offy], [offx+118, offy+5], 1)
    pygame.draw.rect(screen, color, (offx, offy, int(eyesy.config["audio_gain"] * 118), 5))


# loading banner helper
def loading_banner(screen, stuff) :
    screen.fill((0,0,0)) 
        
    font = pygame.font.Font("./font.ttf", 80)
    text = font.render("EYESY", True, (255,255,255))
    textpos = text.get_rect()
    textpos.centerx = screen.get_width() / 2
    textpos.centery = screen.get_height() /2
    screen.blit(text, textpos)

    font = pygame.font.Font("./font.ttf", 16)
    text = font.render(stuff, True, (255,255,255))
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.y = screen.get_height() - 100
    screen.blit(text, text_rect)
    pygame.display.flip()

# FORK: caches. Stock re-computed the palette preview line by line (215 colour lookups + 215 line draws)
# and re-rendered every text label on every frame the OSD was up.
_text_cache = {}
_palette_cache = {"key": None, "surf": None}
# stock's legacy foreground picker (palette slot 0) is random below ~0.04 and above 0.96: those rows stay live


def _text(font, s, aa, fg, bg=None):
    """font.render(s, aa, fg, bg), remembered: the OSD's labels change far less often than every frame"""
    key = (id(font), s, bool(aa), tuple(fg), tuple(bg) if bg is not None else None)
    t = _text_cache.get(key)
    if t is None:
        if len(_text_cache) > 256:
            _text_cache.clear()
        t = font.render(s, aa, fg, bg)
        _text_cache[key] = t
    return t


def _palette_rows(surface, eyesy, xoff, yoff, only_live=False):
    legacy_fg = (not eyesy.palettes_user_defined) and eyesy.fg_palette == 0
    for (w, h, x0, y0, pick, may_be_live) in ((170, 130, 0, 0, eyesy.color_picker_bg_preview, False),
                                              (125, 85, 25, 25, eyesy.color_picker, legacy_fg)):
        for i in range(h):
            v = i / h
            live = may_be_live and (v < 0.05 or v > 0.955)
            if only_live and not live:
                continue
            y = yoff + y0 + i
            pygame.draw.line(surface, pick(v), (xoff + x0, y), (xoff + x0 + w - 1, y))


def draw_color_palette(surface, eyesy):
    key = (eyesy.fg_palette, eyesy.bg_palette, eyesy.palettes_user_defined, id(eyesy.palettes))
    if _palette_cache["key"] != key:
        s = pygame.Surface((170, 130))
        _palette_rows(s, eyesy, 0, 0)
        _palette_cache.update(key=key, surf=s)
    surface.blit(_palette_cache["surf"], (450, 10))
    _palette_rows(surface, eyesy, 450, 10, only_live=True)   # the rows that flicker in stock

def render_overlay_480(screen, eyesy) :

    font = eyesy.font
    
    pygame.draw.rect(screen, (0,0,0), (10, 10, 598, 130))
    #pygame.draw.line(screen, eyesy.LGRAY, [0,480], [720,480], 1)
    #pygame.draw.line(screen, eyesy.LGRAY, [720,480], [720,0], 1)
    
    draw_color_palette(screen, eyesy)

    # first time through, gather some info
    if eyesy.osd_first :
        eyesy.osd_first = False

    # mode
    txt_str = "Mode: (" + str(eyesy.mode_index + 1) +" of "+str(len(eyesy.mode_names)) + ") " + str(eyesy.mode)      
    text = _text(font, txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 30
    screen.blit(text, text_rect)
 
    # usb
    if eyesy.running_from_usb:
        txt_str = "USB"    
    else :
        txt_str = "SD"
    text = _text(font, txt_str, True, eyesy.GREEN, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 404
    text_rect.centery = 30
    screen.blit(text, text_rect)
         
    # scene
    if eyesy.scene_index >= 0 :
        scene_str = "Scene: (" + str(eyesy.scene_index + 1) +" of "+str(len(eyesy.scenes)) + ") " + str(eyesy.scenes[eyesy.scene_index]["name"])
    else:
        scene_str = "Scene: None "
    text = _text(font, scene_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 55
    screen.blit(text, text_rect)   

    # res
    message = f"Screen Size: {eyesy.xres} x {eyesy.yres}"
    text =          _text(font, message, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 80
    screen.blit(text, text_rect)   
 
    # version
    # FORK: stereopsis names itself; right-aligned so a longer version still ends before the palette preview
    message = eyesy.VERSION if not eyesy.VERSION[:1].isdigit() else f"v{eyesy.VERSION}"
    text =          _text(font, message, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.right = 445
    text_rect.centery = 80
    screen.blit(text, text_rect)   
    
    # knobs
    draw_knob_slider_480(screen, eyesy, 20, 105, 0)
    draw_knob_slider_480(screen, eyesy, 33, 105, 1)
    draw_knob_slider_480(screen, eyesy, 46, 105, 2)
    draw_knob_slider_480(screen, eyesy, 59, 105, 3)
    draw_knob_slider_480(screen, eyesy, 73, 105, 4)

    draw_midi(screen,eyesy,89,105)
    draw_gain_bar(screen, eyesy, 286, 105)
    draw_vu_480(screen, eyesy, 286, 113)
       
    # trigger
    pygame.draw.rect(screen, eyesy.LGRAY, (410, 105, 25, 25), 1)
    if eyesy.trig:
        pygame.draw.rect(screen, (255,255,0), (410, 105, 25, 25))

    # errors
    i = 0
    for errorline in eyesy.error.splitlines() :
        errormsg = _text(font, errorline, True, eyesy.LGRAY, eyesy.RED) 
        text_rect.x = 50
        text_rect.y = 150 + (i * 20)
        screen.blit(errormsg, text_rect)
        i += 1


    '''            
    # input level 
    pygame.draw.rect(screen, eyesy.BLACK, (20, 205, 220, 30))
    txt_str = " Input Level:"
    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 220
    screen.blit(text, text_rect)
    draw_vu_480(screen, eyesy, 115, 210)
    
    # Auto Clear   
    if eyesy.auto_clear :
        txt_str = " Persist: No " 
    else :
        txt_str = " Persist: Yes "         
    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 252
    screen.blit(text, text_rect)
    
    # mem
    txt_str = " Memory Used:  "   + str(int(eyesy.memory_used) + 1) + "% "
    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 306
    screen.blit(text, text_rect)
 
    # midi usb dev
#    if (eyesy.usb_midi_present) :
#        txt_str = " USB MIDI:  "   + str(eyesy.usb_midi_name) + " "
#    else :
#        txt_str = " USB MIDI:  None "
#    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
#    text_rect = text.get_rect()
#    text_rect.x = 20
#    text_rect.centery = 334
#    screen.blit(text, text_rect)
 
    # midi ch
#    txt_str = " MIDI CH:  "   + str(eyesy.midi_ch) + " "
#    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
#    text_rect = text.get_rect()
#    text_rect.x = 20
#    text_rect.centery = 362
#    screen.blit(text, text_rect)
         # SSID    
    txt_str = " Network: FIX ME "
    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 418
    screen.blit(text, text_rect)

    # fps
    txt_str = " FPS:  "   + str(int(eyesy.fps)) + " "
    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 10
    text_rect.centery = 10
    screen.blit(text, text_rect)
    
    # version
    txt_str = " v3beta1 "
    text = font.render(txt_str, True, eyesy.LGRAY, eyesy.BLACK)
    text_rect = text.get_rect()
    text_rect.x = 20
    text_rect.centery = 446
    screen.blit(text, text_rect)
    
    # grabs
    #pygame.draw.rect(screen, eyesy.BLACK, (486, 20, 155, 435))
    #text = font.render(" Recent Grabs", True, eyesy.LGRAY, eyesy.BLACK)
    #text_rect = text.get_rect()
    #text_rect.x = 500
    #text_rect.centery = 37
    #screen.blit(text, text_rect)
    #screen.blit(eyesy.tengrabs_thumbs[0], (500, 78 * 0 + 50))
    #screen.blit(eyesy.tengrabs_thumbs[1], (500, 78 * 1 + 50))
    #screen.blit(eyesy.tengrabs_thumbs[2], (500, 78 * 2 + 50))
    #screen.blit(eyesy.tengrabs_thumbs[3], (500, 78 * 3 + 50))
    #screen.blit(eyesy.tengrabs_thumbs[4], (500, 78 * 4 + 50))
    
    # osd, errors
    i = 0
    font = pygame.font.Font("font.ttf", 16)
    for errorline in eyesy.error.splitlines() :
        errormsg = font.render(errorline, True, eyesy.LGRAY, eyesy.RED) 
        text_rect.x = 50
        text_rect.y = 60 + (i * 20)
        screen.blit(errormsg, text_rect)
        i += 1
'''

