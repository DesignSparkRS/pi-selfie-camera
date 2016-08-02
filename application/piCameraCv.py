#!/usr/bin/python

## Raspberry Pi and OpenCV powered auto-selfie camera
##
## Copyright 2016, Electrocomponents Plc

import os,signal,sys
import cv2
import thread
import picamera
import picamera.array
import time
import pygame.mixer
import RPi.GPIO as GPIO
import numpy as np

from PIL import Image

global hasFace
hasFace = False

global attention
attention = 0.0

global mode 
mode = 0
#  0 = looking
#  1 = capture sequence
#  2 = wait for a bit

global inhibit
inhibit = False

def range(v,min_in,max_in,min_out,max_out):
    if v > max_in:
        return max_out
    elif v < min_in:
        return min_out
    else:      
        return (v - min_in) * (max_out - min_out) / (max_in - min_in) + min_out
    
def attnSmooth():
    global hasFace
    global attention
    global mode
    global inhibit
    
    while True:
        if inhibit:
                #flashing when found
            	milli  = time.time() * 1000
            	flashTime = 1500
            	mod = milli%(flashTime*2)
            	if (mod<flashTime):
                    l3.ChangeDutyCycle(range(mod,0,flashTime,15,30))
                    l2.ChangeDutyCycle(range(mod,0,flashTime,15,30))
                    l1.ChangeDutyCycle(range(mod,0,flashTime,15,30))
                
                else:
               		l3.ChangeDutyCycle(range(mod,flashTime,flashTime*2,30,15))
               		l2.ChangeDutyCycle(range(mod,flashTime,flashTime*2,30,15))
               		l1.ChangeDutyCycle(range(mod,flashTime,flashTime*2,30,15))    
            
        elif mode == 0:
            if hasFace:
                attention += 0.25
                if attention > 100:
                    attention = 100

            else:
                attention -= 0.5          
                if attention < 0:
                    attention = 0
                
            l3.ChangeDutyCycle(range(attention,0.0,33.33,0,100))
            l2.ChangeDutyCycle(range(attention,30,66.66,0,100))
            l1.ChangeDutyCycle(range(attention,63,99.99,0,100))
        
        elif mode == 1:
                #flashing when found
            	milli  = time.time() * 1000
            	flashTime = 100
            	mod = milli%(flashTime*2)
            	if (mod<flashTime):
                    l3.ChangeDutyCycle(0)
                    l2.ChangeDutyCycle(0)
                    l1.ChangeDutyCycle(0)
                
                else:
               		l3.ChangeDutyCycle(100)
               		l2.ChangeDutyCycle(100)
               		l1.ChangeDutyCycle(100)
               		
        elif mode == 2:
                #flashing when found
            	milli  = time.time() * 1000
            	flashTime = 50
            	mod = milli%(flashTime*2)
            	if (mod<flashTime):
                    l3.ChangeDutyCycle(0)
                    l2.ChangeDutyCycle(0)
                    l1.ChangeDutyCycle(0)
                
                else:
               		l3.ChangeDutyCycle(100)
               		l2.ChangeDutyCycle(0)
               		l1.ChangeDutyCycle(0)		
        
        time.sleep(0.005)
             

def on_exit(sig,func=None):
    print "exit handler triggered"
    camera.remove_overlay(o)
    camera.stop_preview();
    GPIO.cleanup()
    #cv2.destroyAllWindows()
    sys.exit(1)

if __name__ == "__main__":
    #set exit handler
    signal.signal(signal.SIGTERM, on_exit)

    CAMERA_WIDTH = 800
    CAMERA_HEIGHT = 480

    WRK_WIDTH = 400
    WRK_HEIGHT = 240 
    
    picTimeStamp = 0.0
    WAIT_PERIOD = 5.0
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD);
    GPIO.setup(23,GPIO.OUT);
    GPIO.setup(21,GPIO.OUT);
    GPIO.setup(19,GPIO.OUT);
    
    l1 = GPIO.PWM(19,100)
    l2 = GPIO.PWM(21,100)
    l3 = GPIO.PWM(23,100)
    
    l1.start(0)
    l2.start(0)
    l3.start(0)
        
    GPIO.setup(15,GPIO.IN);
        
    pygame.mixer.init(44100, -16, 1, 1024)
        
    sayCheese = pygame.mixer.Sound("/opt/camera/audio/ChelseaSaysCheese.wav")
    sayCheese.set_volume(1)
    
    shutter = pygame.mixer.Sound("/opt/camera/audio/shutter.wav")
    shutter.set_volume(1)
    
    sndChannel1 = pygame.mixer.Channel(1)
    sndChannel2 = pygame.mixer.Channel(2)
    
    #overlay = cv2.imread('SilentFrame.png',cv2.IMREAD_UNCHANGED)
    overlay = cv2.imread('/opt/camera/images/previewFrame.png')
    overlaySave = Image.open('/opt/camera/images/saveFrame.png')
    
    oc = np.copy(overlay)
    
    win_title = "face detection"

    #face_cascade = cv2.CascadeClassifier('/opt/camera/cascades/haarcascade_frontalface_alt.xml')
    face_cascade = cv2.CascadeClassifier('/opt/camera/cascades/lbpcascade_frontalface.xml')

    #cv2.namedWindow(win_title,cv2.WINDOW_AUTOSIZE)
    
    try:
        thread.start_new_thread( attnSmooth, () )
    except:
        e = sys.exc_info()[0]
        print e
        print "Error: unable to start thread"
    
    
    with picamera.PiCamera() as camera :
        
        
        camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
        camera.framerate = 30
        camera.vflip = True
        
        camera.start_preview();
        
        o = camera.add_overlay(np.getbuffer(oc), layer=3, alpha=75)
        
        with picamera.array.PiRGBArray(camera) as stream :
            while True:
                camera.capture(stream,format="bgr", use_video_port = True)
                image = stream.array
                im_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                
                im_g_s = cv2.resize(im_gray, (WRK_WIDTH,WRK_HEIGHT), interpolation = cv2.INTER_AREA)
                
                faces = face_cascade.detectMultiScale(im_g_s, 1.3, 5)
                                         
                rX = float(CAMERA_WIDTH)/WRK_WIDTH
                rY = float(CAMERA_HEIGHT)/WRK_HEIGHT
                
                oc = np.copy(overlay)
                
                if mode == 2 and time.time()>picTimeStamp+WAIT_PERIOD:
                    mode = 0
                       
                inhibit = not GPIO.input(15)       
                
                if GPIO.input(15) and len(faces) > 0:
#                     
                    hasFace = True
                    
                    for (x,y,w,h) in faces:
                         sx = int(x*rX)
                         sy = int(y*rY)
                         sw = int(w*rX)
                         sh = int(h*rY)
                         cv2.rectangle(oc,(sx,sy),(sx+sw,sy+sh),(255,255,0),5)   

                else:
                    hasFace = False
                    
                if attention == 100:
                    mode = 1
                    sndChannel1.play(sayCheese)  
                    
                    time.sleep(1) 
                    
                    cv2_im = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
                    pil_im = Image.fromarray(cv2_im)
                    pil_im.paste(overlaySave,(0,0), overlaySave)
                    filename = time.strftime("/var/camera/images/%Y-%m-%d-%H-%M-%S.png")
                    pil_im.save(filename)
                    
                    sndChannel2.play(shutter)
                    
#                    camera.capture(filename)
#                    cv2.imwrite(filename ,image) 
                    
                    mode = 2
                    picTimeStamp = time.time()
                    attention = 0
                    l3.ChangeDutyCycle(0)
                    l2.ChangeDutyCycle(0)
                    l1.ChangeDutyCycle(0)
                                      
                o.update(np.getbuffer(oc))
                at = time.time() - picTimeStamp
                o.alpha = int(range(at, 0, .5, 128, 75))
                
                stream.seek(0)
                stream.truncate()
                                         
                if cv2.waitKey(2) & 0xff == ord('q'):
                    break
                                         
                
            camera.remove_overlay(o)
            camera.stop_preview() 
            GPIO.cleanup()                            
            #cv2.destroyAllWindows()    
