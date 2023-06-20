#!/usr/bin/env python3
import cv2
import gi
import numpy as np
import time
import threading
import socket
import serial
import keyboard
import tkinter as tk
from pymavlink import mavutil
import pygame

gi.require_version('Gst', '1.0')
from gi.repository import Gst

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
ser.isOpen()

cv2.namedWindow('rov', cv2.WINDOW_NORMAL)   
cv2.setWindowProperty('rov', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

fr1 = np.zeros((720,1280,3), np.uint8)  
fr2 = np.zeros((720,1280,3), np.uint8) 
fr3 = np.zeros((720,1280,3), np.uint8) 
fr4 = np.zeros((720,1280,3), np.uint8) 
fr5 = np.zeros((1080,1920,3), np.uint8) 

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
s.connect(("192.168.2.2", 45678))   

rr = gg = bb = l1 = l2 = 0
full = False
wr_en1 = wr_en2 = False
las1 = las2 = False
rec1 = rec2 = False
full1 = full2 = full3  =full4= False
sw0 = False
last88 = 0
last89 = 0
sw10 = sw11 = False
b1 = b2 = b3 = b4 = b5 = 0

txt1 = " "
txt2 = " "
bl = False
osd = True
swf2 = swf1 = False
lastf1 = lastf2 = 0
qu = False

mArm = '0'
mGain = '0'
mTurns = '0'
mYaw = '0'
mDepth = '0'
mTemp = '0'
mMode = '0'
pos = ' ' 
dd = ' '

keyboard.add_hotkey('q', lambda: qu_())
#keyboard.add_hotkey('o', lambda: osd_())

#keyboard.add_hotkey('1+p', lambda: pic_1())
#keyboard.add_hotkey('2+p', lambda: pic_2())

keyboard.add_hotkey('1+l', lambda: las_1())
keyboard.add_hotkey('2+l', lambda: las_2())
keyboard.add_hotkey('1+r', lambda: cam_1())  
keyboard.add_hotkey('2+r', lambda: cam_2()) 
keyboard.add_hotkey('1+c', lambda: rec_1()) 
keyboard.add_hotkey('2+c', lambda: rec_2()) 

keyboard.add_hotkey('1+t', lambda: txt_1()) 
keyboard.add_hotkey('2+t', lambda: txt_2())

keyboard.add_hotkey('1+f', lambda: full_1()) 
keyboard.add_hotkey('2+f', lambda: full_2()) 
keyboard.add_hotkey('3+f', lambda: full_3()) 
keyboard.add_hotkey('4+f', lambda: full_4()) 
keyboard.add_hotkey('w', lambda: win_0()) 

#keyboard.add_hotkey('1+l', lambda: las_1())
#keyboard.add_hotkey('2+l', lambda: las_2())
#-------------------------------------------------
def qu_() :
    global qu
    qu = True
#------------------------------------------------- 
def txt_1() :
    global bl
    bl = True
    app = text1()
    app.title("Cam1 Text:") 
    app.mainloop()  
#------------------------------------------------- 

def txt_2() :
    global bl
    bl = True
    app = text2()
    app.title("Cam2 Text:") 
    app.mainloop() 

#------------------------------------------------- 
def cam_2() :
    global sw0,wr_en2
    if sw0==False :
        sw0=True
        wr_en2 = not wr_en2

#------------------------------------------------- 
def cam_1() :
    global sw0,wr_en1
    if sw0==False :
        sw0=True
        wr_en1 = not wr_en1
#------------------------------------------------- 
def las_1() :
    global sw0,las1
    if sw0==False :
        sw0=True
        las1 = not las1
#------------------------------------------------- 
def las_2() :
    global sw0,las2
    if sw0==False :
        sw0=True
        las2 = not las2
#------------------------------------------------- 
def rec_1()  :
    global sw0, rec1, last88
    if sw0 == False:
        sw0 = True
        last88 = time.time()
        rec1 = True
#------------------------------------------------- 
def rec_2()  :
    global sw0, rec2, last89
    if sw0 == False:
        sw0 = True
        last89 = time.time()
        rec2 = True
#------------------------------------------------- 
def full_1()  :
    global sw0, full1, full2, full3,full4
    if sw0 == False:
        sw0 = True
        full1 = True
        full2 = False
        full3 = False
        full4 = False
#------------------------------------------------- 
def full_2()  :
    global sw0, full1, full2, full3,full4
    if sw0 == False:
        sw0 = True
        full1 = False
        full3 = False
        full2 = True
        full4 = False
#------------------------------------------------- 
def full_3()  :
    global sw0, full1, full2, full3,full4
    if sw0 == False:
        sw0 = True
        full1 = False
        full2 = False
        full3 = True
        full4 = False
#------------------------------------------------- 
def full_4()  :
    global sw0, full1, full2, full3,full4
    if sw0 == False:
        sw0 = True
        full1 = False
        full2 = False
        full4 = True
        full3 = False
#------------------------------------------------- 
def win_0()  :
    global sw0, full1, full2, full3,full4,bl
    if bl== True : return   
    if sw0 == False:
        sw0 = True
        full1 = False
        full2 = False
        full3 = False
        full4 = False
#------------------------------------------------- 
def sw() :
    global sw0
    sw0 = False
#------------------------------------------------- 
def light() : 
    global l1,l2,rr,bb,gg
    global b1,b2,b3,b4,b5
    global las1, las2
    global rec1, rec2
    global sw10, sw11
    global wr_en1, wr_en2
    rr = 0
    gg = 0
    bb = 0

    while True:
        
        b = ser.read(14)
        
        cr = b[2]+b[3]+b[4]+b[5]+b[6]+b[7]+b[8]+b[9]+b[10]+b[11]
        if (cr&0xff != b[12]) and (cr>>8 != b[13]) : print("err")
        
        rr = b[2]
        gg = b[3]
        bb = b[4]
        l1 = b[5]
        l2 = b[6]   
        b1 = b[7]
        b2 = b[8]
        
        aa = bytearray.fromhex("de ad 00 00 00 00 00 00 00 00 00 00 00 00") 
        
        aa[2] = rr
        aa[3] = gg
        aa[4] = bb
        aa[5] = l1
        aa[6] = l2
        aa[7] = b3
        aa[8] = b4
        aa[9] = b5
        aa[10] = b1
        aa[11] = b2
        
        cr = aa[2]+aa[3]+aa[4]+aa[5]+aa[6]+aa[7]+aa[8]+aa[9]+aa[10]+aa[11]
        
        aa[12] = cr&0xff
        aa[13] = cr>>8

        try:
            s.sendall(aa)
        except:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("192.168.2.2",45678))
            s.sendall(aa)
            time.sleep(1)
            print ("recon")
            

        keyboard.on_release(lambda e: sw())   
         
        time.sleep(0.05) 
#------------------------------------------------
def keyb():
    while True:
        time.sleep(0.05)
        if keyboard.is_pressed('1'):
            if keyboard.is_pressed('l') :
                las1= True
            if keyboard.is_pressed('c') :   
                r1 = True
        if keyboard.is_pressed('f') :
            full = True
#------------------------------------------------
def show():
    while True :
        sw1=0
        global fr1, fr2, fr3 ,fr4 ,fr5
        global l1, l2
        global las1, las2
        global rec1, rec2
        global wr_en1, wr_en2
        global swf1,swf2,lastf1,lastf2
        global qu
        global full1,full2,full3
        global mArm, mGain, mTurns, mYaw, mTemp, mMode
        global pos,dd,rr,bb,gg

        start = time.time()
        
        fr5 = np.zeros((1080,1920,3), np.uint8) 
        if full1 == False and full2 == False and full3 == False:

            frr1 = cv2.resize(fr1,(960,540)) 
            frr2 = cv2.resize(fr2,(960,540)) 
            frr3 = cv2.resize(fr3,(960,540))    
            frr4 = cv2.resize(fr4,(960,540)) 
                     
            fr5[1080-540:1080, 960:960+960] = frr2[:]  
            fr5[1080-540:1080, :960] = frr1[:] 
            
            fr5[1080-540-540:1080-540, 960:960+960] = frr3[:]  
            
            fr5[:540, :960] = frr4[:] 
 
            cv2.rectangle(fr5,(1,0),(960,540),(0,255,0),1)
            cv2.rectangle(fr5,(1,540),(960,1080-1),(0,255,0),1)
            cv2.rectangle(fr5,(960,540),(960+960-1,1080-1),(0,255,0),1)
            cv2.rectangle(fr5,(960,0),(960+960-1,1080-1),(0,255,0),1)
        
        cam_no  =" "
        #--------------------------    
        if full1 == True :
            fr5 = cv2.resize(fr1,(1920,1080)) 
            cam_no = "1"
        #--------------------------     
        if full2 == True :
            fr5 = cv2.resize(fr2,(1920,1080)) 
            cam_no = "2"
        #--------------------------     
        if full3 == True :
            fr5 = cv2.resize(fr3,(1920,1080))  
            cam_no = "3"   
        #--------------------------     
        if full4 == True :
            fr5 = cv2.resize(fr4,(1920,1080)) 
            cam_no = "4" 
            
        if full1==True or full2==True or full3==True or full4==True:
            
           
            cv2.putText(fr5,"Camera: "+ cam_no, (1320,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 1, cv2.LINE_AA)
                        
            cv2.putText(fr5,"Temp "+ str(mTemp)+'C', (20,35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            cv2.putText(fr5, 'Depth '+str(dd)+'m', (20,85), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            cv2.putText(fr5, 'Turns '+str(mTurns), (20,135), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            
            cv2.putText(fr5, str(mArm), (1770,1000), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)        
            cv2.putText(fr5, str(mMode), (1770,1050), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            
            cv2.putText(fr5, time.asctime(), (1600,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 1, cv2.LINE_AA)
            cv2.putText(fr5, str(mYaw)+' '+pos, (850,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
                               
            cv2.putText(fr5, 'Red: '+ str(rr), (20,960), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
            cv2.putText(fr5, 'Blue: '+ str(bb), (20,910), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
            cv2.putText(fr5, 'Green: '+ str(gg), (20,860), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)            
                        
            cv2.putText(fr5, 'Gain '+mGain+' %', (20,1010), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)  
          #  cv2.putText(fr5, 'C.Tilt '+mTilt+' %', (20,1060), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA) 
            
            cv2.putText(fr5, ' ', (500,800), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA) 
            
           # if sw4 : cv2.putText(fr5, 'Frame saved', (900,1000), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA) 
             
           # if sw6 : cv2.putText(fr5, 'Pressure calibrated', (850,1000), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2, cv2.LINE_AA) 
                  
          #  if writeEn : cv2.putText(frame, 'rec', (1870,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv2.LINE_AA) 
            
           # cv2.putText(fr5, proName, (700,1060), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA) 
                    
        cv2.imshow('rov', fr5)  
        key = cv2.waitKey(1)&0xFF
        if qu == True:
           
            if bl==False : 
                
                wr_en1 = False
                wr_en2 = False
                time.sleep(0.1)  
                exit()
        while start+0.02 > time.time(): time.sleep(0.001)
#------------------------------------------------
def receive():

   
    cap_receive = cv2.VideoCapture('udpsrc port=5615 caps = "application/x-rtp, media=(string)video,  encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

    if not cap_receive.isOpened():
        print('VideoCapture not opened')
        exit(0)

    global fr1
    while True:        
        ret,fr1 = cap_receive.read() 
#------------------------------------------------
def receive2():

    global fr2
    
    cap_receive = cv2.VideoCapture('udpsrc port=5605 caps = "application/x-rtp, media=(string)video,  encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
    
    if not cap_receive.isOpened():
        print('VideoCapture not opened')
        exit(0)

    while True:        
        ret,fr2 = cap_receive.read()      
#------------------------------------------------
def receive3():

    global fr3
    
    cap_receive = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video,  encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
    
    if not cap_receive.isOpened():
        print('VideoCapture not opened')
        exit(0)

    while True:        
        ret,fr3 = cap_receive.read()  
#------------------------------------------------
def receive4():

    global fr4
    
    cap_receive = cv2.VideoCapture('udpsrc port=5610 caps = "application/x-rtp, media=(string)video,  encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
    
    if not cap_receive.isOpened():
        print('VideoCapture not opened')
        exit(0)

    while True:        
        ret,fr4 = cap_receive.read()    
#------------------------------------------------      
def joy() :

    man_switch = False
    
    sw0 = sw1 = False
    lastMill = 0
    mill = 0
    global b3 

    while True :
        event = pygame.event.get() 
        
        i1 = 0
        b3 = 0
        button = 0
        hat = joystick.get_hat(0) # ruka
        if hat ==(0,0) and joystick.get_button(1)==False and joystick.get_button(2)==False and joystick.get_button(3)==False and joystick.get_button(4)==False and joystick.get_button(5)==False and joystick.get_button(6)==False and joystick.get_button(7)==False and joystick.get_button(8)==False and joystick.get_button(9)==False and joystick.get_button(10)==False and joystick.get_button(11)==False and joystick.get_button(12)==False:
            b3|=0
        
        for x in hat:
            if i1 :
                i1 = 0
                if x == -1 : # down

                   # print("d")
                     #button |= 1024
                    #else  : 
                     if man_switch == True : b3 |= 2
                     if man_switch == False: b3 |= 28
                    
                 
                   
            
                if x == 1 : # up
                     #button |= 2048
                   # else :
                    if man_switch == True : b3 |= 1
                    if man_switch == False: b3 |= 24
                  
                   # print("u")
                    
            else :
                i1 = 1
                if x == -1 : # l
                    #print("l")
                    if man_switch == True : b3 |= 8
                    if man_switch == False: b3 |= 17
            
                if x == 1 : # r

                    #print("r")
                    if man_switch == True : b3 |= 4
                    if man_switch == False: b3 |= 18
        

        # if no one button is clicked b3 =hat 0 (do nothing)
        
        
        if joystick.get_button(4) : b3|=64 # tilt1
      
        if joystick.get_button(6) : b3|=128
        
        
        if joystick.get_button(5) : b3|=16 # tilt2
     
        if joystick.get_button(7) : b3|=32
       
        
        if joystick.get_button(3) :  b3|=2 #  manual
        if joystick.get_button(11) :  b3|=256 #  d.hold
        
        if joystick.get_button(12) :  b3|=512  #  stab
        
        
        if joystick.get_button(2) : man_switch = not man_switch  #  smena manipulatora

        if joystick.get_button(10) : b3|=1463
        
        if joystick.get_button(1) :  b2|= 1 #laser

        # if(b2==0){
        # 
        # }

        if joystick.get_button(1) == False: b2|= 0#if button not pushed b2= 0
        

       

        lx = joystick.get_axis(0)
        ly = joystick.get_axis(1)
        rx = joystick.get_axis(3)
        ry = joystick.get_axis(4)
         
        lx *= 1000        
        rx *= 1000
        ry *= 1000        
        ly = (ly+1.0)*500    
    
        mill = time.time() 
        if lastMill+0.1 <= mill :
            lastMill = mill
            if joystick.get_button(9) :
                if not sw2 :
                    sw2 = 1
                    if not sw1 :
                        sw1 = 1
                        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
                        
                    else :
                        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
                        
                        sw1=0

            else :
                sw2 = 0 

        master.mav.manual_control_send(
        master.
        target_system,
        ~int(ry),
        int(lx),
        1000- int(ly), # 500 means neutral throttle
        int(rx),
        b3) 
        
        print(b3)
        time.sleep(0.05)    
#------------------------------------------------      
def mav() :
    global mArm, mGain, mTurns, mYaw, mTemp, mMode, mDepth
    global pos,dd

    while True: 
        msg = master.recv_match()

        if  msg :
            
            if msg.get_type() == 'NAMED_VALUE_FLOAT' :
                index = str(msg).find('PilotGain')
                if index >0 :
                    str2 = str(msg)[index+18:]
                    str2=str2[:-1]
                    val = float(str2)
                    val = 100*val  
                    val=round(val)
                    mGain=str(val)  
                    
                index = str(msg).find('TetherTrn')
                if index >0 :
                    str2 = str(msg)[index+18:]
                    str2=str2[:-1]
                    val = float(str2)
                     #val = 100*val  
                    val=round(val)
                    mTurns=str(val)                             
                   
            if msg.get_type() == 'ATTITUDE':
                mYaw = msg.yaw * 57.2958
                if mYaw < 0 : mYaw = 360 + mYaw
                
                mYaw = round(mYaw,1)   
                            
                #mRoll = msg.roll
                #mPitch = msg.pitch
                if mYaw >= 337.6 or mYaw <= 22.5 : pos = 'N'
                if mYaw >= 22.6 and mYaw <= 67.5 : pos = 'NE'
                if mYaw >= 67.6 and mYaw <= 112.5 : pos = 'E'
                if mYaw >= 112.6 and mYaw <= 157.5 : pos = 'SE'
                
                if mYaw >= 157.6 and mYaw <= 202.5 : pos = 'S'
                if mYaw >= 202.6 and mYaw <= 247.5 : pos = 'SW'
                if mYaw >= 247.6 and mYaw <= 292.5 : pos = 'W'
                if mYaw >= 292.6 and mYaw <= 337.5 : pos = 'NW'
        
            if msg.get_type() == 'HEARTBEAT':
                if msg.system_status == 3 : mArm = 'Dis.Arm'
                if msg.system_status == 4 : mArm = 'Armed'
                
                if msg.custom_mode == 0 : mMode = 'Stab.'
                if msg.custom_mode == 19 : mMode = 'Manual'
                if msg.custom_mode == 2 : mMode = 'D.Hold'
                
            if msg.get_type() == 'SCALED_PRESSURE2':
                mTemp = float(msg.temperature) / 100
                mTemp = round(mTemp,1)
                
                mDepth = float(msg.press_abs)                   
                
                dd = (mDepth-presCal)/100
                dd = round(dd,1) 
                        
        time.sleep(0.001) 
#------------------------------------------------                     
if __name__ == '__main__':
    x01 = threading.Thread(target=keyb)
    x01.daemon = True
    x01.start() 

    x01 = threading.Thread(target=mav)
    x01.daemon = True
    x01.start() 
    
    x02 = threading.Thread(target=joy)
    x02.daemon = True
    x02.start() 
    
    x0 = threading.Thread(target=light)
    x0.daemon = True
    x0.start() 
  
    x = threading.Thread(target=receive)
    x.daemon = True
    x.start()   
   
    x4 = threading.Thread(target=receive2)
    x4.daemon = True
    x4.start() 

    x5 = threading.Thread(target=receive3)
    x5.daemon = True
    x5.start() 
    
    x5 = threading.Thread(target=receive4)
    x5.daemon = True
    x5.start() 

    show()
    while True : time.sleep(1)

    cv2.destroyAllWindows()
#------------------------------------------------
