# -*- coding: utf-8 -*-
"""
Created on Mon Jul 07 14:59:03 2014

@author: derricw
"""

from __future__ import absolute_import, print_function, division
from pymba import *
import numpy as np
import cv2
import time

cv2.namedWindow("test")

with Vimba() as vimba:
    system = vimba.getSystem()

    system.runFeatureCommand("GeVDiscoveryAllOnce")
    time.sleep(0.2)

    camera_ids = vimba.getCameraIds()

    for cam_id in camera_ids:
        print("Camera found: ", cam_id)
        
    c0 = vimba.getCamera(camera_ids[0])
    c0.openCamera()

    #cameraFeatureNames = c0.getFeatureNames();
    #for name in cameraFeatureNames:
 #	print(name);
	
    try:
        #gigE camera
        c0.ExposureAuto = "Off";
	c0.ExposureTimeAbs = 60000;
	#print(c0.GevSCPSPacketSize)
        #print(c0.StreamBytesPerSecond)
 	#print(c0.AcquisitionFrameCount);
	#print(c0.AcquisitionFrameRateAbs);
	#print(c0.AcquisitionFrameRateLimit);
 	#c0.ExposureAuto = False;
        #c0.StreamBytesPerSecond = 124000000
 	#c0.GevSCPSPacketSize = 9000;
    except:
        #not a gigE camera
        pass

    #set pixel format
    c0.PixelFormat="Mono8"
    #c0.PixelFormat = "BGR8Packed";
    #c0.ExposureTimeAbs=60000
 
    c0.AcquisitionMode = "Continuous";

    frame = c0.getFrame()
    frame.announceFrame()

    c0.startCapture()

    framecount = 0
    droppedframes = []

    c0.runFeatureCommand("AcquisitionStart");
    frame.queueFrameCapture();


    while 1:
        frame.waitFrameCapture(1000);
	try:
            frame.queueFrameCapture()
            success = True
        except:
            droppedframes.append(framecount)
            success = False
        #c0.runFeatureCommand("AcquisitionStart")
        #c0.runFeatureCommand("AcquisitionStop")
        #frame.waitFrameCapture(1000)
        frame_data = frame.getBufferByteData()
        if success:
            img = np.ndarray(buffer=frame_data,
                             dtype=np.uint8,
                             shape=(frame.height,frame.width,frame.pixel_bytes))

 	    #img = cv2.resize(img, 
 	#		dsize=(int(frame.width/3), int(frame.height/3)),
 	#		interpolation=cv2.INTER_CUBIC);


            cv2.imshow("test",img)
        framecount+=1
	#print(img);
 	#print(framecount);
        k = cv2.waitKey(1)
        if k == 0x1b:
            cv2.destroyAllWindows()
            print("Frames displayed: %i"%framecount)
            print("Frames dropped: %s"%droppedframes)
            break

    c0.runFeatureCommand("AcquisitionStop");
    c0.endCapture()
    c0.revokeAllFrames()

    c0.closeCamera()
