#!/usr/bin/env python

import rospy;
from sensor_msgs.msg import Image;
from sensor_msgs.msg import CameraInfo;
#from __future__ import absolute_import, print_function, division
from pymba import *
import numpy as np
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError;

#cv2.namedWindow("test")

with Vimba() as vimba:
    #camera info
    camera_info_msg = CameraInfo();
    camera_info_msg.header.frame_id = "avt_manta";
    camera_info_msg.width = 1624;
    camera_info_msg.height = 1234;
    camera_info_msg.K = [1792.707269, 0.0, 821.895887, 0.0, 1790.871098, 624.859714, 0.0, 0.0, 1.0];
    camera_info_msg.D = [-0.225015, 0.358593, -0.005422, 0.009070, 0.0];
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    camera_info_msg.P = [1736.461670, 0.0, 834.094334, 0.0, 0.0, 1751.635254, 621.266132, 0.0, 0.0, 0.0, 1.0, 0.0];
    camera_info_msg.distortion_model = "narrow_stereo";
    ##########

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
	c0.ExposureTimeAbs = 70000;
	print(c0.GevSCPSPacketSize)
        print(c0.StreamBytesPerSecond)
 	print(c0.AcquisitionFrameCount);
	print(c0.AcquisitionFrameRateAbs);
	print(c0.AcquisitionFrameRateLimit);
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

    rospy.init_node('pymba_publisher');
    image_pub  	= rospy.Publisher("/avt_manta/image_raw", Image, queue_size=1);
    cam_info_pub = rospy.Publisher("/avt_manta/camera_info", CameraInfo, queue_size=1);
    bridge  	= CvBridge();

    while not rospy.is_shutdown():
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


            #cv2.imshow("test",img)
 	    ##
 	    image_pub.publish(bridge.cv2_to_imgmsg(img, "mono8"));
	    cam_info_pub.publish(camera_info_msg);
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
