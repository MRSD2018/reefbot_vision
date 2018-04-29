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
    camera_info_msg.width           = rospy.get_param('width') #540;
    camera_info_msg.height          = rospy.get_param('height') #410;
    camera_info_msg.K               = rospy.get_param('K') #[2046.656901, 0.0, 802.943925, 0.0, 2036.041297, 472.235466, 0.0, 0.0, 1.0];
    camera_info_msg.D               = rospy.get_param('D') #[-0.470340, 1.206041, -0.005557, -0.002479, 0.0];
    camera_info_msg.P               = rospy.get_param('P') #[1981.632568, 0.0, 800.827980, 0.0, 0.0, 1976.621704, 467.141610, 0.0, 0.0, 0.0, 1.0, 0.0];
    camera_info_msg.R               = rospy.get_param('R') #[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    #camera_info_msg.P = [1736.461670, 0.0, 834.094334, 0.0, 0.0, 1751.635254, 621.266132, 0.0, 0.0, 0.0, 1.0, 0.0];
    camera_info_msg.distortion_model = rospy.get_param('distortion_model') #"narrow_stereo";
    ##########

    system = vimba.getSystem()

    system.runFeatureCommand("GeVDiscoveryAllOnce")
    time.sleep(0.2)

    camera_ids = vimba.getCameraIds()

    for cam_id in camera_ids:
        print("Camera found: ", cam_id)
        
    c0 = vimba.getCamera(camera_ids[0])
    c0.openCamera()

    if (rospy.get_param('print_features') == 1):
        cameraFeatureNames = c0.getFeatureNames();
        for name in cameraFeatureNames:
            print(name);
    
    try:
        # c0.PixelFormat = "BGR8Packed";
        c0.PixelFormat      = rospy.get_param('pixel_format')
        # c0.ExposureAuto = "Off"
        c0.ExposureAuto     = rospy.get_param('exposure_auto')
        # c0.ExposureTimeAbs=30000 
        c0.ExposureTimeAbs  = rospy.get_param('exposure_time_abs')
        # c0.BalanceWhiteAuto = "Off"
        c0.BalanceWhiteAuto = rospy.get_param('balance_white_auto')
        # c0.BalanceRatioAbs = 1.3    
        c0.BalanceRatioAbs  = rospy.get_param('balance_ratio_abs')
        # c0.Saturation = 2
        c0.Saturation       = rospy.get_param('saturation')
        # c0.BinningHorizontal = 3
        c0.BinningHorizontal= rospy.get_param('binning_horizontal')
        # c0.BinningVertical = 3
        c0.BinningVertical  = rospy.get_param('binning_vertical')
    except:
        print("Problem initializing the camera.")
        pass

    #set pixel format
    #c0.PixelFormat="Mono8"
    
    # c0.PixelFormat = "BGR8Packed";
    # c0.ExposureAuto = "Off"
    # c0.ExposureTimeAbs=30000 
    # c0.BalanceWhiteAuto = "Off"
    # c0.BalanceRatioAbs = 1.3    
    # c0.Saturation = 2
    # c0.BinningHorizontal = 3
    # c0.BinningVertical = 3

    c0.AcquisitionMode = "Continuous";

    frame = c0.getFrame()
    frame.announceFrame()

    c0.startCapture()

    framecount = 0
    droppedframes = []

    c0.runFeatureCommand("AcquisitionStart");
    frame.queueFrameCapture();

    rospy.init_node('pymba_publisher');
    image_pub   = rospy.Publisher("/avt_manta/image_raw", Image, queue_size=1);
    cam_info_pub = rospy.Publisher("/avt_manta/camera_info", CameraInfo, queue_size=1);
    bridge      = CvBridge();

    while not rospy.is_shutdown():
        frame.waitFrameCapture(1000);
        try:
            frame.queueFrameCapture()
            success = True
        except:
            droppedframes.append(framecount)
            success = False
        frame_data = frame.getBufferByteData()
        if success:
            img = np.ndarray(buffer=frame_data,
                             dtype=np.uint8,
                             shape=(frame.height,frame.width,frame.pixel_bytes))

        #img = cv2.resize(img, 
    #       dsize=(int(frame.width/3), int(frame.height/3)),
    #       interpolation=cv2.INTER_CUBIC);


            #cv2.imshow("test",img)
        ##
        #image_pub.publish(bridge.cv2_to_imgmsg(img, "mono8"));
        image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"));
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
