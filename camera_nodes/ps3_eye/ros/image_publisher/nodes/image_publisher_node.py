#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
#from camera_calibration_parsers import readCalibration
from subprocess import call
import subprocess
#from image_publisher.srv import set_gain, set_exposure
import threading
import time
from dynamic_reconfigure.server import Server
from image_publisher.cfg import camera_cfgsConfig
from utils import *

def cfgs_callback(config, level):
    set_manual(camera_id,camera_id)
    set_gain(camera_id,config.gain)
    set_exposure(camera_id,config.exposure)
    return config

def handle_set_gain(req):
    set_gain(camera_id,req.gain)
    print('The gain for the camera'+str(camera_id)+' has been set to:'+str(req.gain))
    return True

def handle_set_exposure(req):
    set_exposure(camera_id,req.exposure)
    print('The exposure for the camera'+str(camera_id)+' has been set to:'+str(req.exposure))
    return True

rospy.init_node('general_image_publisher')
print("Node Initialized")

#Load the node paramters
param_manager=paramManager()
params=param_manager.load_params()

#Image Publisher Subsystem
# calib_load=(True if params['~path_to_calib_file'] is not None else False)   
ipm = imgPubliherMachine(params)

#Image Publisher Thread
if params['~publisher_mode']=='camera':
    img_loop=cameraLoop(params['~camera_id'],params['~fps'], ipm)
    
elif params['~publisher_mode']=='dir':
    img_loop=directoryLoop(params['~img_dir_path'], params['~fps'], ipm, replay=params['~replay'])

publisher_thread=threading.Thread(target=img_loop.loop)

#Initialize the Service Server
# gain_service=rospy.Service('set_gain',set_gain,handle_set_gain)
# gain_service=rospy.Service('set_exposure',set_exposure,handle_set_exposure)
# server=Server(camera_cfgsConfig,cfgs_callback)

try:
    publisher_thread.start()
    rospy.spin()
    #if the ros goes down, the image capruring thread will have to terminate
    img_loop.kill=True
    publisher_thread.join()
except KeyboardInterrupt:
    print("Shutting down")

