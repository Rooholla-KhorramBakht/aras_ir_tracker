from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from subprocess import call
import subprocess
import threading
import time

#ROS parameter manager
class paramManager():
    def __init__(self):
        self.params=\
        {   '~debug_image':True,
            '~remote_ip':'127.0.0.1',
            '~remote_port':5000,
            '~udp_telemetry': True
        }
    def load_params(self):
        for key in self.params.keys():
            if rospy.has_param(key):
                self.params[key]=rospy.get_param(key)
        print(self.params)
        return self.params
