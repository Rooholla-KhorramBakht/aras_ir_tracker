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
from camera_calibration_parsers import readCalibration
from subprocess import call
import subprocess
import threading
import time

#ROS parameter manager
class paramManager():
    def __init__(self):
        self.params=\
        {   '~usb_port':'0-7',
            '~camera_id':0,
            '~fps':10,
            '~path_to_calib_file':None,
            '~img_transforms':'to_gray',
            '~publisher_mode':'dir',
            '~replay':True,
            '~img_dir_path':None,
        }
    def load_params(self):
        for key in self.params.keys():
            if rospy.has_param(key):
                self.params[key]=rospy.get_param(key)
        print(self.params)
        return self.params

#Camera Config Utils
def set_manual(camera_id):
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),'-c', 'auto_exposure=1'],stdout=subprocess.PIPE)
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),'-c', 'gain_automatic=1'],stdout=subprocess.PIPE)

def set_gain(camera_id,val):
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),\
                      '-c', 'gain='+str(val)],stdout=subprocess.PIPE)
    print('The gain for the camera'+str(camera_id)+' has been set to:'+str(val))
    return True

def set_exposure(camera_id,val):
    subprocess.Popen(['v4l2-ctl', '-d', '/dev/video'+str(camera_id),\
                      '-c', 'exposure='+str(val)],stdout=subprocess.PIPE)
    print('The exposure for the camera'+str(camera_id)+' has been set to:'+str(val))

def usb_ports2id(port_list):
    if isinstance(port_list,list)==False:
        port_list=[port_list]
        print(port_list)
    #Get the info for the video devices connected to the system
    tst=subprocess.Popen(['v4l2-ctl', '--list-devices'],stdout=subprocess.PIPE)
    comInfo = tst.stdout.read().decode("utf-8") 
    info = comInfo.split('USB Camera')[1:]
    #Find out what /dev/videox corresponds to the given port name in the parameters
    found={}
    for lst in info:
        for idx in port_list:
            if lst.find(idx) is not -1:
                found[idx]=int(lst[lst.find('video')+5])
    return found

#Image Publisher Utils
class imgPubliherMachine():
    def __init__(self,params, img_topic="camera/image_raw",camera_info_topic="camera/cameraInfo"):
        self.params=params
        self.transforms=params['~img_transforms'].split(',')
        self.pub = rospy.Publisher(camera_info_topic,CameraInfo,queue_size=1)
        self.pub_Image = rospy.Publisher(img_topic,Image,queue_size=1)
        self.image=Image()
        self.bridge = CvBridge()
        self.encoding = "8UC3"
        calib_load=(True if params['~path_to_calib_file'] is not None else False)   
        if calib_load:
            camera_name, self.camera_info = readCalibration(params['~path_to_calib_file'])
        else:
            self.camera_info=CameraInfo()
        self.tranform_fncs={'to_gray':self.to_gray}
    
    def to_gray(self,img):
        self.encoding='mono8'
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
    def apply_transforms(self,img):
        for transform in self.transforms:
            assert transform in self.tranform_fncs, transform+" transformation is not implemented"
            img=self.tranform_fncs[transform](img)
        return img
        
    def img_publish(self,img):
        #Are there any transformations that I needs to be applied?
        if len(self.transforms[0])>0:
            img=self.apply_transforms(img)
        try:
            self.image=self.bridge.cv2_to_imgmsg(img, self.encoding)
            self.image.header.stamp=self.camera_info.header.stamp
        except CvBridgeError as e:
            print(e)
        self.camera_info.header.stamp=rospy.Time.now()
        self.pub.publish(self.camera_info)
        self.pub_Image.publish(self.image)

class cameraLoop():
    def __init__(self,camera_id,fps,ipm):
        self.ipm=ipm
        self.cap=cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FPS,fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.kill=False
    
    def loop(self):
        while self.kill is not True:
            ret,Frame = self.cap.read()
            if ret:
                self.ipm.img_publish(Frame)
            else:
                print('capture_error')  
                
class directoryLoop():
    def __init__(self,root,fps=10,ipm=None,replay=False):
        self.root=root
        self.replay=replay
        self.ipm=ipm
        self.img_files=os.listdir(self.root)
        self.img_files.sort()
        self.sleep_time=1.0/fps;
        self.kill=False
    
    def loop(self):
        while self.kill is not True:
            for img_name in self.img_files:
                file_path=os.path.join(self.root,img_name)
                if os.path.isfile(file_path):
                    frame = cv2.imread(file_path)
                    if self.ipm is not None:
                        self.ipm.img_publish(frame)

                if self.kill:
                    break
                time.sleep(self.sleep_time)
            if self.replay==False:
                break;