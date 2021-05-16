#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from ir_marker_extractor.msg import ir_markers
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from pyclustering.cluster.bsas import bsas
from dynamic_reconfigure.server import Server
from ir_marker_extractor.cfg import ir_marker_extractor_cfgsConfig
import numpy as np

def recongigureator_callback(config,level):
    #Image Thresholding Parameters
    node_processor.marker_extractor.high_threshold=config.high_threshold
    node_processor.marker_extractor.low_threshold=config.low_threshold
    #BSAS Parameters
    node_processor.marker_extractor.bsas_enable=config.bsas_enable
    node_processor.marker_extractor.max_clusters=config.max_clusters
    node_processor.marker_extractor.maxWhitePixels=config.max_white_pixels
    node_processor.marker_extractor.threshold=config.clustering_threshold
    #Blub Detection Parameters
    node_processor.marker_extractor.blubParams.minThreshold=config.blub_min_threshold
    node_processor.marker_extractor.blubParams.maxThreshold=config.blub_max_threshold
    node_processor.marker_extractor.blubParams.filterByArea=config.blub_filterByArea
    node_processor.marker_extractor.blubParams.minArea=config.blub_minArea
    node_processor.marker_extractor.blubParams.filterByCircularity=config.blub_filterByCircularity
    node_processor.marker_extractor.blubParams.minCircularity=config.blub_minCircularity
    node_processor.marker_extractor.blubParams.filterByConvexity=config.blub_filterByConvexity
    node_processor.marker_extractor.blubParams.minConvexity=config.blub_minConvexity
    node_processor.marker_extractor.blubParams.filterByInertia=config.blub_filterByInertia
    node_processor.marker_extractor.blubParams.minInertiaRatio=config.blub_minInertiaRatio
    node_processor.marker_extractor.blubParams.blobColor=config.blub_color
    node_processor.marker_extractor.updateParameters()
    return config
class markerExteractor():
    def __init__(self):
        self.bsas_enable=False
        self.maxWhitePixels=300
        self.low_threshold=100
        self.high_threshold=255
        self.max_clusters = 8
        self.threshold = 20
        self.blubParams = cv2.SimpleBlobDetector_Params()
        self.blubParams.minThreshold = 50;
        self.blubParams.maxThreshold = 255;
        self.blubParams.filterByArea = True
        self.blubParams.minArea = 0
        self.blubParams.filterByCircularity = True
        self.blubParams.minCircularity = 0.3
        self.blubParams.filterByConvexity = True
        self.blubParams.minConvexity = 0.7
        self.blubParams.filterByInertia = True
        self.blubParams.minInertiaRatio = 0.1
        self.blubParams.blobColor = 255

        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.blubDetector = cv2.SimpleBlobDetector(self.blubParams)
        else :
            self.blubDetector = cv2.SimpleBlobDetector_create(self.blubParams)
    def updateParameters(self):
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.blubDetector = cv2.SimpleBlobDetector(self.blubParams)
        else :
            self.blubDetector = cv2.SimpleBlobDetector_create(self.blubParams)
    def detect(self,frame):
        self.cms=[]
        self.image_ROIs=[]
        self.keypoints=[]
        if len(frame.shape)==2:
            img_gray=frame
        else:
            img_gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,img_thresh = cv2.threshold(img_gray,
                                       self.low_threshold,self.high_threshold,cv2.THRESH_TOZERO)
        #Find the clusters
        self.nonzro_samples = cv2.findNonZero(img_thresh)
        if self.nonzro_samples is None:
            return -1,None  #No markers in the image
        else:
            self.nonzro_samples=self.nonzro_samples.reshape(-1, 2).astype('float32')
        if self.nonzro_samples.shape[0]<self.maxWhitePixels:
            if self.bsas_enable:
                result=self.process_bsas(img_thresh)
            else:
                result=self.process_no_bsas(img_thresh)
            return result
        else:
            return -2,None
    def process_bsas(self,img_thresh):
            bsas_instance = bsas(self.nonzro_samples, self.max_clusters, self.threshold)
            bsas_instance.process()
            clusters = bsas_instance.get_clusters()
            #Calculate the center of the clusters and the Regions of Interests
            self.ROIs=np.zeros((len(clusters),4))
            for i,cluster in enumerate(clusters):
                current_batch=self.nonzro_samples[cluster]
                self.cms.append(np.sum(current_batch,axis=0)/current_batch.shape[0])
                row_max=np.max(current_batch[:,1],axis=0)+6
                row_min=np.min(current_batch[:,1],axis=0)-6
                col_max=np.max(current_batch[:,0],axis=0)+6
                col_min=np.min(current_batch[:,0],axis=0)-6
                self.ROIs[i,:]=[row_min,row_max,col_min,col_max]
            for roi in self.ROIs.astype('int32'):
                self.image_ROIs.append(img_thresh.copy()[roi[0]:roi[1],roi[2]:roi[3]])
            #Return The Results
            marker_points=[]
            for i,roi in enumerate(self.image_ROIs):
                keys_in_roi=self.blubDetector.detect(roi)
                for key in keys_in_roi:
                    #Calculate the global coordinate of marker points.
                    #The points are returned in (X(Col),Y(Row)) coordinate.
                    marker_points.append([key.pt[0]+self.ROIs.astype('float32')\
                    [i,2],key.pt[1]+self.ROIs.astype('float32')[i,0]])
            return 0,np.array(marker_points)
    def process_no_bsas(self,img_thresh):
        blubs=self.blubDetector.detect(img_thresh)
        marker_points=[]
        for key in blubs:
            marker_points.append([key.pt[0] ,key.pt[1]])
        return 0,np.array(marker_points)
        
class process():
    def __init__(self,debug_image):
        self.debug_image=debug_image
        self.marker_extractor=markerExteractor()
        self.bridge=CvBridge()
    def process(self,image):
        cv_image=self.bridge.imgmsg_to_cv2(image,desired_encoding='mono8')
        #print(cv_image.shape)
        status,markers=self.marker_extractor.detect(cv_image)
        if self.debug_image:
            img_color=cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
            if markers is not None:
                for marker in markers:
                    cv2.circle(img_color,(int(round(marker[0])), int(round(marker[1]))), 3, 
                                                                           (255,0,255),-1)
            debug_image=self.bridge.cv2_to_imgmsg(img_color)
        else:
            debug_image=None
        return status,markers,debug_image
def image_callback(image):
    markers_msg=ir_markers()
    status,markers,debug_image=node_processor.process(image)
    if debug_image is not None:
        debug_image_pub.publish(debug_image)
    if status==-2: #too many white pixels in the image print
                    #('too many white pixels dtected in the image')
        markers_msg.status=status
        markers_msg.header.stamp=image.header.stamp
        pub.publish(markers_msg)
        return
    if status==-1 or len(markers)==0: #no markers in the image
        #print('no markers in the image')
        markers_msg.status=status
        markers_msg.header.stamp=image.header.stamp
        pub.publish(markers_msg)
        return
    marker_list=[]
    for marker in markers:
        marker_list.append(Point(marker[0],marker[1],0))
    markers_msg.status=status
    markers_msg.header.stamp=image.header.stamp
    markers_msg.markers=marker_list
    #[Point(np.random.randn(),2,3),Point(4,5,6)]#
    pub.publish(markers_msg)
    
if __name__ =='__main__':
    rospy.init_node('ir_marker_extractor_node')
    if rospy.has_param('~debug_image'):
        debug_image=rospy.get_param('~debug_image')
        debug_image_pub=rospy.Publisher('/ir_marker_extractor/debug_image',Image,queue_size=1)
    else:
        debug_image=False
    node_processor=process(debug_image)
    pub=rospy.Publisher('/ir_marker_extractor/ir_markers',ir_markers,queue_size=1)
    rospy.Subscriber('/camera/image_raw',Image,image_callback,queue_size=1)
    srv=Server(ir_marker_extractor_cfgsConfig,recongigureator_callback)
    rospy.spin()
