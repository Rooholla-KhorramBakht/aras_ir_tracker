import pickle
import cv2
import numpy as np
import yaml

class RadTanPinholeCamera:
    def __init__(self, intrinsics_yaml):
        #Load the camera calibration yaml file obtained using the ROS calibration tool
        with open(intrinsics_yaml, 'r') as f:
            calib = yaml.safe_load(f.read())
        #Extract the important paramters from the calibration file
        # Camera Matrix
        self.K = np.array(calib['camera_matrix']['data']).reshape(calib['camera_matrix']['rows'],calib['camera_matrix']['cols'])
        # RadTan Distorsion Parameters
        self.D = np.array(calib['distortion_coefficients']['data']).reshape(-1, 5)
        # Reprojection Matrix
        self.P = np.array(calib['projection_matrix']['data']).reshape(3, 4)
        # Camera Resolution
        self.size = (calib['image_width'], calib['image_height'])
        
    def undist(self,points):
        lpts_ud=cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D, P=self.P)
        return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(lpts_ud)))
    
    def undistNormal(self,points):
        lpts_ud=cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D, P=None)
        return np.squeeze(cv2.convertPointsToHomogeneous(np.float32(lpts_ud)))
        
class StereoCamera:
    def __init__(self, cam1_yaml, cam2_yaml, P1, P2):
        #Load the camera calibration yaml file obtained using the ROS calibration tool
        self.cam1 = RadTanPinholeCamera(cam1_yaml)
        self.cam2 = RadTanPinholeCamera(cam2_yaml)
        #Reprojection matrices from the extrinsic calibration rutine
        self.P1 = P1
        self.P2 = P2
        
    def triangulate(self, x1, x2, undist = False):
        if undist:
            x1_indist = self.cam1.undist(x1.reshape(-1,1,2))
            x2_indist = self.cam2.undist(x2.reshape(-1,1,2))
        else:
            x1_undist = x1
            x2_undist = x2
        landmarks = cv2.triangulatePoints(self.P1, self.P2,x1_undist[...,0:2].reshape(-1,1,2),
                                                           x2_undist[...,0:2].reshape(-1,1,2))
        return (landmarks/landmarks[-1,:]).T.squeeze()
    
    def reproject(self, landmarks):
        #Reproject the landmark on the first camera
        x1_reprojected = self.P1 @ landmarks.T.reshape(4,-1)
        x1_reprojected /= x1_reprojected[-1,:]
        #Reproject the landmark on the second camera
        x2_reprojected = self.P2 @ landmarks.T.reshape(4,-1)
        x2_reprojected /= x2_reprojected[-1,:]
        return x1_reprojected.T, x2_reprojected.T
    
    def undist(self, x1, x2):
        x1_undist = self.cam1.undist(x1)
        x2_undist = self.cam2.undist(x2)
        return x1_undist, x2_undist
