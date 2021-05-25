from pyclustering.cluster.bsas import bsas
import numpy as np
import cv2
import yaml
import pickle
from pyclustering.cluster.bsas import bsas
from dataclasses import dataclass
from src.ir_tracker_utils import *
from src.ps3_cam_utils import *
from src.telemetry_utils import *
import argparse
import yaml

@dataclass
class paramclass():
    max_markers: int=1
    max_clusters: int=8
    threshold: int= 20
    minThreshold: int= 50;
    maxThreshold: int= 255;
    filterByArea: bool= True
    minArea = 0
    filterByCircularity: bool= True
    minCircularity: float= 0.3
    filterByConvexity: bool= True
    minConvexity: float= 0.7
    filterByInertia: bool= True
    minInertiaRatio: float= 0.1
    blobColor: int= 255

detector_params=paramclass()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description = 'A prgram to track infered markers in stereo images.')
    parser.add_argument('config_file', type=str, help='The path to the config yaml file')
    args = parser.parse_args()

    with open(args.config_file,'r') as f:
        configs = yaml.safe_load(f)

    #Generate a list of capture devices/files
    if configs['reference_by_usb']:
        cam_dict=usb_ports2id(configs['cameras'])
        ids_list=[cam_dict[configs['cameras'][i]] for i in range(len(configs['cameras']))]
        cap_devices=[cv2.VideoCapture(id) for id in ids_list]
        #Configure the cameras
        for n in range(len(configs['cameras'])):
            set_manual(ids_list[n])
            set_gain(ids_list[n],configs['gain'])
            set_exposure(ids_list[n],configs['exposure'])
            cap_devices[n].set(cv2.CAP_PROP_FPS,configs['fps'])
            cap_devices[n].set(cv2.CAP_PROP_BUFFERSIZE,1)
    else:
        cap_devices=[cv2.VideoCapture(id) for id in configs['cameras']]
        if not configs['play_from_video']:
            #Configure the cameras
            for n in range(len(configs['cameras'])):
                set_manual(configs['cameras'][n].split('video')[-1])
                set_gain(configs['cameras'][n].split('video')[-1],configs['gain'])
                set_exposure(configs['cameras'][n].split('video')[-1],configs['exposure'])
                cap_devices[n].set(cv2.CAP_PROP_FPS,configs['fps'])
                cap_devices[n].set(cv2.CAP_PROP_BUFFERSIZE,1)
    # Pixel Processors
    camUndists = [undistrodMarkers(file) for file in configs['calib_files']]
    markerExteractor_inst=markerExteractor(detector_params)
    # Telemetry machine
    telem = udp_telemetry()
    telem.OUT_IP = configs['remote_ip']
    telem.OUT_PORT = configs['remote_port']
    #Program main loop
    old_pos=None
    while all(cap_device.isOpened() for cap_device in cap_devices):
        imgs=[cap_devices[n].read()[1] for n in range(len(cap_devices))]
        if all(v is not None for v in imgs):
            points_list=[markerExteractor_inst.detect(img) for img in imgs]
            processed_points_list=[]
            for img,udist,points in zip(imgs,camUndists,points_list):
                if points is not None:
                    #overlay the detecte markers on the image if display is true
                    if configs['display']:
                        for i in range(points.shape[0]):
                            cv2.circle(img,(int(round(points[i,0])), int(round(points[i,1]))), 2, (255,0,255), -1)
                    processed_points_list.append(udist.process(points)[:,0:2].reshape(-1,2))
                else:
                    processed_points_list.append(np.array([-1,-1]).reshape(1,2))

            if configs['print_markers']:
                for j in range(len(configs['cameras'])):
                    print(f'camera{j}: ' + '\t'.join([f'p{i+1}: {processed_points_list[j][i,0]:.1f},{processed_points_list[j][i,1]:.2f}'\
                          for i in range(processed_points_list[j].shape[0])]))
                print('-------------------------')

            telem.transmit_data(processed_points_list)

            if configs['display']:
                [cv2.imshow(f'camera{i}',imgs[i]) for i in range(len(imgs))]
        else:
            print('Error: Can not capture images')
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    [cap.release() for cap in cap_devices]
