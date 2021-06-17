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
        print('referenced by usbn')
        cam_dict=usb_ports2id(configs['cameras'])
        ids_list=[cam_dict[configs['cameras'][i]] for i in range(len(configs['cameras']))]
        cap_devices=[cv2.VideoCapture(id) for id in ids_list]
        #Configure the cameras
        for n in range(len(configs['cameras'])):
            print(cap_devices[0].set(cv2.CAP_PROP_FPS, configs['fps']))
            cap_devices[n].set(cv2.CAP_PROP_BUFFERSIZE,1)
            set_manual(ids_list[n])
            set_gain(ids_list[n],configs['gain'])
            set_exposure(ids_list[n],configs['exposure'])
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
        old_time=time.time()
        imgs=[cap_devices[n].read()[1] for n in range(len(cap_devices))]
        print(1/(time.time()-old_time))
        if all(v is not None for v in imgs):
            if configs['display']:
                [cv2.imshow(f'camera{i}',imgs[i]) for i in range(len(imgs))]
        else:
            print('Error: Can not capture images')
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    [cap.release() for cap in cap_devices]
