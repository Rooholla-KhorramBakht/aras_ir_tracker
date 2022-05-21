import numpy as np
import yaml
import pickle
from src.telemetry_utils import *
from src.cameras import *
from src.bundle_adjustment import *
import argparse
import yaml
import os
import time
import signal
import sys
import threading

# A class to hold the received data and record them into a file when required
class recorderClass():
    def __init__(self, path, ports_list):
        self.path = path
        self.ports_list = ports_list
        self.data ={i:[] for i in ports_list}

    def update(self,port,data):
        self.data[port].append(data)

    def record_to_file(self):
        timetup = time.localtime()
        stamp=time.strftime('%Y-%m-%d-%H:%M:%S', timetup)
        with open(os.path.join(self.path, f'recorded-{stamp}.pckl'), 'wb') as f:
            pickle.dump(self.data,f)
            

# This callback is called by the RX functions of the telemetry class on the reception of new UDP packets from the UDP marker publisher nodes
def callback(reception_stamp, data):
    #Are both cameras online in the data?
    if cam1 in data[1] and cam2 in data[1]:
        markers_in_cam1 = data[1][cam1]
        markers_in_cam2 = data[1][cam2]
        print(markers_in_cam1.shape)
        if markers_in_cam1.shape[0] > configs['max_markers_per_image'] and \
           markers_in_cam2.shape[0]>configs['max_markers_per_image']:
           # is any marker visible in the image? if not, you'll get [-1,-1] with np.sum(x)=-2
            if np.sum(markers_in_cam1)!=-2 and np.sum(markers_in_cam2)!=-2:
                x1 = markers_in_cam1[0,...]
                x2 = markers_in_cam2[0,...]
                #Undistort the pixel location                
                x1_undist, x2_undist = stereoCamera.undist(x1, x2)
                #Geometrical triangulation of the marker locations
                landmarks = stereoCamera.triangulate(x1_undist, x2_undist, undist=False)
                #Reproject the landmark back on the image
                x1_proj, x2_proj = stereoCamera.reproject(landmarks)
                #Print the reprojection error as a measure of the reconstruction quality
                print('Reprojection Error Before BA:')
                print(np.linalg.norm(x1_undist-x1_proj),
                      np.linalg.norm(x2_undist-x2_proj))

                # ba_optimizer.run(x1_undist.reshape(-1,2), x2_undist.reshape(-1,2), landmarks.reshape(-1,3))
                # print(ba_optimizer.L)

    #Print received data if required
    if configs['print_markers']:
        print('---------------------------')
        print(data)
        print('---------------------------')


    # # If required, record the marker locations into the recorder class
    # if configs['record']:
    #     recorder.update(port, [stamp,local_stamp, seq_num, markers_in_frames])

# A function that is called on the reception of Ctrl-C interrupt. It cleans up the system and stores the data
def termination_handling(signum,frame):
    print('\nTerminating')
    #Terminate the data reception threads by setting their running flags to false
    
    rx_telemetry_object.RX_RUNNING = False
    #Store the data recorded into the recorder class into a file
    # if configs['record']:
    #     recorder.record_to_file()
    sys.exit()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description = 'A prgram to capture the UDP markers from multiple marker publisher nodes and aggregate them.')
    parser.add_argument('config_file', type=str, help='The path to the config yaml file')
    args = parser.parse_args()
    # load the configureation paramters from the YAML config file
    with open(args.config_file,'r') as f:
        configs = yaml.safe_load(f)

    with open(configs['ext_params_path'], 'rb') as f:
        ext_params = pickle.load(f)
    #Extract the projection matrices
    P1, P2 = ext_params['P1'], ext_params['P2']
    #Instantiate a stereo camera object
    stereoCamera = StereoCamera(configs['cam1_params_path'], 
                                configs['cam2_params_path'], P1, P2)
    #Instantiate the NLS GTSAM bundle adjuster
    K1 = stereoCamera.cam1.K
    K2 = stereoCamera.cam2.K
    ba_optimizer = stereoBundleAdjustment(K1, K2, ext_params)

    # Extract the ID of the stereo pair 
    cam1_port = configs['cam1_port']
    cam2_port = configs['cam2_port']
    cam1_id = configs['cam1_id']
    cam2_id = configs['cam2_id']
    cam1 = f'{cam1_port}_{cam1_id}'
    cam2 = f'{cam2_port}_{cam2_id}'
    #For each UDP camera node in the system, generate a telemetry object and a specific thread to handle it
    rx_threads = []
    #Each UDP camera node is designated by a unique port number. For each prot, instantiate a telemetry object

    rx_telemetry_object=udp_telemetry()
    rx_telemetry_object.INPUT_IP = configs['local_ip']
    rx_telemetry_object.INPUT_PORT = configs['local_port']
    # To handle the data reception of each telemetry object create a thread and hand over the callback function 
    # that should be called on the reception of each packet
    rx_threads.append(threading.Thread(target=rx_telemetry_object.start_rx, args =(callback,)))
    # Handle Ctrl-C interrupt correctly by calling a function to clean up the system and story the data
    signal.signal(signal.SIGINT, termination_handling)
    # If required, we can record the received data into a file at the end of the node execution
    # if configs['record']:
    #     recorder = recorderClass(configs['path'], configs['camera_ports'])
    # Start the threads 
    for thread in rx_threads:
        thread.start()
