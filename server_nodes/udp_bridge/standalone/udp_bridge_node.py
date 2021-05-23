import numpy as np
import yaml
import pickle
from src.telemetry_utils import *
import argparse
import yaml
import os
import time
import signal
import sys
import threading

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


def callback(stamp, port, markers_in_frames):
    if configs['print_markers']:
        print(port,stamp,[m.shape[0] for m in markers_in_frames])
    if configs['record']:
        recorder.update(port, [stamp, markers_in_frames])

def termination_handling(signum,frame):
    print('\nTerminating')
    for telemetry_object in telemetry_objects:
        telemetry_object.RX_RUNNING = False

    if configs['record']:
        recorder.record_to_file()
    sys.exit()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description = 'A prgram to capture the marker data from multiple publisher nodes over UDP.')
    parser.add_argument('config_file', type=str, help='The path to the config yaml file')
    args = parser.parse_args()

    with open(args.config_file,'r') as f:
        configs = yaml.safe_load(f)

    #Generate a list of telemetry objects
    telemetry_objects=[]
    rx_threads = []
    for port in configs['camera_ports']:
        obj=udp_telemetry()
        obj.INPUT_IP = configs['local_ip']
        obj.INPUT_PORT = port
        telemetry_objects.append(obj)
        rx_threads.append(threading.Thread(target=obj.start_rx, args =(callback,)))

    signal.signal(signal.SIGINT, termination_handling)

    if configs['record']:
        recorder = recorderClass(configs['path'], configs['camera_ports'])

    for thread in rx_threads:
        thread.start()
