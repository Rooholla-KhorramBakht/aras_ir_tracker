import socket
import struct
import numpy as np

class udp_telemetry():
    def __init__(self):
        self.OUT_IP='192.168.1.3'
        self.OUT_PORT=5000
        self.out_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def transmit_data(self,markers_list):
        num_frames=len(markers_list)
        markers_per_frame = [markers.shape[0] for markers in markers_list]
        msg_format=f'i{len(markers_per_frame)}i{2*sum(markers_per_frame)}d'
        marker_data=np.vstack(markers_list)
        arguments = [msg_format] + [num_frames] + markers_per_frame + marker_data.reshape(-1).tolist()
        data=struct.pack(*arguments)
        self.out_socket.sendto(data, (self.OUT_IP, self.OUT_PORT))
