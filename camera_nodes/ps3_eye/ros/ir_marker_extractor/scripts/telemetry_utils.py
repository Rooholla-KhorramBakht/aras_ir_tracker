import socket
import struct
import numpy as np
import time

class udp_telemetry():
    def __init__(self):
        self.OUT_IP='192.168.1.3'
        self.OUT_PORT=5000
        self.INPUT_IP='127.0.0.1'
        self.INPUT_PORT=5000
        self.out_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.RX_RUNNING = True

    def transmit_data(self,markers_list):
        num_frames=len(markers_list)
        markers_per_frame = [markers.shape[0] for markers in markers_list]
        msg_format='i{}i{}d'.format(len(markers_per_frame),2*sum(markers_per_frame))
        marker_data=np.vstack(markers_list)
        arguments = [msg_format] + [num_frames] + markers_per_frame + marker_data.reshape(-1).tolist()
        data=struct.pack(*arguments)
        self.out_socket.sendto(data, (self.OUT_IP, self.OUT_PORT))

    def start_rx(self,callback):
        self.in_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.in_sock.bind((self.INPUT_IP, self.INPUT_PORT))

        while self.RX_RUNNING:
            data = self.in_sock.recvmsg(4096)
            num_of_frames=struct.unpack('i',data[0][0:4])[0]
            markers_per_frame = struct.unpack('{}i'.format(num_of_frames,data[0][4:4+4*num_of_frames]))
            total_number_of_markers = sum(markers_per_frame)
            msg_format='i{}i{}d'.format(markers_per_frame,2*sum(markers_per_frame))
            unpacked=struct.unpack(msg_format,data[0])
            all_markers = np.array(unpacked[(-2*sum(markers_per_frame)):]).reshape(-1,2)
            markers_in_frames=np.split(all_markers, markers_per_frame)
            stamp = time.time()
            callback(stamp, self.INPUT_PORT, markers_in_frames)

    def terminate(self):
        self.RX_RUNNING = False
        in_sock.close()
        out_sock.close()
