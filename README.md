# ARAS IR-Tracker
## Introduction

ARAS IR-Tracker is an open source python infrared marker tracking system similar to the famous Vicon motion capture system. It was desgned and built at Advanced Robotics and Automation Systems (ARAS) laboratories at K.N.Toosi University of Technology in Tehran (link) to serve as referencing system for robotics research. 

The system is designed to be modular with each module encapsulating isolated functionalities required by the system. It is hardware agnostic and by simply designing new tracker nodes, various implementations of the camera system can be added.

## Nodes

The system is comprised of vairous nodes and continues to grow as new functionalities and hardware supprot are added to the system. The system structure is shown in the following and is made up of the following nodes:

- tracker_nodes: Hardware interface to various camera systems. Each node in this directory interracts with its specific camera hardware and extracts the 2D pixel positions of detected markers in the images. Their detections are then transmitted to the server based on a unified UDP data structure. 

- aggregator_node: This node is executed on the central server and aggregates the packets coming from all the camera nodes in the system. It implements a queue to synchronize the reciving packets from the camera nodes and combines them into a unifeid data packet that is then processed by pose tracker nodes.

- stereo_pose_tracker: This node is specifically designed for stereo camera setups. Based on the configuration parameters defined for it, it acquires the tracked markers from two of the cameras in the system and tracks the 3D positon of single markers, the full pose of marker sets, or a general pointcloude set of the tracked and matched markers.

## How to Use the System

To be added ...
