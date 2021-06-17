#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <camera_info_manager/camera_info_manager.h>
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"
#include <image_publisher_cpp/ir_markers.h>
#define CLUSTER_CNT 4

typedef struct{
	float px,py;
	float cx[CLUSTER_CNT];
	float cy[CLUSTER_CNT];
	float Rx[CLUSTER_CNT];
	float Ry[CLUSTER_CNT];
	int cnt[CLUSTER_CNT];
}clustering_result_t;

union clustering_result_packet_t{
	clustering_result_t data;
	char buffer[sizeof(clustering_result_t)];
};

image_transport::Publisher pub;
ros::Publisher info_pub, marker_pub;
camera_info_manager::CameraInfoManager *info_mgr;

void *marker_publisher_thread(void *vargp)
{
  printf("Marker publisher thread started!\r\n");
  ros::NodeHandle nh=*(ros::NodeHandle *)vargp;
  //Get the IP address and Port Number
  std::string ip_address;
  int port_number;

  if (nh.getParam("ip_address", ip_address))
  {
    printf("Raspi Tracker IP Address: %s\n",ip_address.c_str());
  }
  else
  {
    ip_address="172.0.0.1";
    printf("Raspi Tracker IP Address: %s\n",ip_address.c_str());
  }
  if (nh.getParam("port", port_number))
  {
    printf("Raspi Tracker Port Number: %d\n",port_number);
  }
  else
  {
    port_number=5000;
    printf("Raspi Tracker Port Number: %d\n",port_number);
  }

  // std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point points[CLUSTER_CNT];
  unsigned long long old_timestamp=0;
  clustering_result_packet_t udp_packet;
  int transmit_socket;
  struct sockaddr_in addr_con, cliaddr;
  memset(&cliaddr, 0, sizeof(cliaddr));
  int addrlen = sizeof(addr_con);
  addr_con.sin_family = AF_INET;
  addr_con.sin_port = htons(port_number);
  addr_con.sin_addr.s_addr = inet_addr(ip_address.c_str());
  transmit_socket = socket(AF_INET, SOCK_DGRAM,0);
  if (transmit_socket < 0)
    printf("\nCould not open the transmit socket!!\n");
  if ( bind(transmit_socket, (const struct sockaddr *)&addr_con,
            sizeof(addr_con)) < 0 )
  {
      perror("bind failed");
      exit(EXIT_FAILURE);
  }
  while(nh.ok())
  {
    int n;
    socklen_t length;
    n = recvfrom(transmit_socket, (char *)udp_packet.buffer,
                 sizeof(udp_packet.buffer),MSG_WAITALL,
                  ( struct sockaddr *) &cliaddr, &length);

    std_msgs::Header header;
    header.stamp=ros::Time::now();
    sensor_msgs::CameraInfo msgCameraInfo;
    msgCameraInfo=info_mgr->getCameraInfo();
    msgCameraInfo.header=header;
    info_pub.publish(msgCameraInfo);
    image_publisher_cpp::ir_markers ir_markers_msg;
    for(int k=0; k<CLUSTER_CNT; k++)
    {
      points[k].x=udp_packet.data.cx[k];
      points[k].y=udp_packet.data.cy[k];
      points[k].z=-1;
      if(udp_packet.data.cx[k]!=-1)
        ir_markers_msg.markers.push_back(points[k]);
    }
    ir_markers_msg.header=header;
    marker_pub.publish(ir_markers_msg);

    printf("A packet is received by the thread");
  }
}

void *ros_thread(void *vargp)
{
  printf("ROS tred started!\r\n");
  ros::NodeHandle nh=*(ros::NodeHandle *)vargp;
  while(nh.ok())
  {
    sleep(0.01);
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raspi_tracker_image_publisher");
  ros::NodeHandle nh("~");
  //Camera Info
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("raspi_tracker/camera_info", 0);
  marker_pub = nh.advertise<image_publisher_cpp::ir_markers>("raspi_tracker/markers", 0);
  info_mgr = new camera_info_manager::CameraInfoManager(nh);
  info_mgr->setCameraName("raspi_tracker");
  std::string url;
  nh.getParam("calib_file_url",url);
  info_mgr->loadCameraInfo(url);
  //Start the threads
  pthread_t publisher_thread_id, ros_thread_id;
  pthread_create(&publisher_thread_id,NULL,marker_publisher_thread,&nh);
  pthread_create(&ros_thread_id,NULL,ros_thread,&nh);
  pthread_join(ros_thread_id, NULL);
  while (nh.ok()) {
    // loop_rate.sleep();
  }
}
