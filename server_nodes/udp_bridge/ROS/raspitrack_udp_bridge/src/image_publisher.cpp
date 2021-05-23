#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <camera_info_manager/camera_info_manager.h>
#include "sensor_msgs/CameraInfo.h"
#define MAX_BRIGHT_PIXELS 4096
struct pixel_data_t{
 int16_t row;
 int16_t col;
 int32_t val;
};
union udp_packet_t{
    struct{
        uint32_t seq_length;
        uint64_t timestamp;
        uint32_t status;
        struct pixel_data_t pixels[MAX_BRIGHT_PIXELS];
    }Data;
    char buffer[MAX_BRIGHT_PIXELS*8+16];
};
using namespace cv;
image_transport::Publisher pub;
ros::Publisher info_pub;
camera_info_manager::CameraInfoManager *info_mgr;

void *image_publisher_thread(void *vargp)
{
  printf("image publisher thread started!\r\n");
  ros::NodeHandle nh=*(ros::NodeHandle *)vargp;
  //Get the IP address and Port Number
  std::string ip_address;
  int port_number, image_width=640, image_height=480;

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

  nh.getParam("image_width", image_width);
  nh.getParam("image_height", image_height);
  printf("Image size is set to: (%d,%d)\n", image_width,image_height);
  unsigned long long old_timestamp=0;
  udp_packet_t udp_packet;
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
    n = recvfrom(transmit_socket, (char *)udp_packet.buffer, sizeof(udp_packet.buffer),
          MSG_WAITALL, ( struct sockaddr *) &cliaddr, &length);
    Mat img = Mat::zeros(image_height,image_width,CV_8UC1);
    if(udp_packet.Data.status==0)
        for(int i=0; i< udp_packet.Data.seq_length; i++)
        {
            int row=udp_packet.Data.pixels[i].row;
            int col=udp_packet.Data.pixels[i].col;
            img.at<uint8_t>(row,col)=udp_packet.Data.pixels[i].val;
        }
    Mat img_gray;
    cvtColor(img,img_gray,CV_BayerBG2GRAY);
    std_msgs::Header header;
    header.stamp=ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
    // CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));
    sensor_msgs::CameraInfo msgCameraInfo;
    msgCameraInfo=info_mgr->getCameraInfo();
    msgCameraInfo.header=header;
    pub.publish(msg);
    info_pub.publish(msgCameraInfo);
    // imshow("Debug Frame",img);
    // waitKey(1);
    // printf("A packet is received by the thread %lld\r\n",udp_packet.Data.timestamp);
  }
}

void *ros_thread(void *vargp)
{
  printf("ROS tred started!\r\n");
  ros::NodeHandle nh=*(ros::NodeHandle *)vargp;
  while(nh.ok())
  {
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raspi_tracker_image_publisher");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  pub = it.advertise("raspi_tracker/raw_image", 0);
  //Camera Info
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("raspi_tracker/camera_info", 0);
  info_mgr = new camera_info_manager::CameraInfoManager(nh);
  info_mgr->setCameraName("raspi_tracker");
  std::string url;
  nh.getParam("calib_file_url",url);
  info_mgr->loadCameraInfo(url);
  //Start the threads
  pthread_t publisher_thread_id, ros_thread_id;
  pthread_create(&publisher_thread_id,NULL,image_publisher_thread,&nh);
  pthread_create(&ros_thread_id,NULL,ros_thread,&nh);
  pthread_join(ros_thread_id, NULL);
  while (nh.ok()) {


    // loop_rate.sleep();
  }
}
