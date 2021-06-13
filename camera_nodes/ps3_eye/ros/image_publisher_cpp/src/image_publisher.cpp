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
#include "opencv2/videoio.hpp"
#include <camera_info_manager/camera_info_manager.h>
#include "sensor_msgs/CameraInfo.h"
using namespace cv;
image_transport::Publisher pub;
ros::Publisher info_pub;
camera_info_manager::CameraInfoManager *info_mgr;

void *image_publisher_thread(void *vargp)
{
  ros::NodeHandle nh=*(ros::NodeHandle *)vargp;
  int id, fps;
  //Get the relavent paramteres
  if (nh.getParam("camera_id", id))
  {
    printf("Selected Camera ID is: %d\n",id);
  }
  else
  {
    id=0;
    printf("Selected Camera ID is: %d\n",id);
  }
  printf("image publisher thread started!\r\n");
  VideoCapture cap(id);
  int image_width=640, image_height=480;

  nh.getParam("image_width", image_width);
  nh.getParam("image_height", image_height);
  nh.getParam("fps", fps);

  printf("Image size is set to: (%d,%d) and the FPS is %d\n", image_width,image_height,fps);
  cap.set(3,image_width);
  cap.set(4,image_height);
  cap.set(CAP_PROP_FPS,fps);
  cap.set(CAP_PROP_BUFFERSIZE,1);

  while(nh.ok())
  {
    Mat img;
    cap.read(img);
    std_msgs::Header header;
    header.stamp=ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    // CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));
    sensor_msgs::CameraInfo msgCameraInfo;
    msgCameraInfo=info_mgr->getCameraInfo();
    msgCameraInfo.header=header;
    pub.publish(msg);
    info_pub.publish(msgCameraInfo);
//     imshow("Debug Frame",img);
//     waitKey(1);
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
  ros::init(argc, argv, "camera_publisher");
  ros::NodeHandle nh("~");
  ros::Rate rate(5); // ROS Rate at 5Hz
  image_transport::ImageTransport it(nh);
  pub = it.advertise("raw_image", 0);
  //Camera Info
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 0);
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
     rate.sleep();
  }
}
