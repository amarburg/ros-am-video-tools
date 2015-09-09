//
// Based on demo code from:
//   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv.h"
#include <iostream>

#include "config.h"

#define TN_DEFAULT_IPADDR   "10.0.95.1"
#define TN_DEFAULT_STREAM    ""

#ifndef TN_USERNAME
  #define TN_USERNAME ""
#endif

#ifndef TN_PASSWD
  #define TN_PASSWD   ""
#endif

using namespace cv;

int main(int argc, char **argv) {

  ros::init(argc, argv, "trendnet_streamer" );
  ros::NodeHandle nh( ros::this_node::getName() );;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);

  const std::string userName( TN_USERNAME ),
                    password( TN_PASSWD );
  std::string address, stream;


  std::string key;
  if( nh.searchParam("ip_addr", key ) ) {
    ROS_INFO("Found it at: %s", key.c_str() );
  }

  if( ! nh.getParam("config/ip_addr", address ) ) {
    ROS_INFO("Using default IP address");
    address = TN_DEFAULT_IPADDR;
  }

  if( ! nh.getParam("config/stream", stream ) ) {
    ROS_INFO("Using default stream");
    stream = TN_DEFAULT_STREAM;
  }

  // Assemble URI to the RTSP server
  const std::string videoStreamAddress = "rtsp://";
  if( userName.length() > 0 ) {
    videoStreamAddress += userName;
    if( password.length() > 0 ) videoStreamAddress += string(":") + password;
    videoStreamAddress += "@";
  }
  videoStreamAddress += address + "/" + stream;

  //const std::string videoStreamAddress = "rtp://127.0.0.1:12346/";

  ROS_INFO("Opening %s", videoStreamAddress.c_str() );

  VideoCapture cap( videoStreamAddress );

  //open the video stream and make sure it's opened
  if( !cap.isOpened() )
  {
    ROS_ERROR( "Unable to open video source: %s", videoStreamAddress.c_str() );
    return -1;
  }

  Mat image;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(1000);
  while (nh.ok()) {
    if( cap.read( image ) ) {

      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      pub.publish(msg);

//Mat grey;
//      cv::cvtColor( image, grey, COLOR_BGR2GRAY );
//
//      pubMono.publish( cv_bridge::CvImage( std_msgs::Header(), "mono8", grey ).toImageMsg() );

    }

    ros::spinOnce();
    loop_rate.sleep();

  }
}
