//
// Based on demo code from: 
//   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION > 2    // Only for OpenCV3
  #include <opencv2/videoio/videoio.hpp>
#endif
#include <iostream>

#include "config.h"

#define TN_DEFAULT_IPADDR   "10.0.95.1"
#define TN_DEFAULT_STREAM    ""

using namespace cv;

int main(int argc, char **argv) {

  ros::init(argc, argv, "trendnet_streamer" );
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
  
  const std::string userName( TN_USERNAME ),
                    password( TN_PASSWD );
  std::string address, stream;

  if( ! ros::param::get("config/ip_addr", address ) ) {
    ROS_INFO("Using default IP address");
    address = TN_DEFAULT_IPADDR;
  }

  if( ! ros::param::get("config/stream", stream ) ) {
    ROS_INFO("Using default stream");
    address = TN_DEFAULT_STREAM;
  }


  const std::string videoStreamAddress = "rtsp://" + userName + ":" + password + "@" + address + "/" + stream;
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

  ros::Rate loop_rate(100);
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

