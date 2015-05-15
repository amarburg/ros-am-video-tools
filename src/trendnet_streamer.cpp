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

using namespace cv;

int main(int argc, char **argv) {

  ros::init(argc, argv, "trendnet_streamer" );
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  image_transport::Publisher pubMono = it.advertise("camera/mono_image", 1); // I suspect this isn't a very RoS way to do this...  should be re-published by another node?
  
  const std::string userName( TN_USERNAME ),
                    password( TN_PASSWD ),
                    address( TN_IPADDR  ),
                    path( "" );
  const std::string videoStreamAddress = "rtsp://" + userName + ":" + password + "@" + address + "/" + path;
  //const std::string videoStreamAddress = "rtp://127.0.0.1:12346/";

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

      Mat grey;
      cv::cvtColor( image, grey, COLOR_BGR2GRAY );

      pubMono.publish( cv_bridge::CvImage( std_msgs::Header(), "mono8", grey ).toImageMsg() );

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

