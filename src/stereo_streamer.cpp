//
// Based on demo code from: 
//   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#if CV_MAJOR_VERSION > 2    // Only for OpenCV3
#include <opencv2/videoio.hpp>
#endif
#include <iostream>

#include "config.h"

#include "composite_video.h"
#include "composite_canvas.h"

using namespace std;
using namespace cv;
using namespace AplCam;

int main(int argc, char **argv) {

  ros::init(argc, argv, "stereo_streamer" );

  if( argc < 2 ) {
    ROS_ERROR("Video file should be specified on command line" );
    return -1;
  }

  string videoFile( argv[argc-1] );

  ros::NodeHandle nh( ros::this_node::getName() ),
                  leftNh( nh, "left" ),
                  rightNh( nh, "right" );
  image_transport::ImageTransport leftIt(leftNh), rightIt( rightNh );
  image_transport::Publisher leftPub = leftIt.advertise("image_raw", 1),
                             rightPub = rightIt.advertise("image_raw", 1);

  // Deal with camera info
  string leftName, rightName;

  if( ! nh.searchParam("left/name", leftName ) ) {
    ROS_INFO("Using default left camera name");
    leftName = "left";
  }

  if( ! nh.searchParam("right/name", rightName ) ) {
    ROS_INFO("Using default right camera name");
    rightName = "right";
  }

  camera_info_manager::CameraInfoManager leftInfo( leftNh ),
                                         rightInfo( rightNh );

  leftInfo.setCameraName( leftName );
  rightInfo.setCameraName( rightName );



  ROS_INFO("Opening stereo file %s", videoFile.c_str() );


  CompositeVideo cap( videoFile );

  //open the video stream and make sure it's opened
  if( !cap.isOpened() )
  {
    ROS_ERROR( "Unable to open video file: %s", videoFile.c_str() );
    return -1;
  }

  CompositeCanvas canvas;
  sensor_msgs::ImagePtr msg;

  // Use default fps if video files doesn't specify
  float fps = cap.fps();
  if( fps < 0 ) fps = 30.0;

  ros::Rate loop_rate( fps );
  while( cap.read( canvas ) ) {

    //TODO:  Compensate for time of processing
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas[0]).toImageMsg();
    leftPub.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas[1]).toImageMsg();
    rightPub.publish(msg);


    ros::spinOnce();
   // loop_rate.sleep();

  }
}

