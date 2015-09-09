// Cribbed from:
// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv.h"
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

double width = 0;
Size scaleSize(0,0);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  Mat imageOut;

  try
  {
    if( width > 0 ) {
      Mat fullImage( cv_bridge::toCvShare(msg, "bgr8")->image );
      if( fullImage.size().width <= 0 ) return;

      if( scaleSize.width == 0 || scaleSize.height == 0 ) {
        // Initialize scaleSize
        float scale = width / fullImage.size().width;
        scaleSize.width = width;
        scaleSize.height = round( scale * fullImage.size().height );
      }

      cv::resize( fullImage, imageOut, scaleSize );
    } else {
      imageOut = cv_bridge::toCvShare(msg, "bgr8")->image;
    }

    cv::imshow("view", imageOut );
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_player");
  ros::NodeHandle nh( ros::this_node::getName() );

  std::string imageTopic = "";
  const std::string defaultImageTopic = "camera/image";

  if( (imageTopic = ros::names::remap("image")) == "image" ) {
    ROS_WARN("Topic 'image' has not been remapped.  Defaulting to \"%s\"", defaultImageTopic.c_str());
    imageTopic = defaultImageTopic;
  }

  string widthKey;
  if(  nh.searchParam("config/width", widthKey ) ) {
    if( nh.getParam( widthKey, width ) ) {

      ROS_INFO("Got width value of \"%f\" from key \"%s\"", width, widthKey.c_str() );

    } else {
      ROS_WARN("Thought I found a width at \"%s\", but I couldn't retrieve it.", widthKey.c_str() );
    }
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(imageTopic, 1, imageCallback);

  cv::namedWindow("view");
  cv::startWindowThread();
  ros::spin();
  cv::destroyWindow("view");
}
