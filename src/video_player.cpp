// Cribbed from:
// http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_player");

  std::string imageTopic = "";
  const std::string defaultImageTopic = "camera/image";

  if( (imageTopic = ros::names::remap("image")) == "image" ) {
    ROS_WARN("Topic 'image' has not been remapped.  Defaulting to \"%s\"", defaultImageTopic.c_str());
    imageTopic = defaultImageTopic;
  }

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(imageTopic, 1, imageCallback);

  cv::namedWindow("view");
  cv::startWindowThread();
  ros::spin();
  cv::destroyWindow("view");
}
