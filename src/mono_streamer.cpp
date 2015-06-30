//
// Based on demo code from: 
//   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#if CV_MAJOR_VERSION > 2    // Only for OpenCV3
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#else
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif
#include <iostream>

#include "config.h"

#include "composite_video.h"
#include "composite_canvas.h"

using namespace std;
using namespace cv;
using namespace AplCam;

int main(int argc, char **argv) {

	ros::init(argc, argv, "mono_streamer" );

	// Todo:  Make this configurable
	float speedMult = 0.2;

	if( argc < 2 ) {
		ROS_ERROR("Video file should be specified on command line" );
		return -1;
	}

	string videoFile( argv[argc-1] );

	ros::NodeHandle nh( ros::this_node::getName() );
	image_transport::ImageTransport it( nh );
	image_transport::Publisher pub = it.advertise("image_raw", 1 );

	// Deal with camera info
	string name;

	if(  nh.searchParam("config/name", name ) ) {
		nh.getParam( name, name );
	} else {
		ROS_INFO("Using default camera name");
		name = "camera";
	}

	camera_info_manager::CameraInfoManager info( nh, name );


	ROS_INFO( "Camera is named \"%s\"", name.c_str() );
	ROS_INFO( "Opening video file %s", videoFile.c_str() );


	VideoCapture cap( videoFile );

	//open the video stream and make sure it's opened
	if( !cap.isOpened() )
	{
		ROS_ERROR( "Unable to open video file: %s", videoFile.c_str() );
		return -1;
	}

	Mat img;
	sensor_msgs::ImagePtr msg;

	// Use default fps if video files doesn't specify
	float fps = cap.get( CV_CAP_PROP_FPS );
	if( fps <= 0.0 ) fps = 29.97;
	fps *= speedMult;

	bool once = false;
	ros::Rate loop_rate( fps );
	while( nh.ok() ) {
		if( cap.read( img ) ) {
			//TODO:  Compensate for time of processing

			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img ).toImageMsg();
			pub.publish( msg );
		} else if( once == false ) {
			cout << "End of video file!" << endl;
			ROS_INFO("End of video file!");

			once = true;
		}

		ros::spinOnce();
		loop_rate.sleep();

	}
}

