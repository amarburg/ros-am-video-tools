//
// Based on demo code from:
//   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>

#include <tclap/CmdLine.h>

#include "composite_video.h"
#include "composite_canvas.h"

using namespace std;
using namespace cv;
using namespace AplCam;

class StreamerOpts {
public:

	StreamerOpts( int argc, char **argv )
	: valid( parseArgs( argc, argv ) )
	{ ; }

	bool doLoop;
	string videoFile;

	bool valid;

	bool parseArgs( int argc, char **argv ) {

		try {
			TCLAP::CmdLine cmd("stereo_streamer", ' ', "0.1" );

			TCLAP::SwitchArg doLoopArg("l", "loop", "", cmd, false );
			TCLAP::UnlabeledValueArg< std::string > videoFileArg("video-file", "Video file", true, "", "file name", cmd );

			cmd.parse( argc, argv );

			doLoop = doLoopArg.getValue();
			videoFile = videoFileArg.getValue();

		} catch( TCLAP::ArgException &e ) {
			ROS_ERROR( "error: %s for arg %s", e.error().c_str(), e.argId().c_str() );
			return false;
		}

		return validate( );
	}

	bool validate( void )
	{
		return true;
	}

};


int main(int argc, char **argv) {

	ros::init(argc, argv, "stereo_streamer" );

	StreamerOpts opts( argc, argv );
	if( !opts.valid ) exit(-1);

	string videoFile( opts.videoFile );

	ros::NodeHandle nh( ros::this_node::getName() ),
		leftNh( nh, "left" ),
		rightNh( nh, "right" ),
    stereoNh( nh, "stereo" );
	image_transport::ImageTransport leftIt(leftNh), rightIt( rightNh ), stereoIt( stereoNh );
	image_transport::Publisher leftPub = leftIt.advertise("image_raw", 1),
													   rightPub = rightIt.advertise("image_raw", 1),
														 stereoPub = stereoIt.advertise("image_raw", 1);

	// Deal with camera info
	string leftName, rightName;

	if(  nh.searchParam("config/left/name", leftName ) ) {
		nh.getParam( leftName, leftName );
	} else {
		ROS_INFO("Using default left camera name");
		leftName = "left";
	}

	if( nh.searchParam("config/right/name", rightName ) ) {
		nh.getParam( rightName, rightName );
	} else {
		ROS_INFO("Using default right camera name");
		rightName = "right";
	}

	camera_info_manager::CameraInfoManager leftInfo( leftNh, leftName ),
																				 rightInfo( rightNh, rightName );


	ROS_INFO( "Left camera is named \"%s\"", leftName.c_str() );
	ROS_INFO( "Right camera is named \"%s\"", rightName.c_str() );

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
	float fps = cap.fps() / 5;
	if( fps < 0 ) fps = 29.97;

	ros::Rate loop_rate( fps );

	while( nh.ok() ) {
		if( cap.read( canvas ) ) {

			//TODO:  Compensate for time of processing
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas[0]).toImageMsg();
			leftPub.publish(msg);

			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas[1]).toImageMsg();
			rightPub.publish(msg);

			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas).toImageMsg();
			stereoPub.publish(msg);

		} else {
			if( ! opts.doLoop ) break;

			// Should put some flapping control in here, it's possible
			// to get here every loop if the video reader has failed
			cap.rewind();
		}

		ros::spinOnce();
		loop_rate.sleep();

	}


}
