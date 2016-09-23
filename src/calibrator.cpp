#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
/*
 * Written by Kevin Sheridan. fisheye / pinhole Camera calibrator using opencv functions
 * this program has been written to work with ROS and image_transport
 */

#define PINHOLE 1
#define FISHEYE 2
#define PINHOLE_STRING "pinhole"
#define FISHEYE_STRING "fisheye"
#define DEFAULT_BOARD_WIDTH 1
#define DEFAULT_BOARD_HEIGHT 1
#define DEFAULT_NUM_FRAMES 10
#define DEFAUlT_TOPIC "/camera/image"
#define DEFAULT_DISTORTION_MODEL "fisheye"
#define LOOP_RATE 10

int board_width = 0;
int board_height = 0;
int numFrames = 0;
int distortionModel = 1;
std::string distortionModelString;
std::string cameraTopic;

/*
 * reads parameters from ros parameter server
 */
void readParameters()
{
	ros::param::param<int>("~board_width", board_width, DEFAULT_BOARD_WIDTH);
	ros::param::param<int>("~board_height", board_height, DEFAULT_BOARD_HEIGHT);
	ros::param::param<int>("~number_frames", numFrames, DEFAULT_NUM_FRAMES);
	ROS_INFO_STREAM("Looking for " << numFrames << " images of a board with the size " << board_width << " x " << board_height);

	ros::param::param<std::string>("~camera_topic", cameraTopic, DEFAUlT_TOPIC);
	ROS_INFO_STREAM("Using images from topic: " << cameraTopic);

	ros::param::param<std::string>("~distortion_model", distortionModelString, DEFAULT_DISTORTION_MODEL);
	if(distortionModelString.compare(PINHOLE_STRING))
	{
		distortionModel = PINHOLE;
	}
	else
	{
		distortionModel = FISHEYE;
	}
	ROS_INFO_STREAM("using distortion model " << distortionModel << " or " << distortionModelString);
}

/*
 * the callback function for the image subscriber
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrator");
	ros::NodeHandle nh;

	readParameters();

	ros::Rate loop_rate(LOOP_RATE);

	ros::spin();

	return 0;
}
