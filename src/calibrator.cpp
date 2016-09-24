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

#define DATA_COLLECTION_MODE 1
#define DISPLAY_MODE 2

int board_width = 0;
int board_height = 0;
int numFrames = 0;
int distortionModel = 1;
std::string distortionModelString;
std::string cameraTopic;

cv::Mat intrinsic;
cv::Mat distortion;

int mode = DATA_COLLECTION_MODE; // start in data collection mode

std::vector<std::vector<cv::Point2f> > allCorners;
std::vector<std::vector<cv::Point3f> > allPatterns;

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
	if(distortionModelString == PINHOLE_STRING)
	{
		distortionModel = PINHOLE;
	}
	else
	{
		distortionModel = FISHEYE;
	}
	ROS_INFO_STREAM("using distortion model " << distortionModel << " or " << distortionModelString);
}

bool isMatGrayscale(cv::Mat img)
{
	bool isGray = true;
	for(int i = 0; i < img.rows; i++)
	{
		for(int j = 0; j < img.cols; j++)
		{
			cv::Vec3b pixel = img.at<cv::Vec3b>(i, j);
			if(!(pixel[0] == pixel[1] == pixel[2]))
			{
				isGray = false;
			}
		}
	}

	return isGray;
}

/*
 * returns a vector of 3d points that represent all equidistant corners on the chessboard in 3d space
 */
std::vector<cv::Point3f> createChessboardPoints(int width, int height, float pointDistance)
{
	std::vector<cv::Point3f> chessboard_points;

	for(int i = 0; i < width; i++)
	{
		for(int j = 0; j < height; j++)
		{
			cv::Point3f point(pointDistance * i, pointDistance * j, 0.0f);
			chessboard_points.push_back(point);
		}
	}

	return chessboard_points;
}

/*
 * the callback function for the image subscriber
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();

	if(mode == DATA_COLLECTION_MODE)
	{

		cvtColor(image, image, CV_BGR2GRAY); // convert image to grayscale

		std::vector<cv::Point2f> corners;

		//find the chessboard corners in the image
		bool foundChessBoard = cv::findChessboardCorners(image, cv::Size2d(board_width, board_height), corners);
		if(foundChessBoard){cv::find4QuadCornerSubpix(image, corners, cv::Size2d(board_width, board_height));}

		cv::drawChessboardCorners(image, cv::Size2d(board_width, board_height), corners, foundChessBoard);

		if(foundChessBoard)
		{
			allCorners.push_back(corners); //add corners to the corner matrix
			allPatterns.push_back(createChessboardPoints(board_width, board_height, 1.0));

			//check if enough frames have been captured
			if(allCorners.size() >= numFrames)
			{
				ROS_DEBUG_STREAM("object points total: " << allPatterns.size() << " image points total: " << allCorners.size());
				mode = DISPLAY_MODE; // set mode to display

				std::vector<cv::Mat> tvecs;
				std::vector<cv::Mat> rvecs;
				double rms;
				//CALIBRATE
				if(distortionModel == FISHEYE)
				{
					ROS_INFO("Beginning fisheye calibration...");
					rms = cv::fisheye::calibrate(allPatterns, allCorners, cv::Size(image.rows, image.cols), intrinsic, distortion, rvecs, tvecs);
				}
				else if(distortionModel == PINHOLE)
				{

				}

				ROS_INFO_STREAM("Calibration complete with an error of " << rms);
				ROS_INFO("Please copy these matrices into a camera parameter file.");
				ROS_INFO_STREAM("Intrinsic Matrix: " << intrinsic);
				ROS_INFO_STREAM("Distortion Matrix: " << distortion);

			}
		}
	}

	cv::imshow("Calibration", image);
	cv::waitKey(30);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrator");
	ros::NodeHandle nh;

	readParameters();

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub;
	imageSub = it.subscribe(cameraTopic, 1, imageCallback);

	ros::spin();

	return 0;
}
