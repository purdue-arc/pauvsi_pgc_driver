#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "../include/pgc_driver/Driver.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pgcDriver", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	Driver driver = Driver();

	//set up driver's publishers according to ros standards
	if(driver.publish_distort && driver.publish_mono) {driver.monoPublisher = it.advertiseCamera(driver.topic + "/image", 1);}
	if(driver.publish_distort && driver.publish_color) {driver.colorPublisher = it.advertiseCamera(driver.topic + "/image_color", 1);}
	if(driver.publish_rect && driver.publish_color) {driver.rectColorPublisher = it.advertiseCamera(driver.topic + "/image_rect_color", 1);}
	if(driver.publish_rect && driver.publish_mono) {driver.rectMonoPublisher = it.advertiseCamera(driver.topic + "/image_rect", 1);}
	if(driver.publish_scaled_mono) {driver.scaledMonoPublisher = it.advertiseCamera(driver.topic + "/image_scaled", 1);}

	// try to connect the camera
	while(!driver.connectCamera(driver.getSerialNumber()) && nh.ok())
	{
		ros::Duration wait(1);
		wait.sleep(); // wait 1 second
	}

	ros::Rate loop_rate(driver.frame_rate);

	while(nh.ok())
	{
		ROS_DEBUG_ONCE("capturing image");
		cv::Mat image = driver.captureImage();

		//driver.viewImage(image);

		driver.publishMessages(image);

		loop_rate.sleep();
	}


	return 0;
}
