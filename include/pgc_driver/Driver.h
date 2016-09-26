/*
 * Driver.h
 *
 *  Created on: Sep 21, 2016
 *      Author: kevinsheridan
 */

#ifndef PAUVSI_M7_PAUVSI_PGC_DRIVER_INCLUDE_PGC_DRIVER_DRIVER_H_
#define PAUVSI_M7_PAUVSI_PGC_DRIVER_INCLUDE_PGC_DRIVER_DRIVER_H_

#include <ros/ros.h>
#include <flycapture/FlyCapture2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <string>
#include <math.h>

#define DEFAULT_RATE 10
#define DEFAULT_SCALE_X 640
#define DEFAULT_SCALE_Y 480

// these must be global
FlyCapture2::Error error;
FlyCapture2::Camera camera;
FlyCapture2::CameraInfo camInfo;
FlyCapture2::BusManager busMngr;

/*
 * the purpose of this driver class is to abstract the FlyCapture SDK from the pgcDriver node
 */
class Driver
{
private:
	int serial_number;
	std::string intrinsicString;
	std::string distortionString;
	cv::Mat intrinsic;
	cv::Mat distortion;
	ros::Time imageStamp;
	std::string cameraFrame;
	std::string distortionModel;
	int scaleX;
	int scaleY;

public:
	std::string topic;
	bool publish_distort;
	bool publish_rect;
	bool publish_mono;
	bool publish_color;
	bool publish_scaled_mono;
	int frame_rate;

	image_transport::CameraPublisher colorPublisher;
	image_transport::CameraPublisher monoPublisher;
	image_transport::CameraPublisher rectColorPublisher;
	image_transport::CameraPublisher rectMonoPublisher;
	image_transport::CameraPublisher scaledMonoPublisher;

	Driver(){
		this->getParameters(); // get parameters from the param server

		intrinsic = this->createMatFromString(intrinsicString); //convert intrinsic string to cv::Mat
		distortion = this->createMatFromString(distortionString); //convert distortion string to cv::Mat

	}

	/*
	 * reads and sets all parameters from the parameter server
	 */
	void getParameters()
	{
		//get the parameters
		//rate
		ros::param::param<int>("~frame_rate", frame_rate, DEFAULT_RATE);
		ROS_INFO("Camera Frame Rate: %i", frame_rate);

		//serial
		ros::param::param<int>("~serial_number", serial_number, 0);
		ROS_INFO("Driver using %i as serial number", serial_number);

		//camera topic
		ros::param::param<std::string>("~topic", topic, "/camera");

		// distortion publish
		ros::param::param<bool>("~publish_distort", publish_distort, true);
		ros::param::param<bool>("~publish_rectified", publish_rect, false);

		// color publish
		ros::param::param<bool>("~publish_mono", publish_mono, false);
		ros::param::param<bool>("~publish_color", publish_color, true);

		//scale
		ros::param::param<bool>("~publish_scaled_mono", publish_scaled_mono, false);
		ros::param::param<int>("~scaleX", scaleX, DEFAULT_SCALE_X);
		ros::param::param<int>("~scaleY", scaleY, DEFAULT_SCALE_Y);

		// undistort matrices
		ros::param::param<std::string>("~intrinsic", intrinsicString, "0");
		ros::param::param<std::string>("~distortion", distortionString, "0");

		//model
		ros::param::param<std::string>("~distortionModel", distortionModel, "fisheye");

		//frame
		ros::param::param<std::string>("~frame", cameraFrame, "camera");
	}

	/*
	 * will capture image from camera and set the image timestamp
	 */
	cv::Mat captureImage()
	{
		//SET TIMESTAMP
		this->imageStamp = ros::Time::now();

		//CAPTURE
		//------------------------------------
		FlyCapture2::Image rawImage;
		error = camera.RetrieveBuffer( &rawImage );
		if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_ERROR("Capture Error");
		}

		// CONVERT TO BGR
		FlyCapture2::Image bgrImage;
		rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage );

		// CONVERT TO MAT
		unsigned int rowBytes = (double)bgrImage.GetReceivedDataSize()/(double)bgrImage.GetRows();
		//SET IMAGE
		cv::Mat image = (cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(),rowBytes));

		ROS_ERROR_COND(image.cols == 0 || image.rows == 0, "image size is 0 X 0");
		ROS_DEBUG_STREAM_ONCE("image size: " << image.cols << " X " << image.rows);

		//this->viewImage(this->rawImage);

		return image;
	}

	/*
	 * views the image using opencv's imshow
	 */
	void viewImage(cv::Mat img)
	{
		cv::imshow("debug", img);
		cv::waitKey(1);
	}
	/*
	 * This connects the PG Chameleon or other camera to a serial number
	 */
	bool connectCamera(int serNum)
	{
		FlyCapture2::PGRGuid guid; //id of the camera
		//get the camera with specific serial
		error = busMngr.GetCameraFromSerialNumber(serNum, &guid);
		if(error != FlyCapture2::PGRERROR_OK)
		{
			ROS_ERROR("Cannot find camera with specified serial number! - %i", serNum);
			ROS_WARN("Connecting to any camera on bus");
			error = camera.Connect( 0 ); // connect to default cam index 0
		}
		else
		{
			error = camera.Connect(&guid);
		}

		if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_ERROR("Failed to connect to camera\n");
			return false;
		}

		// Get the camera info and print it out
		error = camera.GetCameraInfo( &camInfo );
		if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_ERROR("Failed to get camera info from camera\n");
			return false;
		}
		std::string tempStr = std::string();
		ROS_INFO_STREAM(camInfo.modelName);
		ROS_INFO_STREAM(camInfo.sensorInfo);
		ROS_INFO("%i", camInfo.serialNumber);

		camera.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_FORMAT7, FlyCapture2::FRAMERATE_60);

		error = camera.StartCapture();
		if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
		{
			ROS_WARN("Bandwidth exceeded\n");
			return false;
		}
		else if ( error != FlyCapture2::PGRERROR_OK )
		{
			ROS_ERROR("Failed to start image capture\n");
			return false;
		}
		else if (error == FlyCapture2::PGRERROR_OK)
		{
			ROS_INFO("Started the image capture");
		}

		ROS_DEBUG("connected camera");
		return true;
	}

	/*
	 * this will set the specified camera property to a value or range
	 */
	bool setProperty(const FlyCapture2::PropertyType type, const bool autoSet, unsigned int valueA, unsigned int valueB)
	{
		// return true if we can set values as desired.
		bool retVal = true;

		FlyCapture2::PropertyInfo pInfo;
		pInfo.type = type;

		error = camera.GetPropertyInfo(&pInfo);
		//handle the error
		//TODO

		//check if the property supports manual control
		if(!pInfo.manualSupported)
		{
			ROS_WARN_STREAM("Property type does not support manual control");
			return false;
		}

		if(pInfo.present)
		{
			FlyCapture2::Property prop;
			prop.type = type;
			prop.autoManualMode = (autoSet && pInfo.autoSupported);
			prop.absControl = false;
			prop.onOff = pInfo.onOffSupported;

			if(valueA < pInfo.min)
			{
				valueA = pInfo.min;
				retVal = false;
			}
			else if(valueA > pInfo.max)
			{
				valueA = pInfo.max;
				retVal = false;
			}
			if(valueB < pInfo.min)
			{
				valueB = pInfo.min;
				retVal = false;
			}
			else if(valueB > pInfo.max)
			{
				valueB = pInfo.max;
				retVal = false;
			}
			prop.valueA = valueA;
			prop.valueB = valueB;
			error = camera.SetProperty(&prop);
			//handle error
			if(error != FlyCapture2::PGRERROR_OK)
			{
				ROS_ERROR("Failed to set parameter");
			}

			// Read back setting to confirm
			error = camera.GetProperty(&prop);
			//handle error
			//TODO

			if(!prop.autoManualMode)
			{
				valueA = prop.valueA;
				valueB = prop.valueB;
				ROS_DEBUG_STREAM("property a and b vals are " << valueA << " and " << valueB);
			}
		}
		else     // Not supported
		{
			valueA = 0;
			valueB = 0;
		}

		return retVal;
	}

	/*
	 * This function will take a string containing ; and , and it will create a cv mat from it
	 * this creates a CV32FC1 mat by default
	 */
	cv::Mat createMatFromString(std::string text)
	{
		ROS_DEBUG_STREAM("creating MAT from " << text << std::endl);

		//size
		int rows, cols;
		//setup the row strings
		std::vector<std::string> rowStrings;
		std::stringstream textStream(text);
		std::string segment;

		while(std::getline(textStream, segment, ';'))
		{
			rowStrings.push_back(segment);
		}

		rows = (int)rowStrings.size();

		//find the column size by adding for each segment
		std::stringstream rowStream(rowStrings.at(0));
		cols = 0;
		while(std::getline(rowStream, segment, ','))
		{
			cols++;
		}

		ROS_DEBUG_STREAM("the matrix appears to be rows: " << rows << " cols: " << cols);

		//create the cv::mat
		cv::Mat matrix = cv::Mat(rows, cols, CV_32FC1);
		//now cycle through rows and create the cv::Mat
		for(int i = 0; i < rows; i++)
		{
			std::stringstream rowStream(rowStrings.at(i));
			std::string segment;
			for(int j = 0; j < cols; j++)
			{
				std::getline(rowStream, segment, ',');
				//remove whitespace
				segment = removeSpaces(segment);
				//convert and add to matrix
				ROS_DEBUG_STREAM("attempting to convert " << segment << " to a float");
				matrix.at<float>(i, j) = boost::lexical_cast< float >( segment );
				ROS_DEBUG_STREAM("converted to " << matrix.at<float>(i, j));
			}
		}

		ROS_DEBUG_STREAM("created mat. is this correct? " << matrix);

		return matrix;

	}

	/*
	 * removes the spaces from a string
	 */
	std::string removeSpaces(std::string input)
	{
		input.erase(std::remove(input.begin(),input.end(),' '),input.end());
		return input;
	}


	/*
	 * assembles and publishes all message that have subscribers
	 */
	bool publishMessages(cv::Mat rawImage)
	{
		//assemble the cameraInfo msg
		sensor_msgs::CameraInfo camInfo;
		camInfo.header.frame_id = this->cameraFrame;
		camInfo.header.stamp = this->imageStamp;
		camInfo.distortion_model = this->distortionModel;
		camInfo.height = rawImage.rows;
		camInfo.width = rawImage.cols;
		camInfo.D = this->convertMatToVector(this->distortion);
		std::vector<double> K_vec = this->convertMatToVector(this->intrinsic);
		if(K_vec.size() == 9)
		{
			for(int i = 0; i < 9; i++) camInfo.K.at(i) = K_vec.at(i); // this sets each of the members of K
		}
		else{ROS_WARN_STREAM_ONCE("intrinsic matrix specified not the right size. its size is: " << K_vec.size() << " however it must be 9");}

		//viewImage(raw_image);

		//begin publishing
		if(this->colorPublisher.getNumSubscribers() > 0)
		{
			sensor_msgs::CameraInfoPtr camInfoPtr(new sensor_msgs::CameraInfo(camInfo)); // create the camera info ptr
			//viewImage(raw_image);
			cv::Mat tempMat = rawImage.clone(); // make a deep copy of raw_image to the tempMat
			sensor_msgs::ImagePtr img = cv_bridge::CvImage(camInfo.header, "bgr8", tempMat).toImageMsg(); // create the image ptr
			ROS_DEBUG_ONCE("publishing color distorted image");

			//viewImage(rawImage);

			//this->viewImage(cv_bridge::toCvShare(img, "bgr8")->image);

			colorPublisher.publish(img, camInfoPtr);

			rawImage = tempMat.clone(); // copy the tempMat back over to rawImage
		}

		if(this->monoPublisher.getNumSubscribers() > 0)
		{
			cv::Mat tempMat = rawImage.clone(); // make a deep copy of raw_image to the tempMat
			cv::Mat grayImage;
			cvtColor(tempMat, grayImage, CV_BGR2GRAY);
			sensor_msgs::CameraInfoPtr camInfoPtr(new sensor_msgs::CameraInfo(camInfo)); // create the camera info ptr
			sensor_msgs::ImagePtr img = cv_bridge::CvImage(camInfo.header, "mono8", grayImage).toImageMsg(); // create the image ptr
			monoPublisher.publish(img, camInfoPtr);
			rawImage = tempMat.clone(); // copy the tempMat back over to rawImage
		}

		if(this->rectColorPublisher.getNumSubscribers() > 0)
		{

		}

		if(this->rectMonoPublisher.getNumSubscribers() > 0)
		{

		}

		if(this->scaledMonoPublisher.getNumSubscribers() > 0)
		{
			cv::Mat tempMat = rawImage.clone(); // make a deep copy of raw_image to the tempMat
			cv::Mat grayImage;
			cvtColor(tempMat, grayImage, CV_BGR2GRAY);
			//scale image
			cv::resize(grayImage, grayImage, cv::Size(scaleX, scaleY));
			camInfo.height = scaleY;
			camInfo.width = scaleX;

			sensor_msgs::CameraInfoPtr camInfoPtr(new sensor_msgs::CameraInfo(camInfo)); // create the camera info ptr
			sensor_msgs::ImagePtr img = cv_bridge::CvImage(camInfo.header, "mono8", grayImage).toImageMsg(); // create the image ptr
			scaledMonoPublisher.publish(img, camInfoPtr);
			rawImage = tempMat.clone(); // copy the tempMat back over to rawImage
		}
	}

	/*
	 * converts cv::Mat to vector in double
	 */
	std::vector<double> convertMatToVector(cv::Mat mat){
		std::vector<double> array;
		ROS_DEBUG_STREAM_ONCE("converting " << mat << "vector of doubles");
		for(int i = 0; i < mat.rows; i++)
		{
			for(int j = 0; j < mat.cols; j++)
			{
				array.push_back(mat.at<float>(i, j));
			}
		}

		return array;
	}

	int getSerialNumber()
	{
		return serial_number;
	}
};



#endif /* PAUVSI_M7_PAUVSI_PGC_DRIVER_INCLUDE_PGC_DRIVER_DRIVER_H_ */
