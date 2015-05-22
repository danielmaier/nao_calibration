/*
 * ColorMarkerDetectionMain.cpp
 *
 *  Created on: 02.04.2013
 *      Author: stefan
 */

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <opencv2/core/core.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <string>
#include <vector>

#include "../../include/data_capturing/ColorMarkerDetection.h"

using namespace kinematic_calibration;
using namespace std;

#ifndef PARAMSPATH
#define PARAMSPATH ""
#endif

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void detectFromRosMsg();
void detectFromFile(string filename);

string userParsFileName, classParsFileName;
image_transport::Publisher cmPub;

int main(int argc, char** argv) {
	ros::init(argc, argv, "colorMarkerDetectionMain");
	stringstream userParsFileNameStream, classParsFileNameStream;
	userParsFileNameStream << PARAMSPATH << "CMUserPars.xml";
	classParsFileNameStream << PARAMSPATH << "DesignPars.xml";
	userParsFileName = userParsFileNameStream.str();
	classParsFileName = classParsFileNameStream.str();
	ROS_INFO("userParsFileName is %s", userParsFileName.c_str());
	ROS_INFO("classParsFileName is %s", classParsFileName.c_str());

	if (argc > 1) {
		string filename(argv[1]);
		detectFromFile(filename);
	} else {
		detectFromRosMsg();
	}
	return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
	static ColorMarkerDetection cmd(userParsFileName, classParsFileName);
	vector<double> data;

	if (!cmd.detect(msg, data)) {
		std::cout << "[main] could not detect the color marker.\n";
		// publish the input image with the detected color marker
		cmPub.publish(msg);
		return;
	}
	std::cout << "position " << data[0] << " " << data[1] << "\n";

	cv::Mat image;
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	image = input_bridge->image;

	cv::Point2f center(data[0], data[1]);
	// circle center
	cv::circle(image, center, 3, cv::Scalar(255, 255, 255), -1, 8, 0);

	// publish the input image with the detected color marker
	cmPub.publish(input_bridge->toImageMsg());
}

void detectFromRosMsg() {
	ros::NodeHandle nh;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it(nh);

	sub = it.subscribe("/nao_camera/image_raw", 1, imageCb);
	cmPub = it.advertise("/color_marker_detection/marker", 1);
	ros::spin();
}

void detectFromFile(string filename) {
	cv::Mat image;
	image = cv::imread(filename);

	if (image.empty()) {
		std::cout << "image could not be loaded\n";
		return;
	} else {
		std::cout << "size: " << image.cols << "x" << image.rows << "\n";
	}

	ColorMarkerDetection cmd(userParsFileName, classParsFileName);
	vector<double> data;

	if (!cmd.detect(image, data)) {
		std::cout << "[main] could not detect the color marker.\n";
		return;
	}

	std::cout << "position " << data[0] << " " << data[1] << "\n";

	cv::Point2f center(data[0], data[1]);
	// circle center
	cv::circle(image, center, 3, cv::Scalar(255, 255, 255), -1, 8, 0);

	cv::imshow("test", image);
	cv::waitKey();
}

