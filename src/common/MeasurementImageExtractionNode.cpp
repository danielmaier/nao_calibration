/*
 * MeasurementImageExtractionNode.cpp
 *
 *  Created on: 15.04.2014
 *      Author: stefan
 */

#include "../../include/common/MeasurementImageExtractionNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace kinematic_calibration {

MeasurementImageExtractionNode::MeasurementImageExtractionNode() {
	// register the subscriber
	this->measurementSubscriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&MeasurementImageExtractionNode::measurementCb, this);
}

MeasurementImageExtractionNode::~MeasurementImageExtractionNode() {
	// nothing to do
}

void MeasurementImageExtractionNode::run() {
	// start the endless loop...
	while (ros::ok()) {
		ros::spinOnce();
	}
}

void MeasurementImageExtractionNode::measurementCb(
		const measurementDataConstPtr& msg) {
	// convert the image message...
	cv::Mat image;
	msgToImg(msg->image, image);

	// ... and write it!
	writeImage(image, msg->id);
}

void MeasurementImageExtractionNode::writeImage(const Mat& image, const string& name) {
	// construct the path := folder + name + extension
	stringstream ss;
	ss << this->outFolder;
	ss << name << ".jpg";
	string filename = ss.str();

	// write to disk
	cv::imwrite(filename, image);
}

void MeasurementImageExtractionNode::msgToImg(
		const sensor_msgs::Image& in_msg, cv::Mat& out_image) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvCopy(in_msg,
				sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	out_image = cv_ptr->image;
}

void MeasurementImageExtractionNode::setOutFolder(const string& outFolder)
{
	this->outFolder = outFolder;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "MeasurementImageExtractionNode");
	string folder = "/tmp/";
	MeasurementImageExtractionNode node;
	node.setOutFolder(folder);
	node.run();
}
