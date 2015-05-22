/*
 * SinglePointMarkerDetection.cpp
 *
 *  Created on: 28.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/SinglePointMarkerDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <vector>

namespace kinematic_calibration {

SinglePointMarkerDetection::SinglePointMarkerDetection() {
	// nothing to do
}

SinglePointMarkerDetection::~SinglePointMarkerDetection() {
	// nothing to do
}

void SinglePointMarkerDetection::drawMarker(cv::Mat& image) {
	// check whether the marker position was saved
	if (this->position.size() < 2) {
		ROS_ERROR("Marker position was not set!");
		return;
	}

	// draw the marker position
	std::vector<cv::Point2f> corners;
	corners.push_back(cv::Point2f(position[0], position[1]));
	cv::drawChessboardCorners(image, cv::Size(1, 1), corners, true);
}

void SinglePointMarkerDetection::getImage(cv::Mat& image) {
	image = this->image;
}

void SinglePointMarkerDetection::saveImage(
		const sensor_msgs::ImageConstPtr& in_msg) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg,
				sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	saveImage(cv_ptr->image.clone());
}

void SinglePointMarkerDetection::saveImage(cv::Mat image) {
	this->image = image;
}

void SinglePointMarkerDetection::savePosition(vector<double>& position) {
	this->position = position;
}

} /* namespace kinematic_calibration */
