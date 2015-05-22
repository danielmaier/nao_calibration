/*
 * ColorMarkerDetection.cpp
 *
 *  Created on: 02.04.2014
 *      Author: stefan
 */

#include "../../include/data_capturing/ColorMarkerDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <ros/console.h>
//#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

namespace kinematic_calibration {

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

ColorMarkerDetection::ColorMarkerDetection(string userParsFileName,
		string classParsFileName) :
		cmdetect(userParsFileName, classParsFileName) {

}

ColorMarkerDetection::~ColorMarkerDetection() {

}

bool ColorMarkerDetection::detect(const cv::Mat& image, vector<double>& out) {
	cv::Mat imgCopy = image.clone();
	IplImage img2 = imgCopy;

	// detect
	cmdetect.AccessImage((uchar*)img2.imageData, img2.width, img2.height, img2.widthStep);
	if (cmdetect.FindTarget() < 1)
		return false;

	double x = cmdetect.outValues.center.iX;
	double y = cmdetect.outValues.center.iY;
	out.clear();
	out.push_back(x);
	out.push_back(y);

	// if one coordinate is (close to) zero, the detection might have failed
	if(fabs(x) < 1e-9 || fabs(y) < 1e-9)
		return false;

	// save detected point
	saveImage(image.clone());
	savePosition(out);

	return true;
}

bool ColorMarkerDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	// Convert from ROS message to OpenCV image.
	cv_bridge::CvImageConstPtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv::Mat image = cv_ptr->image;

	return detect(image, out);
}

void ColorMarkerDetection::drawMarker(cv::Mat& image) {
	// again, call the detection method which draws the detected marker into the image
	IplImage img2 = image;
	cmdetect.AccessImage((uchar*) img2.imageData, img2.width, img2.height,
			img2.widthStep);
	cmdetect.FindTarget();

	// draw the (formerly) detected center point
	SinglePointMarkerDetection::drawMarker(image);
}

} /* namespace kinematic_calibration */

