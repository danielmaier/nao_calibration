/*
 * ArucoMarkerDetection.cpp
 *
 *  Created on: 11.04.2014
 *      Author: stefan
 */

#include "../../include/data_capturing/ArucoMarkerDetection.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>

#include "aruco/marker.h"
#include "aruco/markerdetector.h"

namespace kinematic_calibration {

using namespace std;
using namespace cv;
using namespace aruco;

ArucoMarkerDetection::ArucoMarkerDetection() :
		markerSize(-1.0) {
	// nothing to do
}

ArucoMarkerDetection::~ArucoMarkerDetection() {
	// nothing to do
}

bool ArucoMarkerDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	// get the input image
	cv::Mat inImage;
	this->msgToImg(in_msg, inImage);
	return detect(inImage, out);
}

bool ArucoMarkerDetection::detect(const cv::Mat& inImage, vector<double>& out) {
	MarkerDetector detector;
	vector<Marker> markers;
	out.clear();

	// use a copy of the input image
	cv::Mat outImage = inImage.clone();

	// try to detect
	detector.detect(outImage, markers);
	if (markers.size() < 1) {
		// no marker found
		return false;
	}

	// save the image
	saveImage(outImage);

	// calculate the center
	cv::Point2f centerPoint(0.0, 0.0);
	for (int i = 0; i < 4; i++) {
		centerPoint += markers[0][i];
	}
	centerPoint.x = centerPoint.x / 4;
	centerPoint.y = centerPoint.y / 4;

	// save the detected points
	out.push_back(centerPoint.x);
	out.push_back(centerPoint.y);
	savePosition(out);
	this->currentMarker = markers[0];

	// append the corners point coordinates
	for (int i = 0; i < 4; i++) {
		out.push_back(currentMarker[i].x);
		out.push_back(currentMarker[i].y);
	}

	// append the marker size
	out.push_back(markerSize);

	return true;
}

void ArucoMarkerDetection::drawMarker(cv::Mat& image) {
	// draw center
	SinglePointMarkerDetection::drawMarker(image);

	// draw contour
	this->currentMarker.draw(image, Scalar(0, 0, 255), 2, false);
}

void ArucoMarkerDetection::setMarkerSize(const double& markerSize) {
	this->markerSize = markerSize;
}

} /* namespace kinematic_calibration */
