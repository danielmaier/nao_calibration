/*
 * CheckerboardDetection.cpp
 *
 *  Created on: 23.10.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/CheckerboardDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/data_capturing/CircleDetection.h"

namespace kinematic_calibration {

namespace enc = sensor_msgs::image_encodings;

CheckerboardDetection::CheckerboardDetection() :
    squareLength(-1.0), debug(false) {
	// initialize with default value
    this->patternSize = cv::Size(3, 3);
    if (debug)
    {
        cv::namedWindow("CHESS_DETECTOR",cv::WINDOW_AUTOSIZE);
    }
    ROS_DEBUG("Created CheckerboardDetection");
}

CheckerboardDetection::~CheckerboardDetection() {
	// nothing to do
}

bool CheckerboardDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		CheckerboardData& out) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	return detect(cv_ptr->image, out);
}

bool CheckerboardDetection::detect(const cv::Mat& image,
		CheckerboardData& out) {

	// Detect corner using OpenCV.
	cv::Mat gray;
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	corners.clear(); //this will be filled by the detected corners

    ROS_DEBUG("Trying to detect a checkerboard with %dx%d corners", this->patternSize.width, this->patternSize.height);
    bool patternfound = kinematic_calibration::findChessboardCorners(gray,
			this->patternSize, corners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
					+ cv::CALIB_CB_FAST_CHECK);

	if (patternfound) {
		cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(gray, this->patternSize, cv::Mat(corners), patternfound);
        if (debug)
        {
            cv::imshow("CHESS_DETECTOR", gray);
            cv::waitKey(1);
        }
	} else {
        //std::cout << "No pattern found." << std::endl;
        ROS_DEBUG("Not checkerboard found");
		return false;
	}

	/*if (this->patternSize.height == 3 && this->patternSize.width == 3) {
	 // "legacy" case
	 cv::Point2f position = corners[4];
	 out.x = position.x;
	 out.y = position.y;
     } else*/ if (this->patternSize.height % 2 == 1
			&& this->patternSize.width % 2 == 1) {
		// even outer pattern size: take the middle point
		int index = ((int) (patternSize.height / 2)) * patternSize.width
				+ ((int) (patternSize.width / 2));
		cv::Point2f position = corners[index];
		out.x = position.x;
		out.y = position.y;
	} else {
        // uneven outer pattern size: compute center of all points
		out.x = 0.0;
		out.y = 0.0;
		for (int i = 0; i < corners.size(); i++) {
			out.x += corners[i].x;
			out.y += corners[i].y;
		}
		out.x = out.x / corners.size();
		out.y = out.y / corners.size();
	}

	return true;
}

bool CheckerboardDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	CheckerboardData cb;
	out.resize(4);
	if (detect(in_msg, cb)) {
		savePosition(out);
		saveImage(in_msg);
		out[idx_x] = cb.x;
		out[idx_y] = cb.y;
		out[idx_rows] = this->patternSize.height;
		out[idx_cols] = this->patternSize.width;
		for (int i = 0; i < corners.size(); i++) {
			out.push_back(corners[i].x);
			out.push_back(corners[i].y);
		}
		out.push_back(this->squareLength); // square length
		return true;
	}
	return false;
}

bool CheckerboardDetection::setCheckerboardSize(int rows, int columns) {
	// just delegate...
	return setCheckerboardInnerSize(rows - 1, columns - 1);
}

bool CheckerboardDetection::setCheckerboardInnerSize(int rows, int columns) {
	// size must be >= 2x2
	if (rows < 2 || columns < 2)
		return false;

	this->patternSize = cv::Size(rows, columns);
	return true;
}

void CheckerboardDetection::setSquareLength(const double& squareLength) {
	this->squareLength = squareLength;
}

RosCheckerboardDetection::RosCheckerboardDetection(double maxDist) :
		transformListener(ros::Duration(15, 0)), maxDist(maxDist) {
	nh.getParam("marker_frame", this->markerFrame);

	// get camera information
	camerainfoSub = nh.subscribe("/nao_camera/camera_info", 1,
			&RosCheckerboardDetection::camerainfoCallback, this);
	cameraInfoSet = false;
	ROS_INFO("Waiting for camera info message...");
	while (!cameraInfoSet) {
		ros::spinOnce();
	}
	camerainfoSub.shutdown();
	ROS_INFO("Done.");

	ROS_INFO("Filling tf buffer...");
	usleep(1e6);
	ROS_INFO("Done.");
}

RosCheckerboardDetection::~RosCheckerboardDetection() {
	// nothing to do
}

bool RosCheckerboardDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	bool success;
	success = CheckerboardDetection::detect(in_msg, out);

	if (!success) {
		// no checkerboard found at all
		return success;
	}

	if (markerFrame == "") {
		// no marker frame given
		ROS_INFO(
				"No marker_frame parameter set! No automatic removal of outlier!");
		return success;
	}

	// marker frame is given, check distance to predicted position
	string cameraFrame = in_msg->header.frame_id;
	ros::Time now = ros::Time::now();
	tf::StampedTransform transform;
	cv::Point2d center;
	// get the transformation via TF
	if (!transformListener.waitForTransform(cameraFrame, markerFrame, now,
			ros::Duration(1.0))) {
		ROS_INFO("No transformation found from %s to %s!", markerFrame.c_str(),
				cameraFrame.c_str());
		return success;
	}
	transformListener.lookupTransform(cameraFrame, markerFrame, now, transform);
	// get the projected center
	FrameImageConverter fic(cameraModel);
	fic.project(transform, center.x, center.y);

	ROS_INFO("Expecting the checkerboard near %f %f, found it at %f %f",
			center.x, center.y, out[idx_x], out[idx_y]);

	double deltaX = center.x - out[idx_x];
	double deltaY = center.y - out[idx_y];
	double dist = sqrt(deltaX * deltaX - deltaY * deltaY);

	if (dist > maxDist) {
		ROS_INFO("Too far away, probably an outlier.");
		return false;
	}

	return success;
}

void RosCheckerboardDetection::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	cameraModel.fromCameraInfo(msg);
	ROS_INFO("Camera model set.");
	cameraInfoSet = true;
}

} /* namespace kinematic_calibration */
