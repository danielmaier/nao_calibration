/*
 * CircleDetection.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/CircleDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>

#include "../../include/common/FrameImageConverter.h"

#include <vector>

namespace kinematic_calibration {

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

CircleDetection::CircleDetection(Type type) :
		type(type) {
	// TODO Auto-generated constructor stub

}

CircleDetection::~CircleDetection() {
	// TODO Auto-generated destructor stub
}

bool CircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		const cv::Scalar& color, vector<double>& out) {
	// Convert from ROS message to OpenCV image.
	cv::Mat image;
	msgToImg(in_msg, image);

	// delegate call
	return detect(image, color, out);
}

bool CircleDetection::detect(const cv::Mat& image, const cv::Scalar& color,
		vector<double>& out) {
	cv::Mat imageCopy = image.clone();

	bool success;
	vector<cv::Vec3f> circles;
	success = detect(imageCopy, circles);
	if (success) {
		return findClosestColor(image, circles, color, out);
	}
	return success;
}

bool CircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<cv::Vec3f>& out) {
	// Convert from ROS message to OpenCV image.
	cv::Mat image;
	msgToImg(in_msg, image);

	// delegate call
	return detect(image, out);
}

void CircleDetection::processImage(const cv::Mat& in, cv::Mat& out,
		double cannyTreshold) {
	cv::Mat img;
	in.copyTo(img);
	cv::cvtColor(img, out, CV_BGR2GRAY);

	GaussianBlur(out, out, cv::Size(9, 9), 2, 2);

	double treshold1 = cannyTreshold; //150
	double treshold2 = cannyTreshold / 2; //50
	int apertureSize = 3; //7
	cv::Canny(out, out, treshold1, treshold2, apertureSize);
	this->cannyImg = out.clone();

	GaussianBlur(out, out, cv::Size(9, 9), 2, 2);
	this->gaussImg = out.clone();
}

bool CircleDetection::detect(const cv::Mat& image, vector<cv::Vec3f>& out) {
	if (type == Ransac) {
		NodeHandle nh;
		double percentage;
		int iterations;
		int canny;
		nh.param("percentage", percentage, 0.9); // TODO: namespace?!
		nh.param("iterations", iterations, 1000); // TODO: namespace?!
		nh.param("canny", canny, 200); // TODO: namespace?!
		return circlesRansac(image, out, canny, percentage, iterations);
	} else if (type == HoughTransform) {
		return circlesHoughTransform(image, out);
	} else if (type == HoughTransformAdaptive) {
		return circlesHoughTransformAdaptive(image, out);
	} else if (type == Dummy) {
		return circlesDummy(image, out);
	}
	return false;
}

bool CircleDetection::circlesHoughTransform(const cv::Mat& image,
		vector<cv::Vec3f>& out) {
	// process image
	cv::Mat gray;
	processImage(image, gray);

	// detect circles
	double dp = 2;
	double min_dist = 50;
	double param1 = 70;
	double param2 = 70;
	int min_radius = 20;
	int max_radius = 100;

	cv::HoughCircles(gray, out, CV_HOUGH_GRADIENT, dp, min_dist, param1, param2,
			min_radius, max_radius);
	return out.size() > 0;
}

bool CircleDetection::circlesHoughTransformAdaptive(const cv::Mat& image,
		vector<cv::Vec3f>& out) {
	for (int cannyTreshold = 50; cannyTreshold < 300; cannyTreshold += 20) {
		// process image
		cv::Mat gray;
		processImage(image, gray, cannyTreshold);

		vector<cv::Vec3f> addOut;
		for (double param2 = 50; param2 < 150; param2 += 20) {
			// detect circles
			double dp = 2;
			double min_dist = 50;
			double param1 = cannyTreshold; //70;
			//double param2 = 70;
			int min_radius = 20;
			int max_radius = 100;


			vector<cv::Vec3f> tmpOut;
			cv::HoughCircles(gray, tmpOut, CV_HOUGH_GRADIENT, dp, min_dist,
					param1, param2, min_radius, max_radius);

			if(tmpOut.size() > 0) {
				// save current results
				addOut = tmpOut;
			} else {
				// value too high
				break;
			}

			if (false) {
				cout << "Canny: " << cannyTreshold << "\tDetected "
						<< tmpOut.size() << " circles." << " Total so far: "
						<< out.size() << std::endl;
				Mat debugImg;
				image.copyTo(debugImg);
				for (unsigned int i = 0; i < tmpOut.size(); ++i) {
					cv::Point center(cvRound(tmpOut[i][0]),
							cvRound(tmpOut[i][1]));
					int radius = cvRound(tmpOut[i][2]);
					// circle center
					cv::circle(debugImg, center, 3, cv::Scalar(0, 0, 255), -1,
							8, 0);
					// circle outline
					cv::circle(debugImg, center, radius, cv::Scalar(0, 0, 255),
							3, 8, 0);
				}
				imshow("canny debug", debugImg);
				waitKey(0);
			}
		}
		// append current results
		out.insert(out.end(), addOut.begin(), addOut.end());
	}
	return out.size() > 0;
}

bool CircleDetection::circlesRansac(const cv::Mat& inimg,
		vector<cv::Vec3f>& circles, double canny_threshold,
		double circle_threshold, int numIterations) {
	// from https://github.com/pickle27/circleDetector/blob/master/circleDetector.cpp
	cv::Mat image;
	cv::cvtColor(inimg, image, CV_BGR2GRAY);

	CV_Assert(image.type() == CV_8UC1 || image.type() == CV_8UC3);
	circles.clear();

	GaussianBlur(image, image, cv::Size(9, 9), 2, 2);

	// Edge Detection
	Mat edges;
	Canny(image, edges, MAX(canny_threshold / 2, 1), canny_threshold, 3);
	this->cannyImg = edges.clone();

	//GaussianBlur(edges, edges, cv::Size(3, 3), 2, 2);
	threshold(edges, edges, 1, 255, CV_THRESH_BINARY);
	this->gaussImg = edges.clone();

	// Create point set from Canny Output
	std::vector<Point2d> points;
	for (int r = 0; r < edges.rows; r++) {
		for (int c = 0; c < edges.cols; c++) {
			if (edges.at<unsigned char>(r, c) == 255) {
				points.push_back(cv::Point2d(c, r));
			}
		}
	}

	// 4 point objects to hold the random samples
	Point2d pointA;
	Point2d pointB;
	Point2d pointC;
	Point2d pointD;

	// distances between points
	double AB;
	double BC;
	double CA;
	double DC;

	// varibales for line equations y = mx + b
	double m_AB;
	double b_AB;
	double m_BC;
	double b_BC;

	// varibles for line midpoints
	double XmidPoint_AB;
	double YmidPoint_AB;
	double XmidPoint_BC;
	double YmidPoint_BC;

	// variables for perpendicular bisectors
	double m2_AB;
	double m2_BC;
	double b2_AB;
	double b2_BC;

	// RANSAC
	cv::RNG rng;
	int min_point_separation = 10; // change to be relative to image size?
	int colinear_tolerance = 1; // make sure points are not on a line
	int radius_tolerance = 3; // change to be relative to image size?
	int points_threshold = 10; //should always be greater than 4
	//double min_circle_separation = 10; //reject a circle if it is too close to a previously found circle
	//double min_radius = 10.0; //minimum radius for a circle to not be rejected

	int x, y;
	Point2d center;
	double radius;

	// Iterate
	for (int iteration = 0; iteration < numIterations; iteration++) {
		//std::cout << "RANSAC iteration: " << iteration << std::endl;

		// get 4 random points
		pointA = points[rng.uniform((int) 0, (int) points.size())];
		pointB = points[rng.uniform((int) 0, (int) points.size())];
		pointC = points[rng.uniform((int) 0, (int) points.size())];
		pointD = points[rng.uniform((int) 0, (int) points.size())];

		// calc lines
		AB = norm(pointA - pointB);
		BC = norm(pointB - pointC);
		CA = norm(pointC - pointA);
		DC = norm(pointD - pointC);

		// one or more random points are too close together
		if (AB < min_point_separation || BC < min_point_separation
				|| CA < min_point_separation || DC < min_point_separation)
			continue;

		//find line equations for AB and BC
		//AB
		m_AB = (pointB.y - pointA.y) / (pointB.x - pointA.x + 0.000000001); //avoid divide by 0
		b_AB = pointB.y - m_AB * pointB.x;

		//BC
		m_BC = (pointC.y - pointB.y) / (pointC.x - pointB.x + 0.000000001); //avoid divide by 0
		b_BC = pointC.y - m_BC * pointC.x;

		//test colinearity (ie the points are not all on the same line)
		if (abs(pointC.y - (m_AB * pointC.x + b_AB + colinear_tolerance))
				< colinear_tolerance)
			continue;

		//find perpendicular bisector
		//AB
		//midpoint
		XmidPoint_AB = (pointB.x + pointA.x) / 2.0;
		YmidPoint_AB = m_AB * XmidPoint_AB + b_AB;
		//perpendicular slope
		m2_AB = -1.0 / m_AB;
		//find b2
		b2_AB = YmidPoint_AB - m2_AB * XmidPoint_AB;

		//BC
		//midpoint
		XmidPoint_BC = (pointC.x + pointB.x) / 2.0;
		YmidPoint_BC = m_BC * XmidPoint_BC + b_BC;
		//perpendicular slope
		m2_BC = -1.0 / m_BC;
		//find b2
		b2_BC = YmidPoint_BC - m2_BC * XmidPoint_BC;

		//find intersection = circle center
		x = (b2_AB - b2_BC) / (m2_BC - m2_AB);
		y = m2_AB * x + b2_AB;
		center = Point2d(x, y);
		radius = cv::norm(center - pointB);

		/// geometry debug image
		if (false) {
			Mat debug_image = edges.clone();
			cvtColor(debug_image, debug_image, CV_GRAY2RGB);

			Scalar pink(255, 0, 255);
			Scalar blue(255, 0, 0);
			Scalar green(0, 255, 0);
			Scalar yellow(0, 255, 255);
			Scalar red(0, 0, 255);

			// the 3 points from which the circle is calculated in pink
			circle(debug_image, pointA, 3, pink);
			circle(debug_image, pointB, 3, pink);
			circle(debug_image, pointC, 3, pink);

			// the 2 lines (blue) and the perpendicular bisectors (green)
			line(debug_image, pointA, pointB, blue);
			line(debug_image, pointB, pointC, blue);
			line(debug_image, Point(XmidPoint_AB, YmidPoint_AB), center, green);
			line(debug_image, Point(XmidPoint_BC, YmidPoint_BC), center, green);

			circle(debug_image, center, 3, yellow); // center
			circle(debug_image, center, radius, yellow); // circle

			// 4th point check
			circle(debug_image, pointD, 3, red);

			imshow("ransac debug", debug_image);
			waitKey(0);
		}

		//check if the 4 point is on the circle
		if (abs(cv::norm(pointD - center) - radius) > radius_tolerance)
			continue;

		// vote
		std::vector<int> votes;
		std::vector<int> no_votes;
		for (int i = 0; i < (int) points.size(); i++) {
			double vote_radius = norm(points[i] - center);

			if (abs(vote_radius - radius) < radius_tolerance) {
				votes.push_back(i);
			} else {
				no_votes.push_back(i);
			}
		}

		// check votes vs circle_threshold
		if ((float) votes.size() / (2.0 * CV_PI * radius) >= circle_threshold) {
			circles.push_back(Vec3f(x, y, radius));

			if (true) {
				Mat debug_image2 = edges.clone();
				cvtColor(debug_image2, debug_image2, CV_GRAY2RGB);

				Scalar yellow(0, 255, 255);
				Scalar green(0, 255, 0);

				circle(debug_image2, center, 3, yellow); // center
				circle(debug_image2, center, radius, yellow); // circle

				// draw points that voted
				for (int i = 0; i < (int) votes.size(); i++) {
					circle(debug_image2, points[votes[i]], 1, green);
				}

				imshow("ransac debug", debug_image2);
				waitKey(0);
			}

			// remove points from the set so they can't vote on multiple circles
			std::vector<Point2d> new_points;
			for (int i = 0; i < (int) no_votes.size(); i++) {
				new_points.push_back(points[no_votes[i]]);
			}
			points.clear();
			points = new_points;
		}

		// stop RANSAC if there are few points left
		if ((int) points.size() < points_threshold)
			break;
	}
	return circles.size() > 0;
}

bool CircleDetection::circlesDummy(const cv::Mat& inimg,
		vector<cv::Vec3f>& circles) {
	// fill with dummy values
	circles.clear();
	circles.push_back(cv::Vec3f(320,200, 0.1));
	return true;
}

void CircleDetection::msgToImg(const sensor_msgs::ImageConstPtr& in_msg,
		cv::Mat& out_image) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	out_image = cv_ptr->image;
}

bool CircleDetection::findClosestColor(const cv::Mat& image,
		const vector<cv::Vec3f>& circles, const cv::Scalar& color,
		vector<double>& out) {
	double distTreshold = 70;
	int minIdx = -1;
	vector<int> validIndexes;

	for (int i = 0; i < circles.size(); i++) {
		cv::Vec3f currentCircle = circles[i];
		int x = cvRound(currentCircle[0]);
		int y = cvRound(currentCircle[1]);
		int value_b = image.ptr<uchar>(y)[3 * x + 0];
		int value_g = image.ptr<uchar>(y)[3 * x + 1];
		int value_r = image.ptr<uchar>(y)[3 * x + 2];
		double dist = fabs(value_r - color[0]) + fabs(value_g - color[1])
				+ fabs(value_b - color[2]);
		if (dist < distTreshold) {
			validIndexes.push_back(i);
		}
	}

	// check whether we found a circle that matches almost the color
	if (validIndexes.size() <= 0) {
		return false;
	}

	double maxRadius = 0;
	for (int i = 0; i < validIndexes.size(); i++) {
		if (circles[validIndexes[i]][2] > maxRadius) {
			maxRadius = circles[validIndexes[i]][2];
			minIdx = i;
		}
	}

	out.resize(3);
	out[idx_x] = circles[minIdx][0];
	out[idx_y] = circles[minIdx][1];
	out[idx_r] = circles[minIdx][2];
	return true;

}

RosCircleDetection::RosCircleDetection(Type type) :
		CircleDetection(type), transformListener(ros::Duration(15, 0)) {
	double r, g, b;
	nh.getParam("marker_color_r", r);
	nh.getParam("marker_color_g", g);
	nh.getParam("marker_color_b", b);
	this->color = cv::Scalar(r, g, b);
	nh.getParam("marker_frame", this->markerFrame);

	// get camera information
	camerainfoSub = nh.subscribe("/nao_camera/camera_info", 1,
			&RosCircleDetection::camerainfoCallback, this);
	cameraInfoSet = false;
	while (!cameraInfoSet) {
		ROS_INFO("Waiting for camera info message...");
		ros::spinOnce();
	}
	camerainfoSub.shutdown();
	ROS_INFO("Done.");

	ROS_INFO("Filling tf buffer...");
	usleep(1e6);
	ROS_INFO("Done.");
}

bool RosCircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	bool success;
	if (markerFrame != "") {
		string cameraFrame = in_msg->header.frame_id;
		ros::Time now = ros::Time::now();
		tf::StampedTransform transform;
		cv::Point2d center;
		double radius;
		// get the transformation via TF
		transformListener.waitForTransform(cameraFrame, markerFrame, now,
				ros::Duration(1.0));
		transformListener.lookupTransform(cameraFrame, markerFrame, now,
				transform);
		// get the projected center
		FrameImageConverter fic(cameraModel);
		fic.project(transform, center.x, center.y);
		ROS_INFO("Looking for the circle near %f %f.", center.x, center.y);
		radius = 30; // TODO
		cv::Mat image;
		CircleDetection::msgToImg(in_msg, image);
		success = CircleDetection::detect(image, center, radius, out);
	} else {
		success = CircleDetection::detect(in_msg, color, out);
	}
	if (success) {
		saveImage(in_msg);
		savePosition(out);
	} else {
		// fill with dummy values
		out.resize(3);
	}
	return success;
}

bool CircleDetection::detect(const cv::Mat& image, const cv::Point2d& center,
		const double& radius, vector<double>& out) {
	bool success;
	vector<cv::Vec3f> circles;
	success = detect(image, circles);
	ROS_INFO("Detected %lu circles.", circles.size());
	if (success) {
		return findClosestRegion(image, circles, center, radius, out);
	}
	return success;
}

bool CircleDetection::findClosestRegion(const cv::Mat& image,
		const vector<cv::Vec3f>& circles, const cv::Point2d& center,
		const double& radius, vector<double>& out) {
	int bestIdx = -1;
	double minDist = 1e200;
	for (int i = 0; i < circles.size(); i++) {
		double currentDist = (center.x - circles[i][0])
				* (center.x - circles[i][0]);
		currentDist += (center.y - circles[i][1]) * (center.y - circles[i][1]);
		if (currentDist < minDist) {
			minDist = currentDist;
			bestIdx = i;
		}
	}

	if (bestIdx == -1)
		return false;

//if (sqrt(minDist) > radius)
//	return false;

	out.resize(3);
	out[idx_x] = circles[bestIdx][0];
	out[idx_y] = circles[bestIdx][1];
	out[idx_r] = circles[bestIdx][2];

	return true;
}

void RosCircleDetection::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	cameraModel.fromCameraInfo(msg);
	ROS_INFO("Camera model set.");
	cameraInfoSet = true;
}

AveragingCircleDetection::AveragingCircleDetection() {
}

bool AveragingCircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
// save the current image
	images.push_back(in_msg);

// remove images which are too old
	ros::Time timeNew = in_msg->header.stamp;
	ros::Duration timeSpan(1, 0);
	while ((images[0]->header.stamp + timeSpan) < timeNew) {
		images.erase(images.begin());
		gauss.erase(gauss.begin());
		canny.erase(canny.begin());
	}

// delegate to the base detect method
// which in turn calls our processImage() method
	return RosCircleDetection::detect(in_msg, out);
}

void AveragingCircleDetection::processImage(const cv::Mat& in, cv::Mat& out) {
// process the new image (gives canny and canny+gauss)
	RosCircleDetection::processImage(in, out);

// save the processed images
	canny.push_back(RosCircleDetection::getCannyImg());
	gauss.push_back(RosCircleDetection::getGaussImg());

// build the average
	cv::Mat cannyAvg, gaussAvg;
	assert(canny.size() == gauss.size());
	int size = canny.size();
	cannyAvg = canny[0] / size;
	gaussAvg = gauss[0] / size;
	for (int i = 1; i < size; i++) {
		cannyAvg += canny[i] / size;
		gaussAvg += gauss[i] / size;
	}

	cv::threshold(cannyAvg, cannyAvg, 10, 255, cv::THRESH_BINARY);
	cv::threshold(gaussAvg, gaussAvg, 10, 255, cv::THRESH_BINARY);

// save the averaged images
	this->cannyImg = cannyAvg.clone();
	this->gaussImg = gaussAvg.clone();

// return one of the averaged images
	out = gaussImg;
}

} /* namespace kinematic_calibration */

