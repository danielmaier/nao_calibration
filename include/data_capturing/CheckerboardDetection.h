/*
 * CheckerboardDetection.h
 *
 *  Created on: 23.10.2013
 *      Author: stefan
 */

#ifndef CHECKERBOARDDETECTION_H_
#define CHECKERBOARDDETECTION_H_

#include "SinglePointMarkerDetection.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <vector>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

namespace kinematic_calibration {

using namespace std;
using namespace ros;
using namespace cv;

bool findChessboardCorners( InputArray _image, Size patternSize,
                            OutputArray corners, int flags );

class CheckerboardData {
public:
	double x, y;
};

/**
 * Class for detecting the position of a 2x2 checker board.
 */
class CheckerboardDetection: public SinglePointMarkerDetection {
public:
	/**
	 * Constructor.
	 */
	CheckerboardDetection();

	/**
	 * Deconstructor.
	 */
	virtual ~CheckerboardDetection();

	/**
	 * Tries to detect the corner (center) of a 2x2 checker board pattern.
	 * @param[in] in_msg Image which contains the checker board pattern.
	 * @param[out] out Contains the position of the detected checker board.
	 * @return True if the checker board could be detected, false if not.
	 */
	bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			CheckerboardData& out);

	/**
	 * Tries to detect the corner (center) of a 2x2 checker board pattern.
	 * @param[in] in_msg Image which contains the checker board pattern.
	 * @param[out] out Contains the position of the detected checker board.
	 * @return True if the checker board could be detected, false if not.
	 */
	bool detect(const cv::Mat& image, CheckerboardData& out);

	/**
	 * Tries to detect a marker.
	 * @param[in] in_msg image message containing the marker
	 * @param[out] out data describing the found marker
	 * @return true if the marker was detected otherwise false
	 */
	bool detect(const sensor_msgs::ImageConstPtr& in_msg, vector<double>& out);

	enum Index {
		idx_x = 0, idx_y = 1, idx_rows = 2, idx_cols = 3
	};

	/**
	 * Sets the size of the checkerboard pattern.
	 * The size must be >= 3x3.
	 * @param rows Number of rows.
	 * @param columns Number of columns.
	 * @return True if valid values are passed.
	 */
	bool setCheckerboardSize(int rows, int columns);

	/**
	 * Sets the size of the checkerboard pattern in terms of inner corners.
	 * The size must be >= 2x2.
	 * @param rows Number of inner corners per row.
	 * @param columns Number of inner corners per column.
	 * @return True if valid values are passed.
	 */
	bool setCheckerboardInnerSize(int rows, int columns);

	/**
	 * Sets the side length of one square of the checkerboard pattern.
	 * @param squareLength The side length of one square of the checkerboard pattern.
	 */
	void setSquareLength(const double& squareLength);

protected:
	/**
	 * Number of inner corners per a chessboard row and column.
	 */
	cv::Size patternSize;

	/**
	 * Array of detected corners.
	 */
	std::vector<cv::Point2f> corners;

	/**
	 * Denotes the side length of one square of the checkerboard pattern.
	 */
	double squareLength;
    /**
     * debug mode enabled?
     */
    bool debug;
};

/**
 * Class for detecting the position of a 2x2 checker board.
 * Enhanced version, using TF and camera info in order to predict
 * the position of the checkerboard. Accepts only checkerboard
 * position near the predicted position.
 */
class RosCheckerboardDetection: public CheckerboardDetection {
public:
	/**
	 * Constructor.
	 * @param[in] maxDist Maximum distance of predicted and measured checkerboard position.
	 */
	RosCheckerboardDetection(double maxDist);

	/**
	 * Destructor.
	 */
	virtual ~RosCheckerboardDetection();

	/**
	 * Tries to detect the corner (center) of a 2x2 checker board pattern.
	 * Checks whether the distance of detected and predicted position is small.
	 * @param[in] in_msg Image which contains the checker board pattern.
	 * @param[out] out Contains the position of the detected checker board.
	 * @return True if the checker board could be detected, false if not.
	 */
	bool detect(const sensor_msgs::ImageConstPtr& in_msg, vector<double>& out);

protected:
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

private:
	NodeHandle nh;
	ros::Subscriber camerainfoSub;
	string markerFrame;
	double maxDist;
	bool cameraInfoSet;
	image_geometry::PinholeCameraModel cameraModel;
	tf::TransformListener transformListener;
};

} /* namespace kinematic_calibration */
#endif /* CHECKERBOARDDETECTION_H_ */
