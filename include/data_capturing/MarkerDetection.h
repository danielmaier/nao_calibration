/*
 * MarkerDetection.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef MARKERDETECTION_H_
#define MARKERDETECTION_H_

#include <sensor_msgs/Image.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace kinematic_calibration {

using namespace std;

/**
 * Abstract base class for marker detection.
 */
class MarkerDetection {
public:
	MarkerDetection();
	virtual ~MarkerDetection();

	/**
	 * Tries to detect a marker.
	 * @param[in] in_msg image message containing the marker
	 * @param[out] out data describing the found marker
	 * @return true if the marker was detected otherwise false
	 */
	virtual bool detect(const sensor_msgs::ImageConstPtr& in_msg, vector<double>& out) = 0;

	/**
	 * Writes the last image with the detected marker to the hard disk.
	 * @param filename name of the file to be saved
	 * @return true if success otherwise false;
	 */
	virtual bool writeImage(const string& filename);

protected:
	/**
	 * Draws the detected marker into the image.
	 * @param image
	 */
	virtual void drawMarker(cv::Mat& image) = 0;

	/**
	 * Returns the last image.
	 * @param image the last image
	 */
	virtual void getImage(cv::Mat& image) = 0;

	/**
	 * Converts the ROS image message to a OpenCV image.
	 * @param[in] in_msg The ROS image message to convert.
	 * @param[out] out_image The resulting OpenCV image.
	 */
	void msgToImg(const sensor_msgs::ImageConstPtr& in_msg, cv::Mat& out_image);
};

} /* namespace kinematic_calibration */

#endif /* MARKERDETECTION_H_ */
