/*
 * SinglePointMarkerDetection.h
 *
 *  Created on: 28.12.2013
 *      Author: stefan
 */

#ifndef SINGLEPOINTMARKERDETECTION_H_
#define SINGLEPOINTMARKERDETECTION_H_

#include "MarkerDetection.h"

#include <ros/ros.h>

namespace kinematic_calibration {

/**
 * Abstract base class for markers defined by a (x,y) point.
 */
class SinglePointMarkerDetection : public MarkerDetection {
public:
	SinglePointMarkerDetection();
	virtual ~SinglePointMarkerDetection();

protected:
	/**
	 * Draws the detected marker into the image.
	 * @param image
	 */
	void drawMarker(cv::Mat& image);

	/**
	 * Returns the last image.
	 * @param image the last image
	 */
	void getImage(cv::Mat& image);

	/**
	 * Saves the last image.
	 * @param image the last image
	 */
	void saveImage(cv::Mat image);

	/**
	 * Saves the last image from msg.
	 * @param in_msg the last image msg
	 */
	void saveImage(const sensor_msgs::ImageConstPtr& in_msg);

	/**
	 * Saves the last position.
	 * @param position the last position
	 */
	void savePosition(vector<double>& position);


private:
	cv::Mat image;
	vector<double> position;
};

} /* namespace kinematic_calibration */

#endif /* SINGLEPOINTMARKERDETECTION_H_ */
