/*
 * ColorMarkerDetection.h
 *
 *  Created on: 02.04.2014
 *      Author: stefan
 */

#ifndef COLORMARKERDETECTION_H_
#define COLORMARKERDETECTION_H_

#include <sensor_msgs/Image.h>
#include <vector>

#include "SinglePointMarkerDetection.h"
#include "color_marker_detection/CMDetect.hpp"

namespace kinematic_calibration {

using namespace std;

/**
 * Class for detection of the color marker.
 */
class ColorMarkerDetection: public SinglePointMarkerDetection {
public:
	ColorMarkerDetection(string userParsFileName, string classParsFileName);
	virtual ~ColorMarkerDetection();

	virtual bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			vector<double>& out);

	bool detect(const cv::Mat& image, vector<double>& out);

protected:
	void drawMarker(cv::Mat& image);

private:
	CMDetect cmdetect;
};

} /* namespace kinematic_calibration */

#endif /* COLORMARKERDETECTION_H_ */
