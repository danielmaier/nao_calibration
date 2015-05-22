/*
 * ArucoMarkerDetection.h
 *
 *  Created on: 11.04.2014
 *      Author: stefan
 */

#ifndef ARUCOMARKERDETECTION_H_
#define ARUCOMARKERDETECTION_H_

#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <vector>

#include "aruco/marker.h"
#include "SinglePointMarkerDetection.h"

namespace kinematic_calibration {

using namespace std;

/**
 * Class for detecting markers using the ArUco library.
 * Further informations: http://www.uco.es/investiga/grupos/ava/node/26
 */
class ArucoMarkerDetection : public SinglePointMarkerDetection {
public:
	ArucoMarkerDetection();
	virtual ~ArucoMarkerDetection();

	bool detect(const sensor_msgs::ImageConstPtr& in_msg, vector<double>& out);
	bool detect(const cv::Mat& inImage, vector<double>& out);
	void drawMarker(cv::Mat& image);

	void setMarkerSize(const double& markerSize);

private:
	aruco::Marker currentMarker;

	/**
	 * The side length of the aruco marker.
	 */
	double markerSize;
};

} /* namespace kinematic_calibration */

#endif /* ARUCOMARKERDETECTION_H_ */
