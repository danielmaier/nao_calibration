/*
 * CheckerboardMeasurementEdge.cpp
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#include "../../include/optimization/CheckerboardMeasurementEdge.h"

#include <tf/tf.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <utility>
#include <vector>

#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/data_capturing/CheckerboardDetection.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

namespace kinematic_calibration {

CheckerboardMeasurementEdge::CheckerboardMeasurementEdge(
		measurementData measurement, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) :
		MeasurementEdge<2, CheckerboardMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain) {

}

CheckerboardMeasurementEdge::~CheckerboardMeasurementEdge() {
}

void CheckerboardMeasurementEdge::setError(tf::Transform cameraToMarker) {
	double x, y;
	this->frameImageConverter->project(cameraToMarker.inverse(), x, y);

	double dist = cameraToMarker.getOrigin().length();

	// set error
	this->_error[0] = measurement.marker_data[0] - x;
	this->_error[1] = measurement.marker_data[1] - y;
}

/*
void CheckerboardMeasurementEdge::getLinearization(){
    return _jacobianOplus[0];
}
*/

CheckerboardPoseMeasurementEdge::CheckerboardPoseMeasurementEdge(
		measurementData measurement, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) :
		MeasurementEdge<6, CheckerboardPoseMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain) {
}

CheckerboardPoseMeasurementEdge::~CheckerboardPoseMeasurementEdge() {
}

void CheckerboardPoseMeasurementEdge::setError(tf::Transform cameraToMarker) {
	// left top point of the marker denotes the origin of the marker's coordinate system
	double xOrigin, yOrigin;
	this->frameImageConverter->project(cameraToMarker.inverse(), xOrigin,
			yOrigin);

	double rows = this->measurement.marker_data[CheckerboardDetection::idx_rows];
	double cols = this->measurement.marker_data[CheckerboardDetection::idx_cols];
	double squareLength =
			this->measurement.marker_data[measurement.marker_data.size() - 1]; // always the last element
	squareLength = squareLength / 1000; // conversion from [mm] to [m]

	// point on the x axis of the marker's coordinate system
	double xAxisX, yAxisX;
	tf::Transform alongXAxis(tf::Quaternion(0, 0, 0, 1),
			tf::Vector3(squareLength, 0, 0));
	this->frameImageConverter->project((alongXAxis * cameraToMarker).inverse(),
			xAxisX, yAxisX);

	// point on the y axis of the marker's coordinate system
	double xAxisY, yAxisY;
	tf::Transform alongYAxis(tf::Quaternion(0, 0, 0, 1),
			tf::Vector3(0, squareLength, 0));
	this->frameImageConverter->project((alongYAxis * cameraToMarker).inverse(),
			xAxisY, yAxisY);

	// set error
	double offset = 4;

	// top left corner
	this->_error[0] = measurement.marker_data[offset + 0 + 0] - xOrigin;
	this->_error[1] = measurement.marker_data[offset + 0 + 1] - yOrigin;

	// one right from the top left corner
	this->_error[2] = measurement.marker_data[offset + (2 * 1) + 0] - xAxisX;
	this->_error[3] = measurement.marker_data[offset + (2 * 1) + 1] - yAxisX;

	// one down from the top left corner
	this->_error[4] = measurement.marker_data[offset + (2 * cols) + 0] - xAxisY;
	this->_error[5] = measurement.marker_data[offset + (2 * cols) + 1] - yAxisY;

	if (false) {
		cv::Mat outImg;
		cv_bridge::CvImagePtr input_bridge;
		input_bridge = cv_bridge::toCvCopy(measurement.image,
				sensor_msgs::image_encodings::BGR8);
		outImg = input_bridge->image;

		cv::line(outImg, cv::Point(xOrigin - 3, yOrigin - 3),
				cv::Point(xOrigin + 3, yOrigin + 3), CV_RGB(255, 0, 0));
		cv::line(outImg, cv::Point(xOrigin - 3, yOrigin + 3),
				cv::Point(xOrigin + 3, yOrigin - 3), CV_RGB(255, 0, 0));
		cv::circle(outImg,
				cv::Point(measurement.marker_data[offset],
						measurement.marker_data[offset + 1]), 3,
				CV_RGB(255, 0, 0));
		cv::line(outImg, cv::Point(xAxisX - 3, yAxisX - 3),
				cv::Point(xAxisX + 3, yAxisX + 3), CV_RGB(0, 255, 0));
		cv::line(outImg, cv::Point(xAxisX - 3, yAxisX + 3),
				cv::Point(xAxisX + 3, yAxisX - 3), CV_RGB(0, 255, 0));
		cv::circle(outImg,
				cv::Point(measurement.marker_data[offset + (2 * 1)],
						measurement.marker_data[offset + (2 * 1) + 1]), 3,
				CV_RGB(0, 255, 0));
		cv::line(outImg, cv::Point(xAxisY - 3, yAxisY - 3),
				cv::Point(xAxisY + 3, yAxisY + 3), CV_RGB(0, 0, 255));
		cv::line(outImg, cv::Point(xAxisY - 3, yAxisY + 3),
				cv::Point(xAxisY + 3, yAxisY - 3), CV_RGB(0, 0, 255));
		cv::circle(outImg,
				cv::Point(measurement.marker_data[offset + (2 * cols)],
						measurement.marker_data[offset + (2 * cols) + 1]), 3,
				CV_RGB(0, 0, 255));
		string filename = "/tmp/" + measurement.id + "-pose.jpg";
		cv::imwrite(filename, outImg);
	}
}

ArucoPoseMeasurementEdge::ArucoPoseMeasurementEdge(
		measurementData measurement, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) :
		MeasurementEdge<6, ArucoPoseMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain) {
}

ArucoPoseMeasurementEdge::~ArucoPoseMeasurementEdge() {
}

void ArucoPoseMeasurementEdge::setError(tf::Transform cameraToMarker) {
	// left top point of the marker denotes the origin of the marker's coordinate system
	double xOrigin, yOrigin;
	this->frameImageConverter->project(cameraToMarker.inverse(), xOrigin,
			yOrigin);

	double squareLength =
			this->measurement.marker_data[measurement.marker_data.size() - 1]; // always the last element
	squareLength = squareLength / 1000; // conversion from [mm] to [m]

	// point on the x axis of the marker's coordinate system
	double xAxisX, yAxisX;
	tf::Transform alongXAxis(tf::Quaternion(0, 0, 0, 1),
			tf::Vector3(squareLength / 2, - squareLength / 2, 0));
	this->frameImageConverter->project((alongXAxis * cameraToMarker).inverse(),
			xAxisX, yAxisX);

	// point on the y axis of the marker's coordinate system
	double xAxisY, yAxisY;
	tf::Transform alongYAxis(tf::Quaternion(0, 0, 0, 1),
			tf::Vector3(squareLength / 2, squareLength / 2, 0));
	this->frameImageConverter->project((alongYAxis * cameraToMarker).inverse(),
			xAxisY, yAxisY);

	// set error
	double offset = 2;

	// center (r)
	this->_error[0] = measurement.marker_data[0] - xOrigin;
	this->_error[1] = measurement.marker_data[1] - yOrigin;

	// top right (g)
	this->_error[2] = measurement.marker_data[offset + 0 + 0] - xAxisX;
	this->_error[3] = measurement.marker_data[offset + 0 + 1] - yAxisX;

	// bottom right (b)
	this->_error[4] = measurement.marker_data[offset + 6 + 0] - xAxisY;
	this->_error[5] = measurement.marker_data[offset + 6 + 1] - yAxisY;

	if (false) {
		cv::Mat outImg;
		cv_bridge::CvImagePtr input_bridge;
		input_bridge = cv_bridge::toCvCopy(measurement.image,
				sensor_msgs::image_encodings::BGR8);
		outImg = input_bridge->image;

		cv::line(outImg, cv::Point(xOrigin - 3, yOrigin - 3),
				cv::Point(xOrigin + 3, yOrigin + 3), CV_RGB(255, 0, 0));
		cv::line(outImg, cv::Point(xOrigin - 3, yOrigin + 3),
				cv::Point(xOrigin + 3, yOrigin - 3), CV_RGB(255, 0, 0));
		cv::circle(outImg,
				cv::Point(measurement.marker_data[0],
						measurement.marker_data[1]), 3,
				CV_RGB(255, 0, 0));
		cv::line(outImg, cv::Point(xAxisX - 3, yAxisX - 3),
				cv::Point(xAxisX + 3, yAxisX + 3), CV_RGB(0, 255, 0));
		cv::line(outImg, cv::Point(xAxisX - 3, yAxisX + 3),
				cv::Point(xAxisX + 3, yAxisX - 3), CV_RGB(0, 255, 0));
		cv::circle(outImg,
				cv::Point(measurement.marker_data[offset + 0],
						measurement.marker_data[offset + 0 + 1]), 3,
				CV_RGB(0, 255, 0));
		cv::line(outImg, cv::Point(xAxisY - 3, yAxisY - 3),
				cv::Point(xAxisY + 3, yAxisY + 3), CV_RGB(0, 0, 255));
		cv::line(outImg, cv::Point(xAxisY - 3, yAxisY + 3),
				cv::Point(xAxisY + 3, yAxisY - 3), CV_RGB(0, 0, 255));
		cv::circle(outImg,
				cv::Point(measurement.marker_data[offset + 6],
						measurement.marker_data[offset + 6 + 1]), 3,
				CV_RGB(0, 0, 255));
		string filename = "/tmp/" + measurement.id + "-pose.jpg";
		cv::imwrite(filename, outImg);
	}
}

} /* namespace kinematic_calibration */

