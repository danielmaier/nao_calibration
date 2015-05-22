/*
 * ErrorModel.cpp
 *
 *  Created on: 24.02.2014
 *      Author: stefan
 */

#include "../../include/common/ErrorModel.h"

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <map>
#include <string>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/common/MeasurementPose.h"
#include <optimization/CalibrationState.h>
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/optimization/TransformationVertex.h"

#include <fenv.h>
#pragma STDC FENV_ACCESS ON

namespace kinematic_calibration {

ErrorModel::ErrorModel(KinematicChain& kinematicChain) :
		kinematicChain(kinematicChain) {
}

ErrorModel::~ErrorModel() {

}

void ErrorModel::getImageCoordinates(const CalibrationState& state,
		const measurementData& measurement, double& x, double& y) {
    MeasurementPose pose(kinematicChain, measurement.jointState, "");
	pose.predictImageCoordinates(state, x, y);
}




void ErrorModel::appendVector(vector<double>& v1, const vector<double>& v2) {
	v1.insert(v1.end(), v2.begin(), v2.end());
}


SinglePointErrorModel::SinglePointErrorModel(KinematicChain& kinematicChain) :
		ErrorModel(kinematicChain) {
}

SinglePointErrorModel::~SinglePointErrorModel() {
}

void SinglePointErrorModel::getError(const CalibrationState& state,
		const measurementData& measurement, vector<double>& delta) {
	double x, y;
	this->getImageCoordinates(state, measurement, x, y);
	delta.clear();
	delta.push_back((x - measurement.marker_data[0]));
	delta.push_back((y - measurement.marker_data[1]));
}

void SinglePointErrorModel::getSquaredError(
		const CalibrationState& state,
		const measurementData& measurement, vector<double>& error) {
	vector<double> delta;
	this->getError(state, measurement, delta);
	error.push_back(delta[0] * delta[0]);
	error.push_back(delta[1] * delta[1]);
}

CircleErrorModel::CircleErrorModel(KinematicChain& kinematicChain) :
		ErrorModel(kinematicChain) {
}

CircleErrorModel::~CircleErrorModel() {
}

void CircleErrorModel::getError(const CalibrationState& state,
		const measurementData& measurement, vector<double>& delta) {
}

void CircleErrorModel::getSquaredError(const CalibrationState& state,
		const measurementData& measurement, vector<double>& error) {
}

} /* namespace kinematic_calibration */
