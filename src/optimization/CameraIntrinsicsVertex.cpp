/*
 * CameraIntrinsicsVertex.cpp
 *
 *  Created on: 06.12.2013
 *      Author: stefan
 */

#include "../../include/optimization/CameraIntrinsicsVertex.h"

#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <string>

namespace kinematic_calibration {

CameraIntrinsicsVertex::CameraIntrinsicsVertex(
		sensor_msgs::CameraInfo cameraInfo) :
		initial(cameraInfo) {
	if ("plumb_bob" != cameraInfo.distortion_model) {
		ROS_FATAL("Distortion model should be plumb_bob!");
	}
	_estimate = cameraInfo;
}

CameraIntrinsicsVertex::~CameraIntrinsicsVertex() {
}

void CameraIntrinsicsVertex::oplusImpl(const double* delta) {
	//     [fx  0 cx]
	// K = [ 0 fy cy]
	//     [ 0  0  1]
	this->_estimate.K[K_FX_IDX] += delta[0];
	this->_estimate.K[K_FY_IDX] += delta[1];
	this->_estimate.K[K_CX_IDX] += delta[2];
	this->_estimate.K[K_CY_IDX] += delta[3];

	// P[1:3,1:3] = K
	this->_estimate.P[P_FX_IDX] += delta[0];
	this->_estimate.P[P_FY_IDX] += delta[1];
	this->_estimate.P[P_CX_IDX] += delta[2];
	this->_estimate.P[P_CY_IDX] += delta[3];

	// (k1, k2, t1, t2, k3)
	this->_estimate.D[0] += delta[4];
//	this->_estimate.D[1] += delta[5];
//	this->_estimate.D[2] += delta[6];
//	this->_estimate.D[3] += delta[7];
//	this->_estimate.D[4] += delta[8];
}

void CameraIntrinsicsVertex::setToOriginImpl() {
	this->_estimate = initial;

	// set distortion parameters to zero
//	this->_estimate.D[0] = 0;
	this->_estimate.D[1] = 0;
	this->_estimate.D[2] = 0;
	this->_estimate.D[3] = 0;
	this->_estimate.D[4] = 0;
}

} /* namespace kinematic_calibration */
