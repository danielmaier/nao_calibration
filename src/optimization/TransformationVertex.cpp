/*
 * TransformationVertex.cpp
 *
 *  Created on: 27.02.2014
 *      Author: stefan
 */

#include "../../include/optimization/TransformationVertex.h"

namespace kinematic_calibration {

TransformationVertex::TransformationVertex() :
		BaseVertex<6, tf::Transform>() {

}

TransformationVertex::~TransformationVertex() {
	// TODO Auto-generated destructor stub
}

bool TransformationVertex::read(std::istream& in) {
	return false;
}

bool TransformationVertex::write(std::ostream& out) const {
	return false;
}

void TransformationVertex::oplusImpl(const double* delta) {
	// calculate new translation
	double tx_delta = delta[0];
	double ty_delta = delta[1];
	double tz_delta = delta[2];

	double tx_old = this->_estimate.getOrigin()[0];
	double ty_old = this->_estimate.getOrigin()[1];
	double tz_old = this->_estimate.getOrigin()[2];

	tf::Vector3 t_new(tx_old + tx_delta, ty_old + ty_delta, tz_old + tz_delta);

	tf::Quaternion quat_old = this->_estimate.getRotation();

	// calculate new r, p, y
	double roll_delta = delta[3];
	double pitch_delta = delta[4];
	double yaw_delta = delta[5];

	double roll_old, pitch_old, yaw_old;
	tf::Matrix3x3(quat_old).getRPY(roll_old, pitch_old, yaw_old);
	tf::Quaternion quat_new;
	quat_new.setRPY(roll_old + roll_delta, pitch_old + pitch_delta,
			yaw_old + yaw_delta);

	tf::Transform newTransform(quat_new, t_new);
	this->setEstimate(newTransform);
}

void TransformationVertex::setToOriginImpl() {
	this->_estimate = tf::Transform();
}

} /* namespace kinematic_calibration */
