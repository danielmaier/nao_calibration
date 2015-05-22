/*
 * UrdfUpdate.cpp
 *
 *  Created on: 18.12.2013
 *      Author: stefan
 */

#include "../../include/result_publishing/UrdfUpdate.h"

#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/type_traits/is_const.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>
#include <tf/tf.h>
#include <tinyxml.h>
#include <urdf/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <iterator>
#include <vector>

#include "../../include/result_publishing/CameraTransformUpdate.h"
#include "../../include/result_publishing/JointUpdate.h"

namespace kinematic_calibration {

UrdfUpdate::UrdfUpdate() {

}

UrdfUpdate::~UrdfUpdate() {

}

bool UrdfUpdate::readFromFile(const string& filename) {
	ifstream t(filename.c_str());
	if (t.fail()) {
		ROS_ERROR("Could not load the urdf from filename %s.",
				filename.c_str());
	}
	string str((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());
	this->inUrdfXmlString = str;
	this->urdfModel.initString(this->inUrdfXmlString);
	return true;
}

bool UrdfUpdate::readFromRos(const string& paramName) {
	string result;
	if (!nh.getParam(paramName, result)) {
		ROS_ERROR("Could not load the urdf from parameter server.");
		return false;
	}
	this->inUrdfXmlString = result;
	this->urdfModel.initString(this->inUrdfXmlString);
	return true;
}

bool UrdfUpdate::readFromString(const string& urdfString) {
	this->inUrdfXmlString = urdfString;
	this->urdfModel.initString(this->inUrdfXmlString);
	return true;
}

bool UrdfUpdate::writeToFile(const string& filename) {
	TiXmlDocument* doc;
	doc = urdf::exportURDF(this->urdfModel);
	return doc->SaveFile(filename.c_str());
}

bool UrdfUpdate::updateJointOffsets(const map<string, double>& offsets) {
	// apply the offsets to the model
	JointUpdate ju(urdfModel);
	ju.setOffsets(offsets);
	vector<Joint> updatedJoints;
	ju.getModifiedJoints(updatedJoints);

	// update the corresponding xml nodes/attributes
	BOOST_FOREACH(Joint& joint, updatedJoints) {
		urdfModel.joints_[joint.name]->parent_to_joint_origin_transform =
				joint.parent_to_joint_origin_transform;
	}
	return true;
}

bool UrdfUpdate::updateCameraDeltaTransform(
		const tf::Transform& deltaCameraTransform) {
	tf::Transform newCameraToHeadPitch;

	// calculate new transform
	CameraTransformUpdate ctu(urdfModel);
	ctu.getUpdatedCameraToHead(deltaCameraTransform, newCameraToHeadPitch);

	urdfModel.joints_["CameraBottom"]->parent_to_joint_origin_transform.position =
			urdf::Vector3(newCameraToHeadPitch.getOrigin().getX(),
					newCameraToHeadPitch.getOrigin().getY(),
					newCameraToHeadPitch.getOrigin().getZ());
	urdfModel.joints_["CameraBottom"]->parent_to_joint_origin_transform.rotation =
			urdf::Rotation(newCameraToHeadPitch.getRotation().getX(),
					newCameraToHeadPitch.getRotation().getY(),
					newCameraToHeadPitch.getRotation().getZ(),
					newCameraToHeadPitch.getRotation().getW());

	return true;
}

} /* namespace kinematic_calibration */
