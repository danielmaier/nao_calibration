/*
 * ModelLoader.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../../include/common/ModelLoader.h"

#include <ros/node_handle.h>
#include <map>

namespace kinematic_calibration {

ModelLoader::ModelLoader() :
		initialized(false) {

}

ModelLoader::~ModelLoader() {
	// TODO Auto-generated destructor stub
}

bool ModelLoader::initializeFromRos() {
	if (!loadUrdfFromRos())
		return false;

	if (!loadKdlFromUrdf())
		return false;

	initialized = true;
	return true;
}

bool ModelLoader::initializeFromUrdf(string urdfXml) {
	this->urdfXml = urdfXml;

	if (!urdfStringToModel())
		return false;

	if (!loadKdlFromUrdf())
		return false;

	initialized = true;
	return true;
}

bool ModelLoader::loadUrdfFromRos() {
	ros::NodeHandle nh, nh_private("~");

	// Get URDF XML
	std::string urdfXmlName, fullUrdfXmlName;
	nh_private.param("robot_description_name", urdfXmlName,
			std::string("robot_description"));
	nh.searchParam(urdfXmlName, fullUrdfXmlName);

	ROS_DEBUG("Reading xml file from parameter server");
	std::string result;

	if (!nh.getParam(fullUrdfXmlName, result)) {
		ROS_ERROR("Could not load the xml from parameter server.");
		return false;
	}

	this->urdfXml = result;

	return urdfStringToModel();
}

bool ModelLoader::loadKdlFromUrdf() {
	if (!kdl_parser::treeFromUrdfModel(this->urdfModel, this->kdlTree)) {
		ROS_ERROR("Could not initialize tree object");
		return false;
	}
	ROS_DEBUG("KDL tree has %ul joints and %ul segments.", kdlTree.getNrOfJoints(), kdlTree.getNrOfSegments());
	return true;
}

bool ModelLoader::urdfStringToModel() {
	if (!this->urdfModel.initString(urdfXml)) {
		ROS_ERROR("Could not initialize robot model");
		return false;
	}
    ROS_DEBUG("Urdf tree has %zu joints and %zu links.", urdfModel.joints_.size(), urdfModel.links_.size());
	return true;
}

void ModelLoader::getKdlTree(KDL::Tree& kdlTree) {
	if (!initialized) {
		ROS_ERROR("Model was not initialized!");
	}
	kdlTree = this->kdlTree;
}

void ModelLoader::getUrdfModel(urdf::Model& model) {
	if (!initialized) {
		ROS_ERROR("Model was not initialized!");
	}
	model = this->urdfModel;
}

const string& ModelLoader::getUrdfXml() const {
	if (!initialized) {
		ROS_ERROR("Model was not initialized!");
	}
	return urdfXml;
}

} /* namespace kinematic_calibration */

