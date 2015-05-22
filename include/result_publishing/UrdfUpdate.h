/*
 * UrdfUpdate.h
 *
 *  Created on: 18.12.2013
 *      Author: stefan
 */

#ifndef URDFUPDATE_H_
#define URDFUPDATE_H_

#include <boost/property_tree/ptree.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <tf/tf.h>
#include <fstream>
#include <map>
#include <string>

namespace kinematic_calibration {

using namespace std;
using namespace ros;
using namespace urdf;

/**
 * Updates the content of an URDF file/string.
 */
class UrdfUpdate {
public:
	UrdfUpdate();
	virtual ~UrdfUpdate();

	bool readFromFile(const string& filename);
	bool readFromRos(const string& paramName);
	bool readFromString(const string& urdfString);

	bool updateJointOffsets(const map<string, double>&);
	bool updateCameraDeltaTransform(const tf::Transform& deltaCameraTransform);

	bool writeToFile(const string& filename);

private:
	string inUrdfXmlString;
	urdf::Model urdfModel;
	NodeHandle nh;
};

} /* namespace kinematic_calibration */

#endif /* URDFUPDATE_H_ */
