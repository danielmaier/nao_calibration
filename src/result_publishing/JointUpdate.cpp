/*
 * JointUpdater.cpp
 *
 *  Created on: 22.11.2013
 *      Author: stefan
 */

#include "../../include/result_publishing/JointUpdate.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ptree_fwd.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <urdf_model/pose.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <utility>

using namespace boost;
using namespace boost::property_tree;

const ptree& empty_ptree() {
	static ptree t;
	return t;
}

namespace kinematic_calibration {

JointUpdate::JointUpdate(urdf::Model model) :
		model(model) {

}

JointUpdate::~JointUpdate() {
}

void JointUpdate::setOffsets(map<string, double> offsets) {
	this->offsets = offsets;
}

void JointUpdate::getModifiedJoints(vector<urdf::Joint>& joints) {
    cout << "JointUpdate::getModifiedJoints" << endl;
	for (map<string, double>::iterator it = offsets.begin();
			it != offsets.end(); it++) {
		double r, p, y;
		string name = it->first;
		double offset = it->second;
		urdf::Rotation rot_old, rot_new;
		boost::shared_ptr<const urdf::Joint> joint_ptr = model.getJoint(name);
		rot_old = joint_ptr->parent_to_joint_origin_transform.rotation;
		rot_old.getRPY(r, p, y);
		urdf::Vector3 axis = joint_ptr->axis;
		rot_new.setFromRPY(r + axis.x * offset, p + axis.y * offset,
				y + axis.z * offset);
		urdf::Joint joint_new(*joint_ptr.get());
		joint_new.parent_to_joint_origin_transform.rotation = rot_new;
		joints.push_back(joint_new);
		cout << "joint: " << name << " ";
		cout << "axis: " << axis.x << " " <<  axis.y << " " <<
				 axis.z << " ";
		cout << "offset: " << offset << " ";
		cout << "rpy old: " << r << " " << p << " " << y << "\n";
	}
}

void JointUpdate::readOldCalibrationData(const string& filename) {
	// check whether an old file exists at all
	ifstream fi(filename.c_str());

	// if not, there is nothing to do
	if (false == fi.good())
		return;

    cout << "JointUpdate::readOldCalibrationData found old calibration in " << filename << endl;


	// otherwise parse the file and save the contents
	ptree tree;
	read_xml(filename, tree);
	const ptree& calibrationData = tree.get_child("robot", empty_ptree());
	BOOST_FOREACH(const ptree::value_type & f, calibrationData) {
        //cout << "f.first.c_str() " << f.first.c_str() << "\n";
		// check if node name is "property"
		if (!std::strcmp(f.first.c_str(), "property")) {
			// get "name" attribute
			string name = f.second.get<string>("<xmlattr>.name");
			// get "value" attribute
			double value = f.second.get<double>("<xmlattr>.value");
			// save the old parameters
			this->calibrationParameters[name] = value;
		}
	}
}

void JointUpdate::writeCalibrationData(const string& filename) {
	// update the current joints with the offsets
	vector<urdf::Joint> joints;
	getModifiedJoints(joints);

	// add the new parameters to the parameter map
	for (int i = 0; i < joints.size(); i++) {
		urdf::Joint joint = joints[i];
		double r, p, y;
		joint.parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
		string name = joint.name;
		calibrationParameters[name + "_r"] = r;
		calibrationParameters[name + "_p"] = p;
		calibrationParameters[name + "_y"] = y;
	}

	// write the new calibration file
	ofstream file;
	file.open(filename.c_str());
	file << "<?xml version=\"1.0\"?>\n";
	file << "<robot>\n";
	for (map<string, double>::iterator it = calibrationParameters.begin();
			it != calibrationParameters.end(); it++) {
		string name = it->first;
		double value = it->second;
		file << "<property name=\"" << name << "\" value=\"" << value
				<< "\" />";
		file << "\n";
	}
	file << "</robot>\n";
	file.close();
}

void JointUpdate::writeCalibrationData(const map<string, double> offsets,
		const string& filename) {
	// save the offsets
	setOffsets(offsets);

	// read old calibration data, if available
	readOldCalibrationData(filename);

	// write new (and old, unchanged) calibration data
	writeCalibrationData(filename);
}

} /* namespace kinematic_calibration */

