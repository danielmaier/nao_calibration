/*
 * MeasurementToPoseNode.cpp
 *
 *  Created on: 20.03.2014
 *      Author: stefan
 */

#include "../../include/common/MeasurementToPoseNode.h"

namespace kinematic_calibration {

MeasurementToPoseNode::MeasurementToPoseNode() {
	// register callback for measurement messages
	measurementSubscriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&MeasurementToPoseNode::measurementCb, this);

	// get parameters
	nh.getParam("prefix", prefix);
	nh.getParam("filename", filename);
	nh.getParam("measurement_ids", measurementIds);
	nh.getParam("joint_names", jointNames);

	// print stuff
	ROS_INFO("Pose prefix: %s", prefix.c_str());
	ROS_INFO("Filename: %s", filename.c_str());
}

MeasurementToPoseNode::~MeasurementToPoseNode() {
	// nothing to do
}

void MeasurementToPoseNode::run() {
	collectData();
}

void MeasurementToPoseNode::collectData() {
	poseFile.open(filename.c_str());
	while (processedIds.size() != measurementIds.size()) {
		ros::spinOnce();
	}
	poseFile.flush();
	poseFile.close();
}

void MeasurementToPoseNode::measurementCb(const measurementDataConstPtr& msg) {
	ROS_INFO("Message received...");
	if (find(measurementIds.begin(), measurementIds.end(), msg->id)
			== measurementIds.end()) {
		// ignore measurement if not within the list of IDs to be used
		ROS_INFO("Ignore ID %s...", msg->id.c_str());
		return;
	}

	ROS_INFO("Process ID %s...", msg->id.c_str());

	// save the id
	processedIds.push_back(msg->id);

	// print name
	stringstream ss;
	ss << prefix;
	ss << std::setfill ('0') << std::setw (3);
	ss << processedIds.size();
	string poseName = ss.str();
	this->poseFile << "### generated from " << msg->id << "\n";
	this->poseFile << poseName << ":\n";

	// print joint names
	this->poseFile << "  joint_names: [";
	int numOfJoints = 0;
	for (int i = 0; i < msg->jointState.name.size(); i++) {
		const string jointName = msg->jointState.name[i];
		if (find(jointNames.begin(), jointNames.end(), jointName)
				!= jointNames.end()) {
			numOfJoints++;
			// print the joint name
			this->poseFile << "\"" << jointName << "\"";
			if (numOfJoints != jointNames.size()) {
				// print comma if not the last
				this->poseFile << ", ";
			}
		}
	}
	this->poseFile << "]\n";

	// print time from start
	this->poseFile << "  time_from_start: 1.0\n";

	// print positions
	this->poseFile << "  positions: [";
	numOfJoints = 0;
	for (int i = 0; i < msg->jointState.name.size(); i++) {
		const string jointName = msg->jointState.name[i];
		if (find(jointNames.begin(), jointNames.end(), jointName)
				!= jointNames.end()) {
			const double position = msg->jointState.position[i];
			numOfJoints++;
			// print the position
			this->poseFile << position;
			if (numOfJoints != jointNames.size()) {
				// print comma if not the last
				this->poseFile << ", ";
			}
		}
	}
	this->poseFile << "]\n";
	this->poseFile << endl;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "measurementToPose");
	MeasurementToPoseNode node;
	node.run();
}
