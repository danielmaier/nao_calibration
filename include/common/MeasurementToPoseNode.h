/*
 * MeasurementToPoseNode.h
 *
 *  Created on: 20.03.2014
 *      Author: stefan
 */

#ifndef MEASUREMENTTOPOSENODE_H_
#define MEASUREMENTTOPOSENODE_H_

#include <ros/ros.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "kinematic_calibration/measurementData.h"
#include "naoqi_msgs/JointTrajectoryGoal.h"

namespace kinematic_calibration {

using namespace std;
using namespace ros;

/**
 * Extracts the poses from measurements.
 */
class MeasurementToPoseNode {
public:
	/**
	 * Constructor.
	 */
	MeasurementToPoseNode();

	/**
	 * Desctructor.
	 */
	virtual ~MeasurementToPoseNode();

	/**
	 * "Main" loop. Collects the measurements and generates the pose file.
	 * The measurements to be converted have to be contained in the ROS
	 * parameter "measurement_ids". The method returns as soon as all
	 * needed measurement messages are received and converted.
	 */
	void run();

protected:
	/**
	 * Collects the measurements and generates the pose file.
	 * The measurements to be converted have to be contained in the ROS
	 * parameter "measurement_ids". The method returns as soon as all
	 * needed measurement messages are received and converted.
	 */
	void collectData();

	/**
	 * Callback for measurement messages.
	 * Processes one message:
	 * - if its ID is contained in the "measurement_ids" parameter,
	 * the relevant joint_states are used to generate a new pose
	 * - otherwise ignores the message.
	 * @param[in] msg Received message.
	 */
	void measurementCb(const measurementDataConstPtr& msg);

private:
	/**
	 * Prefix for the pose name.
	 * Initialized by the ROS parameter "prefix".
	 */
	string prefix;

	/**
	 * Filename for the new pose file.
	 * Initialized by the ROS parameter "filename".
	 */
	string filename;

	/**
	 * List of measurement IDs that shall be used.
	 */
	vector<string> measurementIds;

	/**
	 * List of measurement IDs that are currently converted.
	 */
	vector<string> processedIds;

	/**
	 * List of joint names to be used.
	 * Initialized by the ROS parameter "joint_names". (List of strings)
	 */
	vector<string> jointNames;

	/**
	 * Subscriber instance for measurement messages.
	 */
	Subscriber measurementSubscriber;

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

	/**
	 * File stream instance.
	 */
	ofstream poseFile;
};

} /* namespace kinematic_calibration */

#endif /* MEASUREMENTTOPOSENODE_H_ */
