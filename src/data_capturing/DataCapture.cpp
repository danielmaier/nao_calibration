/*
 * DataCapture.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/DataCapture.h"

#include <actionlib/client/simple_client_goal_state.h>
#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <image_transport/subscriber.h>
#include <naoqi_msgs/JointTrajectoryGoal.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/init.h>
#include <ros/publisher.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;

namespace kinematic_calibration {

DataCapture::DataCapture(CalibrationContext& context) :
    nhPrivate("/onlineCalibration"),  stiffnessClient(nh, "joint_stiffness_trajectory"),
    trajectoryClient(nh, "joint_trajectory"), bodyPoseClient(nh, "body_pose"),
    it(nh), pauseManager(false), markerFound(false), receivedJointStates(false),
    receivedImage(false), context(context)
{

	// get camera information
    // TODO: Is it okay to outcomment this part? Is cameraModel really not needed?
    /*
    camerainfoSub = nh.subscribe("/nao_camera/camera_info", 1,
			&DataCapture::camerainfoCallback, this);
	ROS_INFO("Waiting for camera info message...");
    ros::Rate r(10.0);
	while (ros::getGlobalCallbackQueue()->isEmpty()) {
        //ros::Duration(0.5).sleep();
        //r.sleep();
	}
	ros::getGlobalCallbackQueue()->callAvailable();
    camerainfoSub.shutdown(); // we are only interested in first camera info...
	ROS_INFO("Done.");
    */



	// subscribe the joint states topic
	ros::SubscribeOptions jointStatesOps = ros::SubscribeOptions::create<
			sensor_msgs::JointState>("/joint_states", 1,
			boost::bind(&DataCapture::jointStatesCallback, this, _1),
			ros::VoidPtr(), &jointStatesQueue);
	jointStateSub = nh.subscribe(jointStatesOps);

	// subscribe the image topic
    imageSub = it.subscribe("nao_camera/image_raw", 1, &DataCapture::imageCallback, this);

	// advertise the measurement data topic
	measurementPub = nh.advertise<measurementData>(
			"/kinematic_calibration/measurement_data", 100);

    captureVisPub = it.advertise("/kinematic_calibration/image", 1);

	// waiting for server
	ROS_INFO("Waiting for stiffness, trajectory and pose server...");
	stiffnessClient.waitForServer();
	trajectoryClient.waitForServer();
	bodyPoseClient.waitForServer();
	ROS_INFO("Done.");

	// initialize list of head joint names
    // TODO: Get this from robot model or config file
	headJointNames.push_back("HeadYaw");
	headJointNames.push_back("HeadPitch");

    //updateMarker();
    ROS_WARN("DataCapture has no joint Names yet! Same goes for PauseManger");

	// initialize seed
	srand(static_cast<unsigned>(time(0)));

}



void DataCapture::retrieveParamsForChain(const std::string & chainName)
{

    this->chainName = chainName;

    // get parameters
    nhPrivate.getParam(chainName + "/headYaw_min", headYawMin);
    nhPrivate.getParam(chainName + "/headYaw_max", headYawMax);
    nhPrivate.getParam(chainName + "/headYaw_step", headYawStep);
    nhPrivate.getParam(chainName + "/headPitch_min", headPitchMin);
    nhPrivate.getParam(chainName + "/headPitch_max", headPitchMax);
    nhPrivate.getParam(chainName + "/headPitch_step", headPitchStep);
    nhPrivate.getParam(chainName + "/image_topic", imageTopic);

    // get public parameters
    //nhPrivate.getParam(chainName + "/chain_name",   this->chainName);
    nhPrivate.getParam(chainName + "/marker_type", markerType);

    // get the marker detection instance
    markerContext = context.getMarkerContext(markerType);
    markerDetection = markerContext->getMarkerDetectionInstance();


}

const vector<string> &DataCapture::getJointNames()
{
    vector<string> tmp;
    cerr << "This is a bug. This function is pure virtual " << endl;
    return tmp;
}

DataCapture::~DataCapture() {
    cerr << "Destroying DataCapture " << endl;
    //disableStiffness(headJointNames);
    //delete markerDetection;
    //delete markerContext;
}

void DataCapture::enableHeadStiffness() {
	ROS_INFO("Setting head stiffness...");
	enableStiffness(headJointNames);
	ROS_INFO("Done.");
}

void DataCapture::disableHeadStiffness() {
	ROS_INFO("Resetting head stiffness...");
	disableStiffness(headJointNames);
	ROS_INFO("Done.");
}

void DataCapture::enableChainStiffness() {
	ROS_INFO("Setting chain stiffness...");
	enableStiffness(getJointNames());
	ROS_INFO("Done.");
}

void DataCapture::disableChainStiffness() {
	ROS_INFO("Resetting chain stiffness stepwise...");
	disableStiffnessSlowly(getJointNames());
	ROS_INFO("Done.");
}

void DataCapture::setStiffness(const vector<string>& jointNames,
		double stiffness) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;
	point.time_from_start.nsec = 0;

	for (unsigned int i = 0; i < jointNames.size(); i++) {
		point.positions.push_back(stiffness);
	}

    naoqi_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = jointNames;
	goal.trajectory.points.push_back(point);

    naoqi_msgs::JointTrajectoryActionResultConstPtr result;
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
}

void DataCapture::enableStiffness(const vector<string>& jointNames) {
	setStiffness(jointNames, 1.0);
}

void DataCapture::disableStiffness(const vector<string>& jointNames) {
	setStiffness(jointNames, 0.0000001);
}

void DataCapture::disableStiffnessSlowly(const vector<string>& jointNames) {
	double value;
	value = 0.1;
	ROS_INFO("Stiffness: %f", value);
	setStiffness(jointNames, value);

	usleep(3e6);
	value = 0.0;
	ROS_INFO("Stiffness: %f", value);
	setStiffness(jointNames, value);
}



/*
void DataCapture::executeBodyPose(double headYaw, double headPitch, bool relative,
        vector<string> additionalJoints, vector<double> additionalPositions) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start.sec = 0;
    point.time_from_start.nsec = 600 * 1000 * 1000;
    point.positions.push_back(headYaw);
    point.positions.push_back(headPitch);
    point.positions.insert(point.positions.end(), additionalPositions.begin(),
            additionalPositions.end());
    JointTrajectoryGoal goal;
    goal.trajectory.joint_names = headJointNames;
    goal.trajectory.joint_names.insert(goal.trajectory.joint_names.end(),
            additionalJoints.begin(), additionalJoints.end());
    goal.trajectory.points.push_back(point);
    goal.relative = relative;
    trajectoryClient.sendGoal(goal);
    trajectoryClient.waitForResult();
}
*/


void DataCapture::enableStiffnesses()
{
    // enable stiffness
    enableHeadStiffness();
    enableChainStiffness();
}

void DataCapture::disableStiffnesses()
{
    // disable stiffness
    disableChainStiffness();
    disableHeadStiffness();

}

// capture marker observation for a pose given by name and an expected marker location (optional)
// TODO: Use expected location
bool DataCapture::capturePose(const std::string & poseName, double x_exp, double y_exp, measurementData & meas_data) {
    DataCaptureOptions dataCaptureOptions = context.getDataCaptureOptions();


    if (getJointNames().empty() )
    {
        ROS_ERROR("Empty joint names! Joint names need to be set for DataCapture");
        return false;
    }



    // check for pause requests:
    // call blocks if pause requested
    if (pauseManager.pauseRequested()) {
        disableChainStiffness();
        disableHeadStiffness();
        pauseManager.pauseIfRequested();
        enableChainStiffness();
        enableHeadStiffness();
    }

    naoqi_msgs::BodyPoseGoal goal;
    goal.pose_name = poseName;
    this->currentPoseName = poseName;
    ROS_INFO("Calling pose manager to adopt pose %s...",
             poseName.c_str());
    actionlib::SimpleClientGoalState goalState =
            bodyPoseClient.sendGoalAndWait(goal);

    // check whether pose could be executed
    if (goalState != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Unable to execute the pose %s. Reason: %s", poseName.c_str(), goalState.text_.c_str());
        return false;
    }
    ROS_DEBUG("Pose manager action finished successfully.");

    // find the marker
    if (dataCaptureOptions.findMarker) {
        // check if the marker is currently visible to the camera
        updateMarker();
        if (!markerFound) {
            // marker is not visible, so find it by moving the head
            ROS_INFO("Moving head in order to find the marker %s", poseName.c_str());
            findMarker();
        }
        if (!markerFound) {
            // marker could not be found, skip the pose
            ROS_INFO("Was not able to find the marker for pose %s", poseName.c_str());
            return false;
        }
    }

    if (dataCaptureOptions.moveMarkerToCorners) {
        ROS_WARN("moveMarkerToCorners is currently not supported (anymore).");
        // move the head s.t. the marker is
        // within the center region of the camera image
        ROS_INFO("Moving head to CENTER region...");
        moveMarkerToImageRegion(CENTER);
        if (!markerFound)
            return false;

        // move the head s.t. the marker is
        // within the corner regions and publish the data
        publishMeasurement(poseName);
        ROS_INFO("Moving head to LEFT_TOP region...");
        setHeadPose(-0.3, 0.15, true, getJointNames(),
                    generateRandomPositions(getJointNames()));
        publishMeasurement(poseName);
        ROS_INFO("Moving head to LEFT_BOTTOM region...");
        setHeadPose(0, -0.3, true, getJointNames(),
                    generateRandomPositions(getJointNames()));
        publishMeasurement(poseName);
        ROS_INFO("Moving head to RIGHT_BOTTOM region...");
        setHeadPose(0.6, 0, true, getJointNames(),
                    generateRandomPositions(getJointNames()));
        publishMeasurement(poseName);
        ROS_INFO("Moving head to RIGHT_TOP region...");
        setHeadPose(0, 0.3, true, getJointNames(),
                    generateRandomPositions(getJointNames()));
        publishMeasurement(poseName);
        ROS_INFO("Monving back to CENTER region...");
        setHeadPose(-0.3, -0.15, true);
    } else {
        // Not moving to marker to corners. Standard procedure
        publishMeasurement(poseName);
        measurementData data;
        if(!getMeasurementData(poseName, data))
        {
            ROS_WARN("Not publishing pose %s", poseName.c_str());
            return false;
        }
        publishMeasurement(data);
        meas_data = data;
    }


    return true;
}


void DataCapture::playChainPoses() {
	int start, end;
	nhPrivate.getParam("params/start_pose_num", start);
	nhPrivate.getParam("params/end_pose_num", end);
    const string& prefix = getPosePrefix();
    ROS_INFO("Capturing data by executing poses %s%3d to %3d from pose manager", prefix.c_str(), start, end);
    /*
    //CalibrationOptions options = context.getCalibrationOptions();
	DataCaptureOptions dataCaptureOptions = context.getDataCaptureOptions();

	// print some info about the settings
	ROS_INFO("Parameter %s ist set to %s:", "find_marker",
			dataCaptureOptions.findMarker ? "true" : "false");
	if (dataCaptureOptions.findMarker)
		ROS_INFO(
				"If the marker ist not visible to the camera, the head will be moved in order to find the marker.");
	else
		ROS_INFO(
				"If the marker ist not visible to the camera, the pose will be skipped.");

	ROS_INFO("Parameter %s ist set to %s:", "move_marker_to_corners",
			dataCaptureOptions.moveMarkerToCorners ? "true" : "false");
	if (!dataCaptureOptions.moveMarkerToCorners)
        ROS_INFO("The measurements will be taken at the poses defined by the specified joint states.");
	else
        ROS_INFO("For each pose, the head will be moved s.t. the marker is in the center and the four corners.");
        */


    if (getJointNames().empty() )
    {
        ROS_ERROR("Empty joint names! Joint names need to be set for DataCapture");
        return;
    }

	// enable stiffness
	enableHeadStiffness();
	enableChainStiffness();
    // TODO: enable body stiffness

	// start the loop
	for (int i = start; i <= end && ros::ok(); i += 1) {

		// execute next pose
		stringstream ss;
		ss << prefix;
		ss << std::setfill('0') << std::setw(3);
		ss << i;
		string poseName = ss.str();
        measurementData meas;
        if (!capturePose(poseName, 0, 0, meas))
            ROS_WARN("Failed to capture data for pose %s. ",poseName.c_str());
    }

	// disable stiffness
	disableChainStiffness();
    disableHeadStiffness();
    // TODO: disable body stiffness
}

void DataCapture::publishMeasurementLoop() {
    ros::Rate r(10.0);
	while (ros::ok()) {
        publishMeasurement("manual-000");
        r.sleep();
	}
}

void DataCapture::moveMarkerToImageRegion(Region region) {
	int xMin = 0;
	int yMin = 0;
	int xMax = 640; // TODO: parameterize!!
	int yMax = 480; // TODO: parameterize!!
	int delta = 150; // TODO: parameterize!!

	// set region rectangle
	double xRegMin, xRegMax, yRegMin, yRegMax;
	switch (region) {
	case LEFT_TOP:
		xRegMin = xMin;
		xRegMax = delta;
		yRegMin = yMin;
		yRegMax = delta;
		break;
	case LEFT_BOTTOM:
		xRegMin = xMin;
		xRegMax = delta;
		yRegMin = yMax - delta;
		yRegMax = yMax;
		break;
	case RIGHT_TOP:
		xRegMin = xMax - delta;
		xRegMax = xMax;
		yRegMin = yMin;
		yRegMax = delta;
		break;
	case RIGHT_BOTTOM:
		xRegMin = xMax - delta;
		xRegMax = xMax;
		yRegMin = yMax - delta;
		yRegMax = yMax;
		break;
	case CENTER:
		xRegMin = (xMax + xMin) / 2 - delta / 2;
		xRegMax = (xMax + xMin) / 2 + delta / 2;
		yRegMin = (yMax + yMin) / 2 - delta / 2;
		yRegMax = (yMax + yMin) / 2 + delta / 2;
		break;
	default:
		ROS_INFO("Region unknown.");
		break;
	}

	// move head into direction until the marker is into the region
	ROS_INFO("Trying to move the head s.t. the marker "
			"position is within the rectangle (%.2f, %.2f) - (%.2f, %.2f)",
			xRegMin, yRegMin, xRegMax, yRegMax);

	bool isInRegionX = false;
	bool isInRegionY = false;
	double lastX = -1, lastY = -1;
	while (!isInRegionX || !isInRegionY) {
		// get current marker position
		updateMarker();
		if (!markerFound) {
			ROS_INFO("Marker lost.");
			break;
		}

		double relX = 0, relY = 0;
		double x = markerData[0];
		double y = markerData[1];

		// check x
		if (x < xRegMin) {
			relX = (xRegMin - x) / 640 + 0.05;
			ROS_INFO("Moving to the right (relPose = %f).", relX);
		} else if (x > xRegMax) {
			relX = (xRegMax - x) / 640 - 0.05;
			ROS_INFO("Moving to the left (relPose = %f).", relX);
		} else {
			isInRegionX = true;
		}

		// check y
		if (y < yRegMin) {
			relY = (yRegMin - y) / 480 + 0.05;
			ROS_INFO("Moving to downwards (relPose = %f).", -relY);
		} else if (y > yRegMax) {
			relY = (yRegMax - y) / 480 - 0.05;
			ROS_INFO("Moving to upwards (relPose = %f).", -relY);
		} else {
			isInRegionY = true;
		}

		// if the position did not change, we are close to the region and can stop
		if (fabs(lastX - x) < 1 && fabs(lastY - y) < 1) {
			break;
		}

		// change the head position if not in region
		if (!isInRegionX || !isInRegionY) {
			setHeadPose(relX, -relY, true);
		}

		// save last position
		lastX = markerData[0];
		lastY = markerData[1];
	}

	ROS_INFO("Moving into region done!");
}

void DataCapture::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	if (msg->header.stamp < this->curTime) {
		ROS_INFO("Skipping too old Image message.");
		return;
	}
	receivedImage = true;
	cameraFrame = msg.get()->header.frame_id;
	markerFound = markerDetection->detect(msg, markerData);
	image = *msg;
	if (markerFound) {
		ROS_INFO("Marker found at position %f %f", markerData[0],
				markerData[1]);
	} else {
        ROS_DEBUG("No marker found.");
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (markerFound && markerData[0] >= 0 && markerData[0] < msg->width && markerData[1] > 0 && markerData[1] < msg->height)
        cv::circle(cv_ptr->image, cv::Point(markerData[0], markerData[1]), 10, CV_RGB(255,0,0));

    captureVisPub.publish(cv_ptr->toImageMsg());
}

void DataCapture::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	cameraModel.fromCameraInfo(msg);
	ROS_INFO("Camera model set.");
}

void DataCapture::findMarker() {
	if (markerFound)
		return;

	for (double headYaw = headYawMin; headYaw <= headYawMax; headYaw +=
			headYawStep) {
		if (markerFound)
			break;
		for (double headPitch = headPitchMin; headPitch <= headPitchMax;
				headPitch += headPitchStep) {
			setHeadPose(-headYaw, headPitch);
			updateMarker();
			if (markerFound)
				break;
		}
	}
}

void DataCapture::jointStatesCallback(
		const sensor_msgs::JointStateConstPtr& msg) {
// TODO: This looks weird... see imageCallback
    if (msg->header.stamp < this->curTime) {
		ROS_INFO("Skipping too old JointState message.");
		return;
	}
	receivedJointStates = true;
	jointState = *msg;
}

void DataCapture::updateMarker() {
	this->curTime = ros::Time::now();
    updateMarkerRobust();
}

void DataCapture::updateMarkerOnce() {
	receivedImage = false;
	usleep(0.01 * 1000 * 1000);
    ROS_INFO_ONCE("Waiting for image message...");
    ROS_DEBUG("Waiting for image message...");
    // TODO: why so complicated?
    ros::Rate r(20.0);
//	while (ros::getGlobalCallbackQueue()->isEmpty())
//		;
//	while (!receivedImage) {
//		ros::getGlobalCallbackQueue()->callAvailable();
//    }
    while(!receivedImage)
    {
        r.sleep();
        ros::spinOnce();
    }
    ROS_DEBUG("Image received");
}

void DataCapture::updateMarkerRobust() {
	int tries = 0;
	bool unstable = true;
	vector<bool> found;
	vector<double> x;
	vector<double> y;
	while (unstable) {
		while (found.size() >= 3) {
			found.erase(found.begin());
			x.erase(x.begin());
			y.erase(y.begin());
		}

		// update and save data
		updateMarkerOnce();
		found.push_back(markerFound);
		x.push_back(markerData.size() > 0 ? markerData[0] : 0);
		y.push_back(markerData.size() > 0 ? markerData[1] : 0);

		if (found.size() < 3)
			continue;

		bool cbNotFound = true;
		bool cbFound = true;
		double xError = 0, yError = 0;
		for (int i = 0; i < found.size(); i++) {
			cbNotFound = cbNotFound && !found[i];
			cbFound = cbFound && found[i];
			xError += fabs(x[0] - x[i]);
			yError += fabs(y[0] - y[i]);
		}
		if (cbNotFound == true) {
			unstable = false;
		} else if (cbFound && xError < 3 && yError < 3) {
			unstable = false;
		}

		if (++tries > 20) {
			// did not find the marker within a specified period
			markerFound = false;
			unstable = false;
            ROS_WARN("Could not find marker after %d tries", 20);
		}
	}
}

void DataCapture::updateJointStates() {
	this->curTime = ros::Time::now();
	updateJointStatesRobust();
}

void DataCapture::updateJointStatesOnce() {
	receivedJointStates = false;
	usleep(0.01 * 1000 * 1000);
	ROS_INFO("Waiting for joint state message...");
	while (jointStatesQueue.isEmpty())
		;
	while (!receivedJointStates) {
		jointStatesQueue.callAvailable();
	}
}

void DataCapture::updateJointStatesRobust() {
	vector<vector<double> > jointPositions;
	bool unstable = true;
	while (unstable) {
		while (jointPositions.size() >= 3) {
			jointPositions.erase(jointPositions.begin());
		}

		// update and save data
		updateJointStatesOnce();
		jointPositions.push_back(this->jointState.position);

		if (jointPositions.size() < 3)
			continue;
		// check whether the positions changed
		vector<double> delta(this->jointState.position.size(), 0.0);
		// loop through the last saved joint states
		for (int i = 0; i < jointPositions.size(); i++) {
			vector<double> current = jointPositions[i];
			// loop through all joint state positions
			for (int j = 0; j < current.size(); j++) {
				delta[j] += fabs(jointPositions[0][j] - current[j]);
			}
		}

		// check deltas
		unstable = false;
		for (int i = 0; i < delta.size(); i++) {
            if (delta[i] > 0.01)
				unstable = true;
		}
	}

	this->jointState.position;
}

void DataCapture::setHeadPose(double headYaw, double headPitch, bool relative,
		vector<string> additionalJoints, vector<double> additionalPositions) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 0;
	point.time_from_start.nsec = 600 * 1000 * 1000;
	point.positions.push_back(headYaw);
	point.positions.push_back(headPitch);
	point.positions.insert(point.positions.end(), additionalPositions.begin(),
			additionalPositions.end());
    naoqi_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = headJointNames;
	goal.trajectory.joint_names.insert(goal.trajectory.joint_names.end(),
			additionalJoints.begin(), additionalJoints.end());
	goal.trajectory.points.push_back(point);
	goal.relative = relative;
	trajectoryClient.sendGoal(goal);
	trajectoryClient.waitForResult();
}

bool DataCapture::getMeasurementData(const std::string & poseName, measurementData & data){
    updateMarker();
    if (!markerFound) {
        ROS_WARN("No marker found!");
        return false;
    }
    updateJointStates();
    unsigned long id = static_cast<unsigned long>(time(0));
    stringstream ss;
    ss << poseName << "-";
    ss << id;
    data.jointState = jointState;
    data.marker_data = markerData;
    data.id = ss.str();
    data.camera_frame = this->cameraFrame;
    data.chain_name = chainName;
    nh.getParam(chainName + "/chain_root", data.chain_root);
    nh.getParam(chainName + "/chain_tip", data.chain_tip);
    nh.getParam(chainName + "/marker_frame", data.marker_frame);
    data.marker_type = this->markerType;
    data.header.stamp = this->curTime;
    data.image = this->image;
    return true;
}


void DataCapture::publishMeasurement(const measurementData & data) {
    ROS_INFO("Publishing measurement data...");
    measurementPub.publish(data);

    stringstream ss;
    ss << data.id;
    // save the image to disk
    ss << ".jpg";
    string filename = ss.str();
    ROS_INFO("Saving image to file %s%s.", "/tmp/", filename.c_str());
    this->markerDetection->writeImage("/tmp/" + filename);
}

void DataCapture::publishMeasurement(const std::string & poseName) {
    measurementData data;
    if(!getMeasurementData(poseName, data))
    {
        ROS_WARN("Not publishing pose %s", poseName.c_str());
        return;
    }
    publishMeasurement(data);
}

vector<double> DataCapture::generateRandomPositions(
		const vector<string>& joints) {
	// TODO: parameterize!
	double low = -0.10, high = 0.10;
	vector<double> positions;
	const vector<string>& jointNames = this->getJointNames();
	for (int i = 0; i < jointNames.size(); i++) {
		double randomValue = low
				+ static_cast<double>(rand())
						/ (static_cast<double>(RAND_MAX / (high - low)));
		positions.push_back(randomValue);
		cout << jointNames[i] << ": " << randomValue << ", ";
	}
	cout << "\n";
	return positions;
}

const string DataCapture::getPosePrefix() {
	return chainName;
}

LeftArmDataCapture::LeftArmDataCapture(CalibrationContext& context) :
		DataCapture(context) {
	jointNames.push_back("LShoulderPitch");
	jointNames.push_back("LShoulderRoll");
	jointNames.push_back("LElbowYaw");
	jointNames.push_back("LElbowRoll");
	jointNames.push_back("LWristYaw");
	jointNames.push_back("LHand");
}

LeftArmDataCapture::~LeftArmDataCapture() {
}

const vector<string>& LeftArmDataCapture::getJointNames() {
	return jointNames;
}

RightArmDataCapture::RightArmDataCapture(CalibrationContext& context) :
		DataCapture(context) {
	jointNames.push_back("RShoulderPitch");
	jointNames.push_back("RShoulderRoll");
	jointNames.push_back("RElbowYaw");
	jointNames.push_back("RElbowRoll");
	jointNames.push_back("RWristYaw");
	jointNames.push_back("RHand");
}

RightArmDataCapture::~RightArmDataCapture() {
}

const vector<string>& RightArmDataCapture::getJointNames() {
	return jointNames;
}

LegDataCapture::LegDataCapture(CalibrationContext& context) :
		DataCapture(context) {
	// left
	jointNamesLeft.push_back("LHipYawPitch");
	jointNamesLeft.push_back("LHipRoll");
	jointNamesLeft.push_back("LHipPitch");
	jointNamesLeft.push_back("LKneePitch");
	jointNamesLeft.push_back("LAnklePitch");
	jointNamesLeft.push_back("LAnkleRoll");

	// right
	jointNamesRight.push_back("LHipYawPitch");
	jointNamesRight.push_back("RHipRoll");
	jointNamesRight.push_back("RHipPitch");
	jointNamesRight.push_back("RKneePitch");
	jointNamesRight.push_back("RAnklePitch");
	jointNamesRight.push_back("RAnkleRoll");

	// both
	jointNames.insert(jointNames.end(), jointNamesLeft.begin(),
			jointNamesLeft.end());
	jointNames.insert(jointNames.end(), jointNamesRight.begin(),
			jointNamesRight.end());
}

LegDataCapture::~LegDataCapture() {
}

const vector<string>& LegDataCapture::getJointNames() {
	return jointNames;
}

LeftLegDataCapture::LeftLegDataCapture(CalibrationContext& context) :
		LegDataCapture(context) {
}

LeftLegDataCapture::~LeftLegDataCapture() {
}

const vector<string>& LeftLegDataCapture::getJointNames() {
	return jointNamesLeft;
}

RightLegDataCapture::RightLegDataCapture(CalibrationContext& context) :
		LegDataCapture(context) {
}

RightLegDataCapture::~RightLegDataCapture() {
}

const vector<string>& RightLegDataCapture::getJointNames() {
    return jointNamesRight;
}

GeneralDataCapture::GeneralDataCapture(CalibrationContext &context) : DataCapture(context)
{

    m_observe = nhPrivate.advertiseService("observe", &GeneralDataCapture::observe, this);
    imageSub.shutdown();
}

GeneralDataCapture::~GeneralDataCapture()
{
    cerr << "Destroying GeneralDataCapture " << endl;
}

void GeneralDataCapture::setJointNames(const vector<string> &jointNamesIn)
{
    jointNames = jointNamesIn;
    pauseManager.setJointNames(jointNames);
}

const vector<string>& GeneralDataCapture::getJointNames() {
    return jointNames;
}


void GeneralDataCapture::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    DataCapture::imageCallback(msg);
}

bool GeneralDataCapture::observe( Capture::Request & req, Capture::Response & resp)
{
    ROS_INFO("Got a request");
    retrieveParamsForChain(req.chainName);
    setJointNames(req.jointNames);
    imageSub = it.subscribe("nao_camera/image_raw", 1, &GeneralDataCapture::imageCallback, this);
    measurementData meas;
    enableStiffnesses();
    if (!capturePose(req.poseName,0,0,meas))
        return false;
    resp.measurement = meas;
    imageSub.shutdown();
    disableStiffnesses();


    return true;
}

} /* namespace kinematic_calibration */
