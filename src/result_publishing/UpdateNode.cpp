/*
 * UpdateNode.cpp
 *
 *  Created on: 27.11.2013
 *      Author: stefan
 */

#include <kinematic_calibration/calibrationResult.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <map>
#include <string>
#include <vector>
#include <camera_calibration_parsers/parse_yml.h>

#include "../../include/common/ModelLoader.h"
#include "../../include/result_publishing/CameraTransformUpdate.h"
#include "../../include/result_publishing/JointUpdate.h"
#include "../../include/result_publishing/UrdfUpdate.h"

using namespace kinematic_calibration;
using namespace ros;
using namespace std;

void resultCb(const calibrationResultConstPtr& msg);
string jointOffsetsFilename;
string cameraTransformFilename;
string urdfFilename;
string markerTransformsFilename;
string cameraIntrnsicsFilename;

int main(int argc, char** argv) {
	init(argc, argv, "JointUpdateNode");
	NodeHandle nh;

	// get the filenames
	nh.param(string("joint_offsets_filename_suffix"), jointOffsetsFilename,
			string("calibration_joint_offsets.xacro"));
	nh.param(string("camera_transform_filename_suffix"),
			cameraTransformFilename,
			string("calibration_camera_transform.xacro"));
	nh.param(string("urdf_filename_suffix"), urdfFilename,
			string("robot_model_calibrated.xml"));
	nh.param(string("marker_transforms_filename_suffix"),
			markerTransformsFilename,
			string("calibration_marker_transformations.xacro"));
	nh.param(string("camera_intrnsics_filename"), cameraIntrnsicsFilename,
			string("nao_bottom_640x480.yaml"));

	ModelLoader modelLoader;
	modelLoader.initializeFromRos();

	urdf::Model model;
	modelLoader.getUrdfModel(model);

	// get the robot name and prepend to the filenames
	string filePrefix = model.getName();
	jointOffsetsFilename = filePrefix + "_" + jointOffsetsFilename;
	cameraTransformFilename = filePrefix + "_" + cameraTransformFilename;
	urdfFilename = filePrefix + "_" + urdfFilename;
	markerTransformsFilename = filePrefix + "_" + markerTransformsFilename;

    Subscriber sub = nh.subscribe("/onlineCalibration/calibration_result",
			1, resultCb);
	spin();
	return 0;
}

void resultCb(const calibrationResultConstPtr& msg) {
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();

	urdf::Model model;
	modelLoader.getUrdfModel(model);

	// write the parameter file for the joint offsets
	map<string, double> offsets;
	for (int i = 0; i < msg->jointNames.size(); i++) {
		offsets[msg->jointNames[i]] = msg->jointOffsets[i];
	}

	JointUpdate ju(model);
	ju.writeCalibrationData(offsets, jointOffsetsFilename);

	// write the parameter file for the camera transformation
	tf::Transform transform;
	tf::transformMsgToTF(msg->cameraTransform, transform);

	CameraTransformUpdate ctu(model);
	ctu.writeCalibrationData(transform, cameraTransformFilename);

	// write the new urdf file
	UrdfUpdate urdfUpdate;
	urdfUpdate.readFromRos("/robot_description");
	urdfUpdate.updateJointOffsets(offsets);
	urdfUpdate.updateCameraDeltaTransform(transform);
	urdfUpdate.writeToFile(urdfFilename);

	// print the marker transformation on the screen and as parameters in a file
	// TODO: move into own class?
	// TODO: also update urdf...
	ofstream file;
	file.open(markerTransformsFilename.c_str());
	file << "<?xml version=\"1.0\"?>\n";
	file << "<robot>\n";
	for (int i = 0; i < msg->chainNames.size(); i++) {
		double rr, rp, ry, tx, ty, tz;
		tf::Transform current;
		tf::transformMsgToTF(msg->endeffectorToMarker[i], current);
		current = current.inverse();
		tf::Matrix3x3(current.getRotation()).getRPY(rr, rp, ry);
		tx = current.getOrigin().getX();
		ty = current.getOrigin().getY();
		tz = current.getOrigin().getZ();
		cout << "chain: " << msg->chainNames[i] << "\n";
		cout << "translation: " << tx << " " << ty << " " << tz << "\n";
		cout << "rotation: " << rr << " " << rp << " " << ry << "\n";
		file << "\t<property name=\"" << msg->chainNames[i] << "_marker_tx"
				<< "\" value=\"" << tx << "\" />\n";
		file << "\t<property name=\"" << msg->chainNames[i] << "_marker_ty"
				<< "\" value=\"" << ty << "\" />\n";
		file << "\t<property name=\"" << msg->chainNames[i] << "_marker_tz"
				<< "\" value=\"" << tz << "\" />\n";
		file << "\t<property name=\"" << msg->chainNames[i] << "_marker_rr"
				<< "\" value=\"" << rr << "\" />\n";
		file << "\t<property name=\"" << msg->chainNames[i] << "_marker_rp"
				<< "\" value=\"" << rp << "\" />\n";
		file << "\t<property name=\"" << msg->chainNames[i] << "_marker_ry"
				<< "\" value=\"" << ry << "\" />\n";
	}
	file << "</robot>\n";
	file.close();

	// write camera intrinsics
	const string cameraName = "nao_camera";
	camera_calibration_parsers::writeCalibrationYml(cameraIntrnsicsFilename,
			cameraName, msg->cameraInfo);
}

