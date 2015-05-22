/*
 * ErrorModelTest.cpp
 *
 *  Created on: 24.02.2014
 *      Author: stefan
 */

#include "../include/common/ErrorModel.h"

#include <gtest/gtest.h>
#include <fstream>
#include <kdl/tree.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include "../include/common/KinematicChain.h"
#include "../include/common/ModelLoader.h"
#include "../include/optimization/KinematicCalibrationState.h"

namespace kinematic_calibration {

TEST(SinglePointErrorModel, getImageCoordinatesTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	CalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	SinglePointErrorModel errorModel(chain);

	// act
	errorModel.getImageCoordinates(state, measurement, x, y);

	// assert
	double x_expected = -465.791;
	double y_expected = 46.8939;
	double eps = 1e-3;
	ASSERT_TRUE(fabs(x - x_expected) < eps);
	ASSERT_TRUE(fabs(y - y_expected) < eps);
}

TEST(SinglePointErrorModel, getErrorTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	CalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	SinglePointErrorModel errorModel(chain);

	vector<double> error;

	// act
	errorModel.getError(state, measurement, error);

	// assert
	double x_expected = -465.791;
	double y_expected = 46.8939;
	double eps = 1e-3;
	ASSERT_EQ(error.size(), 2);
	ASSERT_NEAR(fabs(100 - x_expected), fabs(error[0]), eps);
	ASSERT_NEAR(fabs(100 - y_expected), fabs(error[1]), eps);
}

TEST(SinglePointErrorModel, getSquaredErrorTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	CalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	SinglePointErrorModel errorModel(chain);

	vector<double> error;

	// act
	errorModel.getSquaredError(state, measurement, error);

	// assert
	double x_expected = -465.791;
	double y_expected = 46.8939;
	double x_error = fabs(100 - x_expected);
	double y_error = fabs(100 - y_expected);
	double eps = 1;
	ASSERT_EQ(error.size(), 2);
	ASSERT_NEAR(x_error * x_error, error[0], eps);
	ASSERT_NEAR(y_error * y_error, error[1], eps);
}
/*
class TestingSinglePointErrorModel: public SinglePointErrorModel {
public:
	TestingSinglePointErrorModel(KinematicChain& kinematicChain) :
			SinglePointErrorModel(kinematicChain) {
	}
	using SinglePointErrorModel::calcCameraIntrinsicsDerivatives;
	using SinglePointErrorModel::calcJointOffsetsDerivatives;
	using SinglePointErrorModel::calculateDerivative;
};
*/
/*
TEST(SinglePointErrorModel, calcCameraIntrinsicsDerivativesTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	KinematicCalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	TestingSinglePointErrorModel errorModel(chain);

	vector<double> derivatives;
	double h = 1e-6;

	// act
	derivatives = errorModel.calcCameraIntrinsicsDerivatives(state, measurement,
			h);

	// assert
	// Note: The following expected values are calculated using Mathematica.
	double fx_expected = -628.631 + 4.08241 * state.cameraInfo.P[0];
	double fy_expected = -55.6749 + 0.154985 * state.cameraInfo.P[5];
	double cx_expected = -1771.58 + 2 * state.cameraInfo.P[2];
	double cy_expected = -506.212 + 2 * state.cameraInfo.P[6];
	double eps = 1;
	ASSERT_EQ(derivatives.size(), 9);
	ASSERT_NEAR(fx_expected, derivatives[0], eps);
	ASSERT_NEAR(fy_expected, derivatives[1], eps);
	ASSERT_NEAR(cx_expected, derivatives[2], eps);
	ASSERT_NEAR(cy_expected, derivatives[3], eps);
}*/

/*
 TEST(SinglePointErrorModel, calcJointOffsetsDerivativesTest) {
 // arrange
 ifstream file("nao.urdf");
 std::string urdfStr((std::istreambuf_iterator<char>(file)),
 std::istreambuf_iterator<char>());
 ModelLoader modelLoader;
 modelLoader.initializeFromUrdf(urdfStr);
 KDL::Tree tree;
 modelLoader.getKdlTree(tree);

 map<string, double> pos;
 pos["HeadPitch"] = 0.1;
 pos["HeadYaw"] = 0.2;

 KinematicCalibrationState state;
 state.cameraInfo.height = 480;
 state.cameraInfo.width = 640;
 state.cameraInfo.distortion_model = "plumb_bob";
 state.cameraInfo.D.resize(5);
 for (int i = 0; i < 5; i++)
 state.cameraInfo.D[i] = 0.0;
 for (int i = 0; i < 12; i++)
 state.cameraInfo.P[i] = 0.0;
 for (int i = 0; i < 9; i++)
 state.cameraInfo.K[i] = 0.0;
 state.cameraInfo.P[0] = 550;
 state.cameraInfo.P[5] = 550;
 state.cameraInfo.P[2] = 320;
 state.cameraInfo.P[6] = 200;
 state.cameraInfo.P[10] = 1;
 state.cameraInfo.K[0] = 550;
 state.cameraInfo.K[4] = 550;
 state.cameraInfo.K[2] = 320;
 state.cameraInfo.K[5] = 200;
 state.cameraInfo.K[8] = 1;
 state.cameraToHeadTransformation.setIdentity();

 measurementData measurement;
 measurement.marker_data.push_back(320.0); // x
 measurement.marker_data.push_back(1387.0); // y
 measurement.chain_name = "test_larm";
 measurement.chain_root = "CameraBottom_frame";
 measurement.chain_tip = "torso";
 for (map<string, double>::iterator it = pos.begin(); it != pos.end();
 it++) {
 measurement.jointState.name.push_back(it->first);
 measurement.jointState.position.push_back(it->second);
 }

 KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
 measurement.chain_name);

 double x, y;
 TestingSinglePointErrorModel errorModel(chain);

 vector<double> derivatives;
 double h = 1e-6;

 // act
 derivatives = errorModel.calcJointOffsetsDerivatives(state, measurement,
 h);

 // assert
 // Note: The following expected values are calculated using Mathematica.
 double expected = 0;
 double eps = 1;
 cout << "derivatives[0] " << derivatives[0] << endl;
 cout << "derivatives[1] " << derivatives[1] << endl;
 cout << "derivatives[2] " << derivatives[2] << endl;
 ASSERT_EQ(derivatives.size(), 2);
 ASSERT_NEAR(expected, derivatives[0], eps);
 }*/

/*
TEST(SinglePointErrorModel, calcDerivativeTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	measurementData measurement;
	string chain_name = "test_larm";
	string chain_root = "CameraBottom_frame";
	string chain_tip = "LWristYaw_link";

	KinematicChain chain(tree, chain_root, chain_tip, chain_name);
	TestingSinglePointErrorModel errorModel(chain);
	double derivative;
	double h = 2;
	vector<double> errorMinus, errorPlus;
	errorMinus.push_back(1);
	errorMinus.push_back(2);
	errorPlus.push_back(3);
	errorPlus.push_back(4);

	// act
	derivative = errorModel.calculateDerivative(errorMinus, errorPlus,
			h);

	// assert
	double expected = 1.0;
	double eps = 1e-6;
	ASSERT_NEAR(expected, derivative, eps);
}*/

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
