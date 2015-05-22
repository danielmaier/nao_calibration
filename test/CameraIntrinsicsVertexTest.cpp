/*
 * CameraIntrinsicsVertexTest.cpp
 *
 *  Created on: 06.12.2013
 *      Author: stefan
 */

#include "../include/optimization/CameraIntrinsicsVertex.h"

#include <gtest/gtest.h>
#include <sensor_msgs/CameraInfo.h>

namespace kinematic_calibration {

TEST(CameraIntrinsicsVertexTest, oplusDeltaTest) {
	// arrange
	sensor_msgs::CameraInfo msg;
	msg.distortion_model = "plumb_bob";
	msg.height = 480;
	msg.width = 640;
	msg.distortion_model = "plumb_bob";
	msg.D.resize(5);
	for (int i = 0; i < 5; i++)
		msg.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		msg.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		msg.K[i] = 0.0;
	msg.P[0] = 550;
	msg.P[5] = 550;
	msg.P[2] = 320;
	msg.P[6] = 200;
	msg.P[10] = 1;
	msg.K[0] = 550;
	msg.K[4] = 550;
	msg.K[2] = 320;
	msg.K[5] = 200;
	msg.K[8] = 1;

	CameraIntrinsicsVertex vertex(msg);
	double delta[vertex.dimension()];
	for (int i = 0; i < vertex.dimension(); i++)
		delta[i] = i + 1;

	// act
	vertex.oplus(delta);
	sensor_msgs::CameraInfo updatedCamera = vertex.estimate();

	// assert
	ASSERT_EQ(msg.P[0] + 1, updatedCamera.P[0]);
	ASSERT_EQ(msg.P[5] + 2, updatedCamera.P[5]);
	ASSERT_EQ(msg.P[2] + 3, updatedCamera.P[2]);
	ASSERT_EQ(msg.P[6] + 4, updatedCamera.P[6]);
	ASSERT_EQ(updatedCamera.K[0], updatedCamera.P[0]);
	ASSERT_EQ(updatedCamera.K[4], updatedCamera.P[5]);
	ASSERT_EQ(updatedCamera.K[2], updatedCamera.P[2]);
	ASSERT_EQ(updatedCamera.K[5], updatedCamera.P[6]);
	ASSERT_EQ(msg.D[0] + 5, updatedCamera.D[0]);
	ASSERT_EQ(msg.D[1] + 6, updatedCamera.D[1]);
	ASSERT_EQ(msg.D[2] + 7, updatedCamera.D[2]);
	ASSERT_EQ(msg.D[3] + 8, updatedCamera.D[3]);
	ASSERT_EQ(msg.D[4] + 9, updatedCamera.D[4]);
}

TEST(CameraIntrinsicsVertexTest, setToOriginTest) {
	// arrange
	sensor_msgs::CameraInfo msg;
	msg.distortion_model = "plumb_bob";
	msg.height = 480;
	msg.width = 640;
	msg.distortion_model = "plumb_bob";
	msg.D.resize(5);
	for (int i = 0; i < 5; i++)
		msg.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		msg.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		msg.K[i] = 0.0;
	msg.P[0] = 550;
	msg.P[5] = 550;
	msg.P[2] = 320;
	msg.P[6] = 200;
	msg.P[10] = 1;
	msg.K[0] = 550;
	msg.K[4] = 550;
	msg.K[2] = 320;
	msg.K[5] = 200;
	msg.K[8] = 1;

	CameraIntrinsicsVertex vertex(msg);
	double delta[vertex.dimension()];
	for (int i = 0; i < vertex.dimension(); i++)
		delta[i] = i + 1;

	// act
	vertex.oplus(delta);
	vertex.setToOrigin();
	sensor_msgs::CameraInfo updatedCamera = vertex.estimate();

	// assert
	ASSERT_EQ(msg.P[0], updatedCamera.P[0]);
	ASSERT_EQ(msg.P[5], updatedCamera.P[5]);
	ASSERT_EQ(msg.P[2], updatedCamera.P[2]);
	ASSERT_EQ(msg.P[6], updatedCamera.P[6]);
	ASSERT_EQ(updatedCamera.K[0], updatedCamera.P[0]);
	ASSERT_EQ(updatedCamera.K[4], updatedCamera.P[5]);
	ASSERT_EQ(updatedCamera.K[2], updatedCamera.P[2]);
	ASSERT_EQ(updatedCamera.K[5], updatedCamera.P[6]);
	ASSERT_EQ(msg.D[0], updatedCamera.D[0]);
	ASSERT_EQ(msg.D[1], updatedCamera.D[1]);
	ASSERT_EQ(msg.D[2], updatedCamera.D[2]);
	ASSERT_EQ(msg.D[3], updatedCamera.D[3]);
	ASSERT_EQ(msg.D[4], updatedCamera.D[4]);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
