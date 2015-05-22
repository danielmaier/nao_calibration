/*
 * TransformationVertexTest.cpp
 *
 *  Created on: 27.02.2014
 *      Author: stefan
 */

#include "../include/optimization/TransformationVertex.h"

#include <gtest/gtest.h>

namespace kinematic_calibration {

TEST(TransformationVertexTest, setEstimate) {
	// arrange
	TransformationVertex vertex;
	tf::Transform tfTransform;
	double tx = 0.2, ty = 0.3, tz = 0.5;
	double rr = 0.7, rp = 0.11, ry = 0.13;
	tf::Quaternion quat;
	quat.setRPY(rr, rp, ry);
	tfTransform.setOrigin(tf::Vector3(tx, ty, tz));
	tfTransform.setRotation(quat);

	// act
	vertex.setEstimate(tfTransform);
	tf::Transform newTransform = vertex.estimate();

	// assert
	double eps = 1e-2;
	ASSERT_NEAR(tx, newTransform.getOrigin()[0], eps);
	ASSERT_NEAR(ty, newTransform.getOrigin()[1], eps);
	ASSERT_NEAR(tz, newTransform.getOrigin()[2], eps);
	ASSERT_NEAR(quat.getX(), newTransform.getRotation().getX(), eps);
	ASSERT_NEAR(quat.getY(), newTransform.getRotation().getY(), eps);
	ASSERT_NEAR(quat.getZ(), newTransform.getRotation().getZ(), eps);
	ASSERT_NEAR(quat.getW(), newTransform.getRotation().getW(), eps);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
