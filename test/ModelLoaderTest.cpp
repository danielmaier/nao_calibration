/*
 * ModelLoaderTest.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/common/ModelLoader.h"

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

namespace kinematic_calibration {

TEST(ModelLoaderTest, initializeFromUrdfTest1) {
	// arrange
	ifstream file("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
	                 std::istreambuf_iterator<char>());
	ModelLoader modelLoader;

	// act
	bool success;
	success = modelLoader.initializeFromUrdf(urdfStr);

	// assert
	ASSERT_TRUE(success);
}

TEST(ModelLoaderTest, initializeFromUrdfTest2) {
	// arrange
	ModelLoader modelLoader;
	std::string urdfStr("foo");

	// act
	bool success;
	success = modelLoader.initializeFromUrdf(urdfStr);

	// assert
	ASSERT_FALSE(success);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
