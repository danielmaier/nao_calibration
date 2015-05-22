/*
 * JointUpdaterTest.cpp
 *
 *  Created on: 22.11.2013
 *      Author: stefan
 */

#include "../include/result_publishing/JointUpdate.h"

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <map>

#include "../include/common/ModelLoader.h"

using namespace std;

namespace kinematic_calibration {

TEST(JointUpdateTest, writeCalibrationDataTest1) {
	/*
	 * arrange
	 */

	// initialize the model
	urdf::Model model;
	ifstream file("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	modelLoader.getUrdfModel(model);
	string filename = "test_calib_data_1.xml";

	// initialize the offsets
	map<string, double> offsets;
	offsets["LElbowYaw"] = 0.1;
	offsets["LShoulderPitch"] = 0.1;

	// initialize the update instance
	JointUpdate ju(model);


	/*
	 * act
	 */
	ju.writeCalibrationData(offsets, filename);
	remove(filename.c_str());


	/*
	 * assert
	 */
	ASSERT_TRUE(ju.offsets.count("LElbowYaw"));
	ASSERT_FLOAT_EQ(0.1, ju.offsets["LElbowYaw"]);
	ASSERT_TRUE(ju.calibrationParameters.count("LElbowYaw_r"));
	ASSERT_FLOAT_EQ(0.1, ju.calibrationParameters["LElbowYaw_r"]);

	ASSERT_TRUE(ju.offsets.count("LShoulderPitch"));
	ASSERT_FLOAT_EQ(0.1, ju.offsets["LShoulderPitch"]);
	ASSERT_TRUE(ju.calibrationParameters.count("LShoulderPitch_p"));
	ASSERT_FLOAT_EQ(0.1, ju.calibrationParameters["LShoulderPitch_p"]);
}

TEST(JointUpdateTest, writeCalibrationDataTest2) {
	/*
	 * arrange
	 */

	// initialize the model
	urdf::Model model;
	ifstream modelfile("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(modelfile)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	modelLoader.getUrdfModel(model);
	string filename = "test_calib_data_2.xml";
	remove(filename.c_str());

	// initialize the offsets
	map<string, double> offsets;
	offsets["LElbowYaw"] = 0.1;

	// initialize the update instance
	JointUpdate ju(model);

	// write "old" calibration file
	ofstream outfile(filename.c_str());
	outfile << "<?xml version=\"1.0\"?>" << "\n";
	outfile << "<robot>" << "\n";
	outfile << "<property name=\"LShoulderPitch_p\" value=\"0.1\" />" << "\n";
	outfile << "</robot>" << "\n";
	outfile.close();

	/*
	 * act
	 */
	ju.writeCalibrationData(offsets, filename);
	bool oldDataExists = false;
	bool newDataExists = false;
	ifstream infile(filename.c_str());

	while(!infile.eof()) {
		string line;
		getline(infile, line);
		cout << line;
		if(string::npos != line.find("name=\"LElbowYaw_r\" value=\"0.1\"")) {
			newDataExists = true;
		}
		if(string::npos != line.find("name=\"LShoulderPitch_p\" value=\"0.1\"")) {
			oldDataExists = true;
		}
	}
	infile.close();

	remove(filename.c_str());

	/*
	 * assert
	 */
	ASSERT_TRUE(oldDataExists);
	ASSERT_TRUE(newDataExists);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
