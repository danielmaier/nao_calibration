/*
 * ColorMarkerDetectionTest.cpp
 *
 *  Created on: 02.04.2014
 *      Author: stefan
 */

#include "../include/data_capturing/ColorMarkerDetection.h"

#include <gtest/gtest.h>
#include <opencv2/core/types_c.h>

#ifndef PARAMSPATH
#define PARAMSPATH ""
#endif

namespace kinematic_calibration {

TEST(ColorMarkerDetectionTest, detectionTest) {
	cv::Mat image;
	image = cv::imread("colormarker1.png");

	if(image.empty()) {
		std::cout << "image could not be loaded\n";
		return;
	}

	stringstream userParsFileNameStream, classParsFileNameStream;
	userParsFileNameStream << PARAMSPATH << "CMUserPars.xml";
	classParsFileNameStream << PARAMSPATH << "DesignPars.xml";
	string userParsFileName = userParsFileNameStream.str();
	string classParsFileName = classParsFileNameStream.str();

	ColorMarkerDetection detection(userParsFileName, classParsFileName);
	std::vector<double> data;

	if(!detection.detect(image, data)) {
		std::cout << "[test] could not detect the color marker.\n";
	}

	std::cout << "position " << data[0] << " " << data[1] << "\n";
	ASSERT_TRUE(std::abs(data[0] - 311) < 1);
	ASSERT_TRUE(std::abs(data[1] - 398) < 1);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
