/*
 * ArucoMarkerDetectionTest.cpp
 *
 *  Created on: 11.04.2014
 *      Author: stefan
 */

#include "../include/data_capturing/ArucoMarkerDetection.h"

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <iostream>

#define FILENAME "aruco_marker_1.png"

namespace kinematic_calibration {

TEST(ArucoMarkerDetectionTest, detectionTest) {
	cv::Mat image;
	image = cv::imread(FILENAME);

	if(image.empty()) {
		std::cout << "image could not be loaded\n";
		return;
	}

	ArucoMarkerDetection detection;
	vector<double> data;

	if(!detection.detect(image, data)) {
		std::cout << "[test] could not detect the aruco marker.\n";
	}

	std::cout << "position " << data[0] << " " << data[1] << "\n";
	ASSERT_TRUE(std::abs(data[0] - 428.495) < 1);
	ASSERT_TRUE(std::abs(data[1] - 551.998) < 1);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
