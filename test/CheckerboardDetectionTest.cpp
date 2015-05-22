/*
 * CheckerboardDetectionTest.cpp
 *
 *  Created on: 23.10.2013
 *      Author: stefan
 */

#include "../include/data_capturing/CheckerboardDetection.h"

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <iostream>

#define FILENAME "board_4x4.ppm"

namespace kinematic_calibration {

TEST(CheckerboardDetectionTest, detectionTest) {
	cv::Mat image;
	image = cv::imread("board_4x4.ppm");

	if(image.empty()) {
		std::cout << "image could not be loaded\n";
		return;
	}

	CheckerboardDetection detection;
	CheckerboardData data;

	if(!detection.detect(image, data)) {
		std::cout << "[test] could not detect the checkerboard.\n";
	}

	std::cout << "position " << data.x << " " << data.y << "\n";
	ASSERT_TRUE(std::abs(data.x - 311.5) < 1);
	ASSERT_TRUE(std::abs(data.y - 399.3) < 1);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
