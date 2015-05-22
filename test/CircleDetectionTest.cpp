/*
 * CircleDetectionTest.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../include/data_capturing/CircleDetection.h"

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/opencv.hpp>
#include <cstddef>

namespace kinematic_calibration {

TEST(CircleDetectionTest, test1) {
	// arrange: create image containing one circle
	cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
	cv::circle(image, cv::Point(200, 200), 25, cv::Scalar(0, 0, 255), -1);
	CircleDetection cd;
	std::vector<cv::Vec3f> out;

	// act
	cd.detect(image, out);

	// assert
	ASSERT_EQ(1, out.size());
	ASSERT_TRUE(fabs(200 - out[0][0]) < 5);
	ASSERT_TRUE(fabs(200 - out[0][1]) < 5);
	ASSERT_TRUE(fabs(25 - out[0][2]) < 5);
}

TEST(CircleDetectionTest, test2) {
	// arrange: create image containing one circle
	cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
	cv::circle(image, cv::Point(200, 200), 25, cv::Scalar(0, 0, 255), -1);
	cv::circle(image, cv::Point(300, 300), 25, cv::Scalar(255, 0, 0), -1);
	CircleDetection cd;
	std::vector<double> out;

	// act
	cd.detect(image, cv::Scalar(255, 0, 0), out);

	// assert
	ASSERT_TRUE(fabs(200 - out[0]) < 5);
	ASSERT_TRUE(fabs(200 - out[1]) < 5);
	ASSERT_TRUE(fabs(25 - out[2]) < 5);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
