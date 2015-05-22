/*
 * ImagePreprocessingNode.cpp
 *
 *  Created on: 09.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/ImagePreprocessingNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <ros/init.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;

namespace kinematic_calibration {

ImagePreprocessingNode::ImagePreprocessingNode() :
		it(nh), differenceInitialized(false), nhPrivate("~") {
	nhPrivate.getParam("in_topic", inTopic);
	nhPrivate.getParam("out_topic", outTopic);
	sub = it.subscribe(inTopic, 1, &ImagePreprocessingNode::imageCb, this);
	pub = it.advertise(outTopic, 1);
}

ImagePreprocessingNode::~ImagePreprocessingNode() {
	// TODO Auto-generated destructor stub
}

void ImagePreprocessingNode::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	std::cout << "Image received.\n";

	// initialize difference image if not yet done
	int difference_b, difference_g, difference_r;
	cv::Mat differenceMat = input_bridge->image.clone();
	int cols = input_bridge->image.cols, rows = input_bridge->image.rows;
	if (!differenceInitialized) {
		difference = input_bridge->image.clone();
		int x = input_bridge->image.cols / 2;
		int y = input_bridge->image.rows / 2;
		uchar value_b = input_bridge->image.ptr<uchar>(y)[3 * x + 0];
		uchar value_g = input_bridge->image.ptr<uchar>(y)[3 * x + 1];
		uchar value_r = input_bridge->image.ptr<uchar>(y)[3 * x + 2];
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				difference_b = value_b
						- input_bridge->image.ptr<uchar>(i)[3 * j + 0];
				difference_g = value_g
						- input_bridge->image.ptr<uchar>(i)[3 * j + 1];
				difference_r = value_r
						- input_bridge->image.ptr<uchar>(i)[3 * j + 2];
				difference.ptr<uchar>(i)[3 * j + 0] = crop(difference_b);
				difference.ptr<uchar>(i)[3 * j + 1] = crop(difference_g);
				difference.ptr<uchar>(i)[3 * j + 2] = crop(difference_r);
			}
		}
		differenceInitialized = true;
		std::cout << "difference initialized.\n";
	}

	// add the difference image to the received image
	differenceMat = difference.clone();
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int b = input_bridge->image.ptr<uchar>(i)[3 * j + 0];
			int g = input_bridge->image.ptr<uchar>(i)[3 * j + 1];
			int r = input_bridge->image.ptr<uchar>(i)[3 * j + 2];
			int bdiff = differenceMat.ptr<uchar>(i)[3 * j + 0];
			int gdiff = differenceMat.ptr<uchar>(i)[3 * j + 1];
			int rdiff = differenceMat.ptr<uchar>(i)[3 * j + 2];
			input_bridge->image.ptr<uchar>(i)[3 * j + 0] = crop(b + bdiff);
			input_bridge->image.ptr<uchar>(i)[3 * j + 1] = crop(g + gdiff);
			input_bridge->image.ptr<uchar>(i)[3 * j + 2] = crop(r + rdiff);
		}
	}

	// publish the non-vignetted image
	std::cout << "publishing image.\n";
	pub.publish(input_bridge->toImageMsg());
}

unsigned char ImagePreprocessingNode::crop(int value) const {
	if (value < 0) {
		return 0;
	} else if (value > 255) {
		return 255;
	} else {
		return (unsigned char) value;
	}
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "imagePreprocessingNode");
	ImagePreprocessingNode node;
	ros::spin();
	return 0;
}
