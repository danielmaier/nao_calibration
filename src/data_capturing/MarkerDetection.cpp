/*
 * MarkerDetection.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/MarkerDetection.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

namespace kinematic_calibration {

using namespace std;
namespace enc = sensor_msgs::image_encodings;

MarkerDetection::MarkerDetection() {

}

MarkerDetection::~MarkerDetection() {
}

bool MarkerDetection::writeImage(
		const string& filename) {
    cv::Mat image;
    this->getImage(image);
    this->drawMarker(image);
    if(!image.data) {
        cout << "Image data was not set!" << endl;
    }
    cout << "writing image with size " << image.rows << " x " << image.cols << endl;
    bool success = cv::imwrite(filename, image);
    if(!success) {
        cout << "Could not save the image " << filename << endl;
    }
    return success;
}

void MarkerDetection::msgToImg(const sensor_msgs::ImageConstPtr& in_msg,
		cv::Mat& out_image) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	out_image = cv_ptr->image;
}

} /* namespace kinematic_calibration */


