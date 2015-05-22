/*
 * CircleDetectionMain.cpp
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */


#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/core/types_c.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>

#include "../../include/data_capturing/CircleDetection.h"

using namespace kinematic_calibration;
using namespace std;

void imageCb(const sensor_msgs::ImageConstPtr& msg, RosCircleDetection* cd);
void detectFromRosMsg();

int main(int argc, char** argv) {
	ros::init(argc, argv, "circleDetectionMain");
	detectFromRosMsg();
	return 0;
}

image_transport::Publisher circlePub;
image_transport::Publisher cannyPub;
image_transport::Publisher gaussPub;

void imageCb(const sensor_msgs::ImageConstPtr& msg, RosCircleDetection* cd) {
    vector<double> data;

    if (!cd->detect(msg, data)) {
		std::cout << "[main] could not detect the circle.\n";
		return;
	}
	std::cout << "position " << data[0] << " " << data[1] << "\n";

	cv::Mat image;
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg,
			sensor_msgs::image_encodings::BGR8);
	image = input_bridge->image;

	cv::Point2f center(data[0], data[1]);
    int radius = (int)data[2];
    // circle center
    cv::circle( image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
    // circle outline
    cv::circle( image, center, radius, cv::Scalar(255,0,0), 3, 8, 0 );


    // publish image after applying the canny filter
    cv_bridge::CvImage cannyMsg;
    cannyMsg.header   = input_bridge->header;
    cannyMsg.encoding = "mono8";
    cannyMsg.image    = cd->getCannyImg();
    cannyPub.publish(cannyMsg.toImageMsg());

    // publish image after applying the gaussian filter
    cv_bridge::CvImage gaussMsg;
    gaussMsg.header   = input_bridge->header;
    gaussMsg.encoding = "mono8";
    gaussMsg.image    = cd->getGaussImg();
    gaussPub.publish(gaussMsg.toImageMsg());

    // publish the input image with the detected circle
	circlePub.publish(input_bridge->toImageMsg());
}

void detectFromRosMsg() {
	ros::NodeHandle nh;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it(nh);

	//AveragingCircleDetection cd;
	RosCircleDetection cd(CircleDetection::HoughTransformAdaptive);
	sub = it.subscribe("/nao_camera/image_raw", 1, boost::bind(imageCb, _1, &cd));
	circlePub = it.advertise("/circle_detection/circle", 1);
	cannyPub = it.advertise("/circle_detection/canny", 1);
	gaussPub = it.advertise("/circle_detection/gauss", 1);
	ros::spin();
}



