/*
 * ImagePreprocessingNode.h
 *
 *  Created on: 09.12.2013
 *      Author: stefan
 */

#ifndef IMAGEPREPROCESSINGNODE_H_
#define IMAGEPREPROCESSINGNODE_H_

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <opencv2/core/core.hpp>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <string>

using namespace std;

namespace kinematic_calibration {

/**
 * Node for preprocessing the camera image.
 */
class ImagePreprocessingNode {
public:
	ImagePreprocessingNode();
	virtual ~ImagePreprocessingNode();

protected:
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	unsigned char crop(int value) const;

private:
	ros::NodeHandle nh, nhPrivate;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it;
	image_transport::Publisher pub;
	string inTopic, outTopic;
	bool differenceInitialized;
	cv::Mat difference;

};

} /* namespace kinematic_calibration */

#endif /* IMAGEPREPROCESSINGNODE_H_ */
