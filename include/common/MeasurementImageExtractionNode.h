/*
 * MeasurementImageExtractionNode.h
 *
 *  Created on: 15.04.2014
 *      Author: stefan
 */

#ifndef MEASUREMENTIMAGEEXTRACTIONNODE_H_
#define MEASUREMENTIMAGEEXTRACTIONNODE_H_

#include <kinematic_calibration/measurementData.h>
#include <opencv2/core/core.hpp>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <string>

namespace kinematic_calibration {

using namespace std;
using namespace cv;
using namespace ros;

/**
 * Node which collects measurement messages
 * and writes the contained images to the disk.
 */
class MeasurementImageExtractionNode {
public:
	/**
	 * Constructor.
	 */
	MeasurementImageExtractionNode();

	/**
	 * Destructor.
	 */
	virtual ~MeasurementImageExtractionNode();

	/**
	 * Main loop that waits for measurements and writes the images to the hard disk.
	 */
	void run();

	/**
	 * Sets the out folder.
	 * @param outFolder The out folder.
	 */
	void setOutFolder(const string& outFolder);

protected:
	/**
	 * Callback for measurement messages.
	 */
	void measurementCb(const measurementDataConstPtr& msg);

	/**
	 * Writes the image to the hard disk.
	 * @param[in] image The image to be written to the hard disk.
	 * @param[in] name The filename.
	 */
	void writeImage(const Mat& image, const string& name);

	/**
	 * Converts the ROS image message to a OpenCV image.
	 * @param[in] in_msg The ROS image message to convert.
	 * @param[out] out_image The resulting OpenCV image.
	 */
	void msgToImg(const sensor_msgs::Image& in_msg, cv::Mat& out_image);

	/**
	 * Output folder for the measurement images.
	 */
	string outFolder;

	/**
	 * Node handle instance.
	 */
	NodeHandle nh;

	/**
	 * Subscriber instance for measurements.
	 */
	Subscriber measurementSubscriber;
};

} /* namespace kinematic_calibration */

#endif /* MEASUREMENTIMAGEEXTRACTIONNODE_H_ */
