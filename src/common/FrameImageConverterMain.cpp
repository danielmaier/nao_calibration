/*
 * FrameImageConverterMain.cpp
 *
 *  Created on: 17.11.2013
 *      Author: stefan
 */

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
//#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <kdl/tree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/common/ModelLoader.h"

using namespace kinematic_calibration;
using namespace std;
using namespace ros;

image_transport::Subscriber cameraSub;
ros::Subscriber infoSub;
image_transport::Publisher pub_;
ros::Subscriber jointSub;
map<string, double> jointPositions;
image_geometry::PinholeCameraModel cameraModel;

void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
void infoCb(const sensor_msgs::CameraInfoConstPtr& image_msg);
void jointsCb(const sensor_msgs::JointStateConstPtr& msg);

int main(int argc, char** argv) {
	ros::init(argc, argv, "FrameImageConverterMain");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);

	infoSub = nh_.subscribe("/nao_camera/camera_info", 1, infoCb);

	cameraSub = it_.subscribe("/nao_camera/image_raw", 1, imageCb);
	pub_ = it_.advertise("image_out", 1);

	jointSub = nh_.subscribe("/joint_states", 1, jointsCb);

	ros::spin();
}

void jointsCb(const sensor_msgs::JointStateConstPtr& msg) {
	ROS_INFO("Joint states message received.");

	if (!jointPositions.empty())
		return;

	for (int i = 0; i < msg->name.size(); i++) {
		jointPositions.insert(
				make_pair<string, double>(msg->name[i], msg->position[i]));
	}

	jointSub.shutdown();
}

void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg) {
	ROS_INFO("Info message received.");
	cameraModel.fromCameraInfo(info_msg);
	infoSub.shutdown();
}

void imageCb(const sensor_msgs::ImageConstPtr& image_msg) {
	ROS_INFO("Image message received.");

	if (!cameraModel.initialized())
		return;

	if (jointPositions.empty())
		return;

	ModelLoader modelLoader;
	modelLoader.initializeFromRos();
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);
	//KinematicChain kinematicChain(tree, "CameraTop_frame", "l_wrist", "l_arm");
	KinematicChain kinematicChain(tree, "CameraTop_frame", "l_gripper", "l_arm");

	cv::Mat image;
	cv_bridge::CvImagePtr input_bridge;
	try {
		input_bridge = cv_bridge::toCvCopy(image_msg,
				sensor_msgs::image_encodings::BGR8);
		image = input_bridge->image;
	} catch (cv_bridge::Exception& ex) {
		ROS_ERROR("[draw_frames] Failed to convert image");
		return;
	}

	FrameImageConverter frameImageConverter(cameraModel);

	tf::Transform transform;
	double x, y;
	kinematicChain.getRootToTip(jointPositions, transform);
	frameImageConverter.project(transform, x, y);
	ROS_INFO("(x,y): %f, %f", x, y);
	cv::Point2d uv(x, y);

	static const int RADIUS = 3;
	cv::circle(image, uv, RADIUS*10, CV_RGB(255, 0, 0), -1);
	CvSize text_size;
	text_size.height = 10;
	int baseline;
	CvFont font_;
	cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
	cvGetTextSize("wrist", &font_, &text_size, &baseline);
	CvPoint origin = cvPoint(uv.x - text_size.width / 2,
			uv.y - RADIUS - baseline - 3);
	cv: putText(image, "wrist", origin, cv::FONT_HERSHEY_SIMPLEX, 12,
			CV_RGB(255, 0, 0));

	pub_.publish(input_bridge->toImageMsg());
}

