/*
 * KinematicChainTfComparison.h
 *
 *  Created on: 01.12.2013
 *      Author: stefan
 */

#ifndef KINEMATICCHAINTFCOMPARISON_H_
#define KINEMATICCHAINTFCOMPARISON_H_

//#include <kdl/tree.hpp>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include "../../include/common/KinematicChain.h"
#include "../../include/common/ModelLoader.h"

namespace kinematic_calibration {
class KinematicChain;
} /* namespace kinematic_calibration */

using namespace tf;
using namespace ros;

namespace kinematic_calibration {

class KinematicChainTfComparison {
public:
	KinematicChainTfComparison(string root = "CameraBottom_frame", string tip = "LWristYaw_link");

	virtual ~KinematicChainTfComparison();

	void jsCb(const sensor_msgs::JointStateConstPtr& js);

private:
	NodeHandle nh;
	TransformListener tfListener;
	Subscriber jsSubscriber;
	string root, tip;
	KinematicChain* kinematicChain;
};

}

#endif /* KINEMATICCHAINTFCOMPARISON_H_ */
