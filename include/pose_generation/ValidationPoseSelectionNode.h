/*
 * ValidationPoseSelectionNode.h
 *
 *  Created on: 27.06.2014
 *      Author: stefan
 */

#ifndef VALIDATIONPOSESELECTIONNODE_H_
#define VALIDATIONPOSESELECTIONNODE_H_

#include "PoseSource.h"
#include "../../include/pose_generation/ValidationPoseSource.h"

#include <boost/shared_ptr.hpp>

using namespace ros;
using namespace std;
using namespace boost;

namespace kinematic_calibration {


/**
 * Pose selection node for the selection of poses for cross validation.
 */
class ValidationPoseSelectionNode {
public:
	ValidationPoseSelectionNode(int numOfPartitionsPerChain, bool use_bag=false);
	virtual ~ValidationPoseSelectionNode();
	void run();

protected:
	int numOfPartitionsPerChain;
	ros::NodeHandle nhPrivate;
	ValidationPoseSource poseSource;

};

} /* namespace kinematic_calibration */

#endif /* VALIDATIONPOSESELECTIONNODE_H_ */
