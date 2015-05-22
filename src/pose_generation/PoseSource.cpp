/*
 * PoseSource.cpp
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/PoseSource.h"

#include "../../include/pose_generation/PoseSampling.h"
#include "../../include/common/CalibrationContext.h"


using boost::shared_ptr;

namespace kinematic_calibration {


boost::shared_ptr<MeasurementPoseSet> PoseSource::getInitialPoseSet(
		const KinematicChain& kinematicChain, CalibrationState& state,
		const int& n) {
	// get poses for the current chain
	vector<MeasurementPose> poses;
	this->getPoses(kinematicChain, poses);

	// initialize the pose set
    boost::shared_ptr<MeasurementPoseSet> poseSet = make_shared<MeasurementPoseSet>(
			state);
	poseSet->addMeasurementPoses(poses);
	poseSet->initializePoseSet(n);

	return poseSet;
}





} /* namespace kinematic_calibration */
