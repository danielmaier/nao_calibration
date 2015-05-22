/*
 * JointOffsetOptimization.cpp
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#include "../../include/optimization/JointOffsetOptimization.h"

namespace kinematic_calibration {

JointOffsetOptimization::JointOffsetOptimization(CalibrationContext& context,
		vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter,
		CalibrationState initialState) :
		context(context), measurements(measurements), kinematicChains(
				kinematicChains), frameImageConverter(frameImageConverter), initialState(
				initialState) {
}

JointOffsetOptimization::~JointOffsetOptimization() {

}

} /* namespace kinematic_calibration */

