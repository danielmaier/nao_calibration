/*
 * JointOffsetOptimization.h
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSETOPTIMIZATION_H_
#define JOINTOFFSETOPTIMIZATION_H_

#include <kinematic_calibration/measurementData.h>
//#include <map>
//#include <string>
#include <vector>

#include <optimization/CalibrationState.h>
#include "../../include/common/KinematicChain.h"
#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/CalibrationContext.h"

using namespace std;

namespace kinematic_calibration {

/**
 * Abstract class for optimizing the joint offsets (and marker transformation).
 */
class JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	JointOffsetOptimization(CalibrationContext& context, vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter,
			CalibrationState initialState = CalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~JointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
	 */
    virtual int optimize(CalibrationState& optimizedState) = 0;

protected:
	/**
	 * Measurements of joint state and marker position (2D).
	 */
	vector<measurementData>& measurements;

	/**
	 * Kinematic chains for which the joint offsets should be optimized.
	 */
	vector<KinematicChain> kinematicChains;

	/**
	 * Conversion of 3D transformation into 2D image coordinates.
	 */
	FrameImageConverter& frameImageConverter;

	/**
	 * Initial state for the optimization;
	 */
	CalibrationState initialState;

	/**
	 * Calibration context instance.
	 */
	CalibrationContext& context;
};

} /* namespace kinematic_calibration */
#endif /* JOINTOFFSETOPTIMIZATION_H_ */
