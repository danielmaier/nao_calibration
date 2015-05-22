/*
 * CheckerboardContext.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/common/CheckerboardContext.h"

namespace kinematic_calibration {

CheckerboardContext::CheckerboardContext() {
	// nothing to do

}

CheckerboardContext::~CheckerboardContext() {
	// nothing to do
}

boost::shared_ptr<MarkerDetection> CheckerboardContext::getMarkerDetectionInstance() {
	// get parameter for the checkerboard size (default: 4x4)
	int rows = 4, columns = 4;
	double squarelength = -1.0;
	nh.param("checkerboard_rows", rows, rows);
	nh.param("checkerboard_columns", columns, columns);
	nh.param("checkerboard_squarelength", squarelength, squarelength);
    ROS_DEBUG(
			"Using a %dx%d checkerboard with square length %f (-1.0 if not set).",
			rows, columns, squarelength);
    boost::shared_ptr<CheckerboardDetection> instance (new CheckerboardDetection());
	instance->setCheckerboardSize(rows, columns);
	instance->setSquareLength(squarelength);
	return instance;
	//return new RosCheckerboardDetection(50.0);
}

inline g2o::OptimizableGraph::Edge* CheckerboardContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	string marker_optimization_type = "single_point";
	nh.param("marker_optimization_type", marker_optimization_type,
			marker_optimization_type);
	if (marker_optimization_type == "full_pose") {
		return new CheckerboardPoseMeasurementEdge(m, frameImageConverter,
				kinematicChain);
	} else {
		return new CheckerboardMeasurementEdge(m, frameImageConverter,
				kinematicChain);
	}
}

} /* namespace kinematic_calibration */
