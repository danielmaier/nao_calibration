/*
 * ArucoContext.cpp
 *
 *  Created on: 14.04.2014
 *      Author: stefan
 */

#include "../../include/common/ArucoContext.h"

namespace kinematic_calibration {

ArucoContext::ArucoContext() {
	// nothing to do
}

ArucoContext::~ArucoContext() {
	// nothing to do
}

inline boost::shared_ptr<MarkerDetection> ArucoContext::getMarkerDetectionInstance() {
	double markerSize = -1.0;
	nh.param("aruco_markersize", markerSize, markerSize);
	ROS_INFO("Using an aruco marker with side length %f (-1.0 if not set).",
			markerSize);
    boost::shared_ptr<ArucoMarkerDetection> instance (new ArucoMarkerDetection());
	instance->setMarkerSize(markerSize);
	return instance;
}

inline g2o::OptimizableGraph::Edge* ArucoContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	string marker_optimization_type = "single_point";
	nh.param("marker_optimization_type", marker_optimization_type,
			marker_optimization_type);
	if (marker_optimization_type == "full_pose") {
		return new ArucoPoseMeasurementEdge(m, frameImageConverter,
				kinematicChain);
	} else {
		return new CheckerboardMeasurementEdge(m, frameImageConverter,
				kinematicChain);
	}
}

} /* namespace kinematic_calibration */
