/*
 * CalibrationContext.cpp
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */

#include "../../include/common/CalibrationContext.h"

#include "../../include/common/CheckerboardContext.h"
#include "../../include/common/CircleContext.h"
#include "../../include/common/ColorMarkerContext.h"

namespace kinematic_calibration {

CalibrationContext::CalibrationContext() {
	// TODO Auto-generated constructor stub

}

CalibrationContext::~CalibrationContext() {
	// TODO Auto-generated destructor stub
}

RosCalibContext::RosCalibContext(NodeHandle nodehandle) : nh(nodehandle) {
}

RosCalibContext::~RosCalibContext()  {
}

boost::shared_ptr<MarkerContext> RosCalibContext::getMarkerContext(const string& type) const {
	if ("checkerboard" == type) {
        return boost::shared_ptr<MarkerContext>(new CheckerboardContext());
	} else if ("circle" == type) {
        return boost::shared_ptr<MarkerContext>(new CircleContext());
	} else if ("color" == type) {
        return boost::shared_ptr<MarkerContext>(new ColorMarkerContext());
	} else {
		ROS_ERROR("Unknown marker type: %s", type.c_str());
		ROS_ERROR("Available types are: checkerboard\ncircle\ncolor");
        return boost::shared_ptr<MarkerContext>();
	}
}

g2o::OptimizableGraph::Edge* RosCalibContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) const {
    boost::shared_ptr<MarkerContext> markerContext( getMarkerContext(m.marker_type));
    if (!markerContext) {
		ROS_ERROR("Unknown marker type: %s", m.marker_type.c_str());
		return NULL;
	} else {
		g2o::OptimizableGraph::Edge* edge = markerContext->getMeasurementEdge(m,
				frameImageConverter, kinematicChain);
        //delete markerContext;
		return edge;
	}
}

CalibrationOptions RosCalibContext::getCalibrationOptions() const {
	CalibrationOptions options;
    cout << "Checking calibration options in namespace " << nh.getNamespace() << endl;
	nh.getParam("calibrate_joint_offsets", options.calibrateJointOffsets);
	nh.getParam("calibrate_camera_transform", options.calibrateCameraTransform);
	nh.getParam("calibrate_camera_intrinsics",
			options.calibrateCameraIntrinsics);
	nh.getParam("calibrate_marker_transform", options.calibrateMarkerTransform);
	nh.param("calibrate_joint_6D", options.calibrateJoint6D, false);
	options.markerOptimizationType = "single_point";
	nh.param("marker_optimization_type", options.markerOptimizationType,
			options.markerOptimizationType);
    ROS_INFO("Calibrate joints6d: %d, calibrate intrinsics %d, calibrate offsets %d", options.calibrateJoint6D, options.calibrateCameraIntrinsics, options.calibrateJointOffsets);
	return options;
}

DataCaptureOptions RosCalibContext::getDataCaptureOptions() const {
	DataCaptureOptions options;
    nh.getParam("find_marker", options.findMarker);
    nh.getParam("move_marker_to_corners",
			options.moveMarkerToCorners);
	return options;
}

OptimizationOptions RosCalibContext::getOptimizationOptions() const {
	OptimizationOptions options;
    cout << "Getting OptimizationOptions from namespace " << nh.getNamespace() << endl;
	nh.getParam("max_iterations", options.maxIterations);
    ROS_INFO("Using %d iterations", options.maxIterations);
	nh.param("debug_optimization", options.debug, false);
	nh.param("use_robust_kernel", options.useRobustKernel, false);
	nh.param("do_early_stopping", options.doEarlyStopping, false);
	nh.param("gain_threshold", options.gainThreshold, 1e-3);
	return options;
}

} /* namespace kinematic_calibration */

