/*
 * CalibrationContext.h
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONCONTEXT_H_
#define CALIBRATIONCONTEXT_H_

#include "MarkerContext.h"

#include <string>
#include <ros/ros.h>

using namespace std;

namespace kinematic_calibration {

// forward declarations
class CalibrationOptions;
class DataCaptureOptions;
class OptimizationOptions;

/**
 * Abstract class containing factory methods for a specific calibration context.
 */
class CalibrationContext {
public:
	/**
	 * Constructor.
	 */
	CalibrationContext();

	/**
	 * Destructor.
	 */
	virtual ~CalibrationContext();

	/**
	 * Creates and returns an instance to a marker context
	 * (i.e. factory methods for a specific marker type).
	 * NOTE: The caller is responsible for destruction of the returned object!
	 * @param type Type of the marker.
	 * @return An instance to a marker context
	 */
    virtual boost::shared_ptr<MarkerContext> getMarkerContext(const string& type) const = 0;

	/**
	 * Creates and returns the edge for the g2o graph appropriate to the measurement type.
	 * NOTE: The caller is responsible for destruction of the returned object!
	 * @param m The measurement for which the edge should be created.
	 * @param frameImageConverter The instance responsible for projecting from 3D to 2D.
	 * @param kinematicChain The respective kinematic chain.
	 * @return The edge for the g2o graph appropriate to the measurement type.
	 */
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) const = 0;

	/**
	 * Returns the general options for the calibration.
	 * @return The general options for the calibration.
	 */
	virtual CalibrationOptions getCalibrationOptions() const = 0;

	/**
	 * Returns the specific options for the data capturing for one kinematic chain.
	 * @return The specific options for the data capturing for one kinematic chain.
	 */
    virtual DataCaptureOptions getDataCaptureOptions() const = 0;

	/**
	 * Returns the options for the optimization.
	 * @return The options for the optimization.
	 */
	virtual OptimizationOptions getOptimizationOptions() const = 0;
};

/**
 * ROS specific context. Gets parameters etc. from the ROS environment.
 */
class RosCalibContext: public CalibrationContext {
public:
    RosCalibContext(NodeHandle nodeHandle=NodeHandle());
	virtual ~RosCalibContext();

	// overridden methods
    virtual boost::shared_ptr<MarkerContext> getMarkerContext(const string& type) const;
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) const;
	virtual CalibrationOptions getCalibrationOptions() const;
	virtual DataCaptureOptions getDataCaptureOptions() const;
	virtual OptimizationOptions getOptimizationOptions() const;

protected:
	ros::NodeHandle nh;
};

/**
 * Container for calibration options.
 */
class CalibrationOptions {
public:
	bool calibrateJointOffsets;
	bool calibrateCameraTransform;
	bool calibrateCameraIntrinsics;
	bool calibrateMarkerTransform;
	bool calibrateJoint6D;
	string markerOptimizationType;
};

/**
 * Container for data capture options.
 */
class DataCaptureOptions {
public:
	bool findMarker;
	bool moveMarkerToCorners;
};

/**
 * Container for optimization options.
 */
class OptimizationOptions {
public:
	int maxIterations;
	bool debug;
	bool useRobustKernel;
	bool doEarlyStopping;
	double gainThreshold;
};
} /* namespace kinematic_calibration */

#endif /* CALIBRATIONCONTEXT_H_ */
