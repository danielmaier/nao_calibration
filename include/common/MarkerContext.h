/*
 * MarkerContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef ABSTRACTCONTEXT_H_
#define ABSTRACTCONTEXT_H_

#include <g2o/core/optimizable_graph.h>
#include <kinematic_calibration/measurementData.h>
#include <ros/node_handle.h>

#include "../data_capturing/MarkerDetection.h"

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"

namespace kinematic_calibration {

using namespace ros;

/**
 * Class providing strategy, factory and other methods for a specific marker type.
 */
class MarkerContext {
public:
	/**
	 * Constructor.
	 */
	MarkerContext();

	/**
	 * Destructor.
	 */
	virtual ~MarkerContext();

	/**
	 * Factory to create an instance for detecting a specific marker.
	 * Note: This method creates a new instance on the heap with every call.
	 * It is the users responsibility to handle the deletion of the instance.
	 * @return an instance for detecting a specific marker
	 */
    virtual boost::shared_ptr<MarkerDetection> getMarkerDetectionInstance() = 0;

	/**
	 * Factory method to create an edge instance for optimization with a specific marker
	 * using the g2o framework.
	 * Note: This method creates a new instance on the heap with every call.
	 * It is the users responsibility to handle the deletion of the instance.
	 * @param m Measurement data.
	 * @param frameImageConverter Instance for 3D -> 2D projection.
	 * @param kinematicChain Kinematic chain for which the measurement was taken.
	 * @return edge instance for optimization with a specific marker
	 */
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(
			const measurementData& m, FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) = 0;

protected:
	NodeHandle nh;
};

} /* namespace kinematic_calibration */

#endif /* ABSTRACTCONTEXT_H_ */
