/*
 * ArucoContext.h
 *
 *  Created on: 14.04.2014
 *      Author: stefan
 */

#ifndef ARUCOCONTEXT_H_
#define ARUCOCONTEXT_H_

#include "../data_capturing/ArucoMarkerDetection.h"
#include "MarkerContext.h"
#include <kinematic_calibration/measurementData.h>
#include "../optimization/CheckerboardMeasurementEdge.h"

namespace kinematic_calibration {

/**
 * Aruco marker context.
 */
class ArucoContext: public MarkerContext {
public:
	/**
	 * Constructor.
	 */
	ArucoContext();

	/**
	 * Desctructor.
	 */
	virtual ~ArucoContext();

    virtual boost::shared_ptr<MarkerDetection> getMarkerDetectionInstance();

	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(
			const measurementData& m, FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);
};

} /* namespace kinematic_calibration */

#endif /* ARUCOCONTEXT_H_ */
