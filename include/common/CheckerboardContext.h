/*
 * CheckerboardContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CHECKERBOARDCONTEXT_H_
#define CHECKERBOARDCONTEXT_H_

#include "../data_capturing/CheckerboardDetection.h"
#include "MarkerContext.h"
#include <kinematic_calibration/measurementData.h>
#include "../optimization/CheckerboardMeasurementEdge.h"
#include <boost/shared_ptr.hpp>

namespace kinematic_calibration {

/**
 * Checkerboard marker context.
 */
class CheckerboardContext: public MarkerContext {
public:
	/**
	 * Constructor.
	 */
	CheckerboardContext();

	/**
	 * Desctructor.
	 */
	virtual ~CheckerboardContext();

    virtual boost::shared_ptr<MarkerDetection> getMarkerDetectionInstance();

	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(
			const measurementData& m, FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);
};

} /* namespace kinematic_calibration */

#endif /* CHECKERBOARDCONTEXT_H_ */
