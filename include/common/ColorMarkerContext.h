/*
 * ColorMarkerContext.h
 *
 *  Created on: 14.04.2014
 *      Author: stefan
 */

#ifndef COLORMARKERCONTEXT_H_
#define COLORMARKERCONTEXT_H_

#include "../data_capturing/ColorMarkerDetection.h"
#include "MarkerContext.h"
#include <kinematic_calibration/measurementData.h>
#include "../optimization/CheckerboardMeasurementEdge.h"

namespace kinematic_calibration {

/**
 * Color marker context.
 */
class ColorMarkerContext: public MarkerContext {
public:
	/**
	 * Constructor.
	 */
	ColorMarkerContext();

	/**
	 * Destructor.
	 */
	virtual ~ColorMarkerContext();

    virtual boost::shared_ptr<MarkerDetection> getMarkerDetectionInstance();

	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(
			const measurementData& m, FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);
};

} /* namespace kinematic_calibration */

#endif /* COLORMARKERCONTEXT_H_ */
