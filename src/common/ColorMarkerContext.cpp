/*
 * ColorMarkerContext.cpp
 *
 *  Created on: 14.04.2014
 *      Author: stefan
 */

#include "../../include/common/ColorMarkerContext.h"

#ifndef PARAMSPATH
#define PARAMSPATH ""
#endif

namespace kinematic_calibration {

ColorMarkerContext::ColorMarkerContext() {
	// nothing to do
}

ColorMarkerContext::~ColorMarkerContext() {
	// nothing to do
}

inline boost::shared_ptr<MarkerDetection> ColorMarkerContext::getMarkerDetectionInstance() {
	// TODO: path as Ros parameter!
	// use hard coded path:
	stringstream userParsFileNameStream, classParsFileNameStream;
	userParsFileNameStream << PARAMSPATH << "CMUserPars.xml";
	classParsFileNameStream << PARAMSPATH << "DesignPars.xml";
	string userParsFileName = userParsFileNameStream.str();
	string classParsFileName = classParsFileNameStream.str();
    return boost::shared_ptr<MarkerDetection>(new ColorMarkerDetection(userParsFileName, classParsFileName));
}

inline g2o::OptimizableGraph::Edge* ColorMarkerContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	return new CheckerboardMeasurementEdge(m, frameImageConverter,
			kinematicChain);
}

} /* namespace kinematic_calibration */
