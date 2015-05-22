/*
 * CheckerboardMeasurementEdge.h
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#ifndef CHECKERBOARDMEASUREMENTEDGE_H_
#define CHECKERBOARDMEASUREMENTEDGE_H_

#include <kinematic_calibration/measurementData.h>
#include <iostream>
#include <map>
#include <string>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/MeasurementEdge.h"

using namespace std;
using namespace g2o;

namespace kinematic_calibration {

/**
 * Class representing an edge between the transformation for the marker and the joint offsets.
 */
class CheckerboardMeasurementEdge: public MeasurementEdge<2,
		CheckerboardMeasurementEdge> {
public:
	/**
	 * Constructor.
	 * @param measurement Measurement represented by the edge.
	 */
	CheckerboardMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);

	/**
	 * Destructor.
	 */
	virtual ~CheckerboardMeasurementEdge();

	//using MeasurementEdge<2, CheckerboardMeasurementEdge>::getFrameImageConverter;
	//using MeasurementEdge<2, CheckerboardMeasurementEdge>::setFrameImageConverter;

	/**
	 * Sets the single error terms to the error vector.
	 * @param cameraToMarker Transformation from camera frame to marker frame
	 */
	void setError(tf::Transform cameraToMarker);


    // TODO: Check this! This is for getting the jacobian for an edge
    //void linearizeOplus() {BaseClass::linearizeOplus();}
    //void getLinearization();

};

class ArucoPoseMeasurementEdge: public MeasurementEdge<6,
ArucoPoseMeasurementEdge> {
public:
	/**
	 * Constructor.
	 * @param measurement Measurement represented by the edge.
	 */
	ArucoPoseMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);

	/**
	 * Destructor.
	 */
	virtual ~ArucoPoseMeasurementEdge();

	/**
	 * Sets the single error terms to the error vector.
	 * @param cameraToMarker Transformation from camera frame to marker frame
	 */
	void setError(tf::Transform cameraToMarker);
};

class CheckerboardPoseMeasurementEdge: public MeasurementEdge<6,
		CheckerboardPoseMeasurementEdge> {
public:
	/**
	 * Constructor.
	 * @param measurement Measurement represented by the edge.
	 */
	CheckerboardPoseMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);

	/**
	 * Destructor.
	 */
	virtual ~CheckerboardPoseMeasurementEdge();

	/**
	 * Sets the single error terms to the error vector.
	 * @param cameraToMarker Transformation from camera frame to marker frame
	 */
	void setError(tf::Transform cameraToMarker);
};

} /* namespace kinematic_calibration */

#endif /* CHECKERBOARDMEASUREMENTEDGE_H_ */
