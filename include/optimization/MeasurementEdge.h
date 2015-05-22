/*
 * MeasurementEdge.h
 *
 *  Created on: 30.12.2013
 *      Author: stefan
 */

#ifndef MEASUREMENTEDGE_H_
#define MEASUREMENTEDGE_H_

#include <g2o/core/base_multi_edge.h>
#include <g2o/core/hyper_graph.h>
#include <kdl/frames.hpp>
#include <kinematic_calibration/measurementData.h>
#include <tf/tf.h>
#include <iostream>
#include <map>
#include <string>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "JointOffsetVertex.h"

namespace kinematic_calibration {

using namespace g2o;
using namespace std;

enum JointOptimizationType {
	NONE, JOINT_OFFSETS, JOINT_6D
};

/**
 * Interface for edges having vertices of type JointFrameVertex
 */
class EdgeWithJointFrameVertices {
public:
	EdgeWithJointFrameVertices() {
	}

	virtual ~EdgeWithJointFrameVertices() {
	}

	virtual void setJointFrameVertex(const string& jointName,
			g2o::HyperGraph::Vertex* v) = 0;

	virtual void setJointOptimizationType(
			JointOptimizationType jointOptimizationType) = 0;

};

class EdgeWithJointOffsetVertexContainer {
public:
	EdgeWithJointOffsetVertexContainer() : jointOffsetVertexContainer(NULL) {
	}


	virtual ~EdgeWithJointOffsetVertexContainer() {
	}


	virtual void setJointOffsetVertexContainer(
			JointOffsetVertexContainer* jointOffsetVertexContainer) {
		if(this->jointOffsetVertexContainer) {
			delete this->jointOffsetVertexContainer;
		}
		this->jointOffsetVertexContainer = jointOffsetVertexContainer;
	}

	virtual JointOffsetVertexContainer* getJointOffsetVertexContainer(
			JointOffsetVertexContainer* jointOffsetVertexContainer) {
		return this->jointOffsetVertexContainer;
	}

protected:
	JointOffsetVertexContainer* jointOffsetVertexContainer;
};

/**
 * Abstract base class for measurement edges for g2o.
 */
template<int D, class Derived>
class MeasurementEdge: public BaseMultiEdge<D, measurementData>,
		public EdgeWithJointFrameVertices, public EdgeWithJointOffsetVertexContainer {
public:
	/**
	 * Constructor.
	 * @param measurement Measurement represented by the edge.
	 */
	MeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);

	/**
	 * Destructor.
	 */
	virtual ~MeasurementEdge() {
	}
	;

	/**
	 * Computes the error of the edge and stores it in an internal structure.
	 */
	void computeError();

	void setJointFrameVertex(const string& jointName,
			g2o::HyperGraph::Vertex* v);

	bool read(std::istream&) {
		return false;
	}
	bool write(std::ostream&) const {
		return false;
	}

	/**
	 * Returns the frame image converter.
	 * @return the frame image converter
	 */
	const FrameImageConverter* getFrameImageConverter() const;

	/**
	 * Sets the frame image converter.
	 * @param frameImageConverter the frame image converter
	 */
	void setFrameImageConverter(FrameImageConverter* frameImageConverter);

	/**
	 * Returns the kinematic chain.
	 * @return the kinematic chain
	 */
	const KinematicChain* getKinematicChain() const;

	/**
	 * Sets the kinematic chain.
	 * @param kinematicChain the kinematic chain
	 */
	void setKinematicChain(KinematicChain* kinematicChain);

	/**
	 * Sets the joint optimization type.
	 * @param jointOptimizationType The new joint optimization type.
	 */
	void setJointOptimizationType(JointOptimizationType jointOptimizationType);

	/**
	 * Gets the debug flag.
	 * @return The debug flag.
	 */
	bool isDebug() const;

	/**
	 * Sets the debug flag.
	 * @param debug The debug flag.
	 */
	void setDebug(bool debug);


    // TODO: Check this! This is for getting the jacobian for an edge
    //void linearizeOplus() { BaseMultiEdge<D, measurementData>::linearizeOplus();}
    //void getLinearization();

protected:
	/**
	 * Kinematic chain for which the joint offsets should be converted.
	 */
	KinematicChain* kinematicChain;

	/**
	 * Conversion of 3D transformation into 2D image coordinates.
	 */
	FrameImageConverter* frameImageConverter;

	/**
	 * Measurement data.
	 */
	measurementData measurement;

	/**
	 * Joint positions (extracted of the measurement data).
	 */
	map<string, double> jointPositions;

	/**
	 * Contains the indexes of the vertices for the joint frames.
	 */
	map<string, int> jointFrameVerticesIndexes;

	/**
	 * Iterates through all joint frames vertices and puts
	 * the current frames into a map and returns it.
	 * @return map of joint frames
	 */
	map<string, KDL::Frame> getJointFrames();

	/**
	 * Flag that indicates if the debug mode is on.
	 */
	bool debug;

	/**
	 * Variable that indicates how the joints should be treated.
	 */
	JointOptimizationType jointOptimizationType;
};

} /* namespace kinematic_calibration */

#include "../../src/optimization/MeasurementEdge.cpp"

#endif /* MEASUREMENTEDGE_H_ */
