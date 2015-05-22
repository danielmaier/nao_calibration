/*
 * LocOptKinCalState.h
 *
 *  Created on: 15.01.2014
 *      Author: stefan
 */

#ifndef LOCOPTKINCALSTATE_H_
#define LOCOPTKINCALSTATE_H_

#include <g2o/core/hyper_graph.h>
#include <kinematic_calibration/measurementData.h>
#include <map>
#include <string>
#include <vector>

#include "../common/FrameImageConverter.h"
#include "../common/KinematicChain.h"
#include "../common/CalibrationContext.h"
#include "../optimization/JointOffsetVertex.h"
#include "../optimization/CameraIntrinsicsVertex.h"
#include "../optimization/MeasurementEdge.h"
#include <optimization/CalibrationState.h>

#include "g2o/types/slam3d/vertex_se3.h"

namespace kinematic_calibration {

using namespace std;

/**
 * Extension to KinematicCalibrationState, thought
 * for use with local optimization algorithms.
 */
class LocOptKinCalState: public CalibrationState {
public:
	/**
	 * Constructor that initializes the graph.
	 * @param context the context
	 * @param initialState initial state
	 */
	LocOptKinCalState(const CalibrationContext& context,
			CalibrationState& initialState,
			vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter);

	/**
	 * Constructor that uses an already built graph.
	 * @param context the context
	 * @param initialState initial state
	 * @param delta current delta
	 * @param edges edges of the graph
	 * @param vertices vertices of the graph
	 */
	LocOptKinCalState(const CalibrationContext& context,
			CalibrationState& initialState, double* delta,
			int deltaSize, vector<g2o::OptimizableGraph::Edge*> edges,
			map<string, g2o::OptimizableGraph::Vertex*> vertices,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter);
	/**
	 * Destructor.
	 */
	virtual ~LocOptKinCalState();

	/**
	 * Returns the error of the state.
	 * @return the error of the state
	 */
	double getError();

	/**
	 * Returns a list of neighbor states.
	 * @param step current "precision"
	 * @return a list of successor states
	 */
	vector<boost::shared_ptr<LocOptKinCalState> > getNeighborStates(double step);

	CalibrationState getResult() const;

protected:
	const CalibrationContext& context;
	CalibrationState& initialState;
	vector<KinematicChain> kinematicChains;
	FrameImageConverter& frameImageConverter;
	vector<g2o::OptimizableGraph::Edge*> edges;
	map<string, g2o::OptimizableGraph::Vertex*> vertices;
	double error;
	int deltaSize;
	double* delta;

	/**
	 * Builds the graph (edges, vertices) using the initialState.
	 */
	void buildGraph(const CalibrationContext& context,
			const CalibrationState& initialState,
			vector<measurementData>& measurements);

	/**
	 * initializes the delta array
	 */
	void initializeDelta();

	/**
	 * update the error
	 */
	void updateError();

	/**
	 * update the state
	 */
	void updateState();
};

} /* namespace kinematic_calibration */

#endif /* LOCOPTKINCALSTATE_H_ */
