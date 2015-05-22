/*
 * LocOptKinCalState.cpp
 *
 *  Created on: 15.01.2014
 *      Author: stefan
 */

#include "../../include/optimization/LocOptKinCalState.h"

#include <bits/streambuf_iterator.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/base_vertex.h>
#include <image_geometry/pinhole_camera_model.h>
#include <kdl/frames.hpp>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <algorithm>
#include <cstring>
#include <utility>

#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include <optimization/CalibrationState.h>
#include "../../include/optimization/MeasurementEdge.h"

namespace kinematic_calibration {

LocOptKinCalState::LocOptKinCalState(const CalibrationContext& context,
		CalibrationState& initialState,
		vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter) :
		context(context), initialState(initialState), edges(
				vector<g2o::OptimizableGraph::Edge*>()), vertices(
				map<string, g2o::OptimizableGraph::Vertex*>()), delta(NULL), kinematicChains(
				kinematicChains), frameImageConverter(frameImageConverter) {

	// initially build the graph
	buildGraph(context, initialState, measurements);

	// initialize the delta vector
	initializeDelta();

	// update the error
	updateError();

	// update the state
	updateState();
}

LocOptKinCalState::LocOptKinCalState(const CalibrationContext& context,
		CalibrationState& initialState, double* delta,
		int deltaSize, vector<g2o::OptimizableGraph::Edge*> edges,
		map<string, g2o::OptimizableGraph::Vertex*> vertices,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter) :
		context(context), initialState(initialState), edges(edges), vertices(
				vertices), delta(delta), deltaSize(deltaSize), kinematicChains(
				kinematicChains), frameImageConverter(frameImageConverter) {
	// update the error
	updateError();

	// update the state
	updateState();
}

void LocOptKinCalState::initializeDelta() {
	// get the dimension
	int size = 0;
	for (map<string, g2o::OptimizableGraph::Vertex*>::iterator it =
			this->vertices.begin(); it != this->vertices.end(); it++) {
		string key = it->first;
		if (!it->second->fixed()) {
			size += it->second->dimension();
		}
	}
	// allocate the space
	delta = (double*) std::malloc(size * sizeof(double));

	if (NULL == delta) {
		throw "Could not allocate space for the delta array!";
	}

	// initialize with zero
	for (int i = 0; i < size; i++)
		delta[i] = 0.0;

	deltaSize = size;
}

void LocOptKinCalState::updateError() {
	int idx = 0;
	error = 0;

	// set the deltas to the vertices
	for (map<string, g2o::OptimizableGraph::Vertex*>::iterator it =
			this->vertices.begin(); it != this->vertices.end(); it++) {
		string key = it->first;
		if (!it->second->fixed()) {
			string prefix = "MarkerTransformation_";
			if (key.substr(0, prefix.size()) == prefix) {
				// set marker transformation manually to initial value
				string chain_name = key.substr(prefix.size());
				tf::Transform tfTransformation =
						this->initialState.markerTransformations[chain_name];
				Eigen::Affine3d eigenAffine;
				Eigen::Isometry3d eigenIsometry;
				tf::transformTFToEigen(tfTransformation, eigenAffine);
				eigenIsometry.translation() = eigenAffine.translation();
				eigenIsometry.linear() = eigenAffine.rotation();;
				static_cast<VertexSE3*>(it->second)->setEstimate(
						eigenIsometry);
			} else if ("CameraTransformation" == key) {
				// set camera transformation manually to initial value
				Eigen::Affine3d initialCameraToHeadAffine;
                tf::transformTFToEigen(this->initialState.cameraTransformation,
						initialCameraToHeadAffine);
				Eigen::Isometry3d initialCameraToHeadIsometry;
				initialCameraToHeadIsometry.translation() =
						initialCameraToHeadAffine.translation();
				initialCameraToHeadIsometry.linear() = initialCameraToHeadAffine.rotation();
				static_cast<VertexSE3*>(it->second)->setEstimate(initialCameraToHeadIsometry);


			} else {
				// reset the vertex value to its initial value
				it->second->setToOrigin();
			}
			// add the current delta
			it->second->oplus(&delta[idx]);
			idx += it->second->dimension();
		}
	}

	// get the errors from the edges
	for (int i = 0; i < this->edges.size(); i++) {
		edges[i]->computeError(); cout << i << " " << endl;
		error += edges[i]->chi2(); cout << i << " " << endl;
	}
	cout << "error " << error << endl;
}

void LocOptKinCalState::updateState() {
	// iterate through all vertices
	for (map<string, g2o::OptimizableGraph::Vertex*>::iterator it =
			this->vertices.begin(); it != this->vertices.end(); it++) {
		string key = it->first;
		g2o::OptimizableGraph::Vertex* vertex = it->second;

		// check which type of vertex we have
		if ("JointOffset" == key) {
			this->jointOffsets =
					static_cast<JointOffsetVertex*>(vertex)->estimate();
		} else if ("CameraTransformation" == key) {
			Eigen::Isometry3d eigenTransform =
					static_cast<VertexSE3*>(vertex)->estimate();
			tf::transformEigenToTF(eigenTransform,
                    this->cameraTransformation);
		} else if ("CameraIntrinsics" == key) {
			this->cameraInfo =
					static_cast<CameraIntrinsicsVertex*>(vertex)->estimate();
		}
		// "MarkerTransformation_" + kinematicChains[i].getName()
		string prefix = "MarkerTransformation_";
		if (key.substr(0, prefix.size()) == prefix) {
			string chain_name = key.substr(prefix.size());
			Eigen::Isometry3d eigenTransform =
					static_cast<VertexSE3*>(vertex)->estimate();
			tf::Transform tfTransform;
			tf::transformEigenToTF(eigenTransform, tfTransform);
			this->markerTransformations[chain_name] = tfTransform;
		}

		// "JointTransformation_" + it->first
		prefix = "JointTransformation_";
		if (key.substr(0, prefix.size()) == prefix) {
			string joint_name = key.substr(prefix.size());
			Eigen::Isometry3d eigenTransform =
					static_cast<VertexSE3*>(vertex)->estimate();
			tf::Transform tfTransform;
			tf::transformEigenToTF(eigenTransform, tfTransform);
			this->jointTransformations[joint_name] = tfTransform;
		}
	}
}

double LocOptKinCalState::getError() {
	return error;
}

vector<boost::shared_ptr<LocOptKinCalState> > LocOptKinCalState::getNeighborStates(
		double step) {
	vector<boost::shared_ptr<LocOptKinCalState> > states;

	// add
	for (int i = 0; i < deltaSize; i++) {
		// allocate new delta
		double* newDelta = (double*) malloc(deltaSize * sizeof(double));
		memcpy(newDelta, delta, deltaSize);
		newDelta[i] = delta[i] + step;
		// create new state
		boost::shared_ptr<LocOptKinCalState> ptr(
				new LocOptKinCalState(context, initialState, newDelta,
						deltaSize, edges, vertices, kinematicChains,
						frameImageConverter));
		states.push_back(ptr);
	}

	// subtract
	for (int i = 0; i < deltaSize; i++) {
		// allocate new delta
		double* newDelta = (double*) malloc(deltaSize * sizeof(double));
		memcpy(newDelta, delta, deltaSize);
		newDelta[i] = delta[i] - step;
		// create new state
		boost::shared_ptr<LocOptKinCalState> ptr(
				new LocOptKinCalState(context, initialState, newDelta,
						deltaSize, edges, vertices, kinematicChains,
						frameImageConverter));
		states.push_back(ptr);
	}

	return states;
}

CalibrationState LocOptKinCalState::getResult() const {
	CalibrationState state(*this);
	return state;
}

void LocOptKinCalState::buildGraph(const CalibrationContext& context,
		const CalibrationState& initialState,
		vector<measurementData>& measurements) {
	// get the options
	CalibrationOptions options = context.getCalibrationOptions();

	int id = 0;

	// build the graph:

	// instantiate the vertex for the joint offsets
	vector<string> jointNames;
	for (int i = 0; i < kinematicChains.size(); i++) {
		vector<string> currentNames;
		kinematicChains[i].getJointNames(currentNames);
		// add joint names if not already contained
		for (int j = 0; j < currentNames.size() - 1; j++) {
			if (find(jointNames.begin(), jointNames.end(), currentNames[j])
					== jointNames.end()) {
				jointNames.push_back(currentNames[j]);
			}

		}
	}
	JointOffsetVertex* jointOffsetVertex = new JointOffsetVertex(jointNames);
	//jointOffsetVertex->setEstimate(initialState.jointOffsets);
	jointOffsetVertex->setId(++id);
	jointOffsetVertex->setFixed(!options.calibrateJointOffsets);
	//jointOffsetVertex->setFixed(true);
	vertices["JointOffset"] = jointOffsetVertex;

	// instantiate the vertices for the marker transformations
	map<string, VertexSE3*> markerTransformationVertices;
	map<string, KinematicChain> kinematicChainsMap;
	for (int i = 0; i < kinematicChains.size(); i++) {
		VertexSE3* markerTransformationVertex = new VertexSE3();
		markerTransformationVertex->setId(++id);
		markerTransformationVertex->setFixed(!options.calibrateMarkerTransform);
		vertices["MarkerTransformation_" + kinematicChains[i].getName()] =
				markerTransformationVertex;
		KinematicChain currentChain = kinematicChains[i];
		markerTransformationVertices.insert(
				make_pair(currentChain.getName(), markerTransformationVertex));
		kinematicChainsMap.insert(
				make_pair(currentChain.getName(), currentChain));
	}

	// instantiate the vertex for the camera transformation
	VertexSE3* cameraToHeadTransformationVertex = new VertexSE3();
	cameraToHeadTransformationVertex->setId(++id);
	cameraToHeadTransformationVertex->setFixed(
			!options.calibrateCameraTransform);

	Eigen::Affine3d initialCameraToHeadAffine;
    tf::transformTFToEigen(this->initialState.cameraTransformation,
			initialCameraToHeadAffine);
	Eigen::Isometry3d initialCameraToHeadIsometry;
	initialCameraToHeadIsometry.translation() =
			initialCameraToHeadAffine.translation();
	initialCameraToHeadIsometry.linear() = initialCameraToHeadAffine.rotation();
	cameraToHeadTransformationVertex->setEstimate(initialCameraToHeadIsometry);
	vertices["CameraTransformation"] = cameraToHeadTransformationVertex;

	// instantiate the vertex for the camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex = new CameraIntrinsicsVertex(
			frameImageConverter.getCameraModel().cameraInfo());
	cameraIntrinsicsVertex->setId(++id);
	cameraIntrinsicsVertex->setFixed(!options.calibrateCameraIntrinsics);
	cameraIntrinsicsVertex->setToOrigin();
	vertices["CameraIntrinsics"] = cameraIntrinsicsVertex;

	// instantiate the vertices for the joint 6D transformations
	map<string, KDL::Frame> framesToTip; // contains all frames (=transformations) to optimize
	for (int i = 0; i < kinematicChains.size(); i++) {
		map<string, KDL::Frame> current = kinematicChains[i].getFramesToTip();
		framesToTip.insert(current.begin(), current.end());
	}
	map<string, g2o::HyperGraph::Vertex*> jointFrameVertices;
	for (map<string, KDL::Frame>::iterator it = framesToTip.begin();
			it != framesToTip.end(); it++) {
		VertexSE3* vertex = new VertexSE3();
		Eigen::Affine3d eigenAffine;
		tf::transformKDLToEigen(it->second, eigenAffine);
		Eigen::Isometry3d eigenIsometry;
		eigenIsometry.translation() = eigenAffine.translation();
		eigenIsometry.linear() = eigenAffine.rotation();
		vertex->setEstimate(eigenIsometry);
		vertex->setId(++id);
		vertex->setFixed(true); // TODO: parameterize!
		jointFrameVertices[it->first] = vertex;
		vertices["JointTransformation_" + it->first] = vertex;
	}

	// add edges
	for (int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];
		if (markerTransformationVertices.count(current.chain_name) == 0
				|| kinematicChainsMap.count(current.chain_name) == 0) {
			ROS_ERROR("Measurement for unknown kinematic chain: %s",
					current.chain_name.c_str());
			continue;
		}
		g2o::OptimizableGraph::Edge* edge = context.getMeasurementEdge(current,
				&frameImageConverter,
				&kinematicChainsMap.find(current.chain_name)->second);
		edge->setId(++id);
		edge->vertices()[0] = markerTransformationVertices[current.chain_name];
		edge->vertices()[1] = jointOffsetVertex;
		edge->vertices()[2] = cameraToHeadTransformationVertex;
		edge->vertices()[3] = cameraIntrinsicsVertex;
		for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
				jointFrameVertices.begin(); it != jointFrameVertices.end();
				it++) {
			dynamic_cast<EdgeWithJointFrameVertices*>(edge)->setJointFrameVertex(
					it->first, it->second);
		}
		edge->computeError();
		edges.push_back(edge);
	}
}

LocOptKinCalState::~LocOptKinCalState() {
	if (delta)
		free(delta);
}

} /* namespace kinematic_calibration */

