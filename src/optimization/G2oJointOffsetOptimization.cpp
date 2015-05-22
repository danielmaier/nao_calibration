/*
 * G2oJointOffsetOptimization.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/G2oJointOffsetOptimization.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <cstdio>

#include "kinematic_calibration/measurementData.h"
#include "../../include/optimization/JointOffsetOptimization.h"
#include <optimization/CalibrationState.h>
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/optimization/CheckerboardMeasurementEdge.h"
#include "../../include/optimization/TransformationVertex.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include <kdl/kdl.hpp>
#include <tf_conversions/tf_kdl.h>

#include <time.h>
#include <map>

using namespace std;

namespace kinematic_calibration {

//typedef TransformationVertex MarkerTransformationVertex;
typedef TransformationVertex CameraTransformationVertex;

class SaveStateHyperGraphAction: public g2o::HyperGraphAction {
public:
	/// Constructor.
	SaveStateHyperGraphAction(vector<CalibrationState>& states,
			JointOffsetVertex* jointOffsetVertex,
			map<string, TransformationVertex*> markerTransformationVertices,
			TransformationVertex* cameraToHeadTransformationVertex,
			CameraIntrinsicsVertex* cameraIntrinsicsVertex,
			map<string, g2o::HyperGraph::Vertex*> jointFrameVertices);

	/// Desctructor.
	virtual ~SaveStateHyperGraphAction();

	/// Reimplementation of the action.
	virtual HyperGraphAction* operator()(const HyperGraph* graph,
			Parameters* parameters = 0);

	vector<CalibrationState>& states;
	JointOffsetVertex* jointOffsetVertex;
	map<string, TransformationVertex*> markerTransformationVertices;
	TransformationVertex* cameraToHeadTransformationVertex;
	CameraIntrinsicsVertex* cameraIntrinsicsVertex;
	map<string, g2o::HyperGraph::Vertex*> jointFrameVertices;
};




G2oJointOffsetOptimization::G2oJointOffsetOptimization(
		CalibrationContext& context, vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,	FrameImageConverter& frameImageConverter,
		CalibrationState initialState) :
		JointOffsetOptimization(context, measurements, kinematicChains, frameImageConverter, initialState),
		saveIntermediateStates(false) {
    /*
    ROS_INFO("G2oJointOffsetOptimization:: Setting initial joint offsets to ");
    for(map<string, double>::const_iterator it = initialState.jointOffsets.begin(); it != initialState.jointOffsets.end(); ++it)
    {
        cout << it->first << " " << it->second << endl;
    }
    */
}

G2oJointOffsetOptimization::~G2oJointOffsetOptimization() {
}

int G2oJointOffsetOptimization::optimize(
		CalibrationState& optimizedState) {
	typedef BlockSolver<BlockSolverTraits<-1, -1> > MyBlockSolver;
	//typedef BlockSolverX MyBlockSolver;
	//typedef LinearSolverDense<MatrixXd> MyLinearSolver;
	typedef LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

	// allocating the optimizer
	SparseOptimizer optimizer;

	// create the linear solver
	MyLinearSolver* linearSolver = new MyLinearSolver();
	//linearSolver->setTolerance(1e-10);
	//linearSolver->setAbsoluteTolerance(true);

	// create the block solver on top of the linear solver
	MyBlockSolver* blockSolver = new MyBlockSolver(linearSolver);
	blockSolver->setLevenberg(true);
	blockSolver->setSchur(false);
	blockSolver->setWriteDebug(true);

	// create the algorithm to carry out the optimization
//	OptimizationAlgorithmGaussNewton* algorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
	OptimizationAlgorithmLevenberg* algorithm =
			new OptimizationAlgorithmLevenberg(blockSolver);
	//algorithm->setMaxTrialsAfterFailure(100);
	//algorithm->printVerbose(cout);
	optimizer.setAlgorithm(algorithm);
	//blockSolver->init(&optimizer);

	// get the options
	CalibrationOptions options = context.getCalibrationOptions();
	OptimizationOptions optOptions = context.getOptimizationOptions();

	// compute statistics
	optimizer.setComputeBatchStatistics(true); // TODO: as parameter?

	int id = 0;

	// build the graph:

	// determine the joint optimization type
	JointOptimizationType jointOptType;
	if (options.calibrateJoint6D)
		jointOptType = JOINT_6D;
	else if (options.calibrateJointOffsets)
		jointOptType = JOINT_OFFSETS;
	else
		jointOptType = NONE;

	// instantiate the vertex for the joint offsets
	// TODO: as parameter / from model!
	vector<string> mimicJointsFirst, mimicJointsSecond;
	mimicJointsFirst.push_back("RHipYawPitch");
	mimicJointsSecond.push_back("LHipYawPitch");
	map<string, string> currentMimicJoints;
	vector<string> jointNames;
	int skipLast = options.calibrateMarkerTransform ? 1 : 0;
	int skipFirst = options.calibrateCameraTransform ? 1 : 0;
	for (int i = 0; i < kinematicChains.size(); i++) {
		vector<string> currentNames;
		kinematicChains[i].getJointNames(currentNames);
		// TODO: Make this nicer ( ignore certain joints like HeadPitch, not just the first one)
		for (int j = skipFirst; j < currentNames.size() - skipLast; j++) {
			string currentName = currentNames[j];
			// check if not already contained
			if (find(jointNames.begin(), jointNames.end(), currentNames[j])
					!= jointNames.end()) {
				// joint already contained
				continue;
			}

			// check if current is mimic joint
			for (int k = 0; k < mimicJointsFirst.size(); k++) {
				if (currentName == mimicJointsFirst[k]
						&& find(jointNames.begin(), jointNames.end(),
								mimicJointsSecond[k]) != jointNames.end()) {
					currentMimicJoints[mimicJointsSecond[k]] = currentName;
					continue;
				}
			}
			for (int k = 0; k < mimicJointsSecond.size(); k++) {
				if (currentName == mimicJointsSecond[k]
						&& find(jointNames.begin(), jointNames.end(),
								mimicJointsFirst[k]) != jointNames.end()) {
					currentMimicJoints[mimicJointsFirst[k]] = currentName;
					continue;
				}
			}

			// everything ok -> add the joint
			jointNames.push_back(currentNames[j]);
		}
    }
    // TODO: Iterate over initialstate and pick non mimic jointoffsets
    //JointOffsetVertex* jointOffsetVertex = new JointOffsetVertex(jointNames);
    // TODO: Check if this initialization is correct wrt. the mimic joints
    JointOffsetVertex* jointOffsetVertex = new JointOffsetVertex(initialState.jointOffsets);
    jointOffsetVertex->setEstimate(initialState.jointOffsets);
	jointOffsetVertex->setId(++id);
	jointOffsetVertex->setFixed(jointOptType != JOINT_OFFSETS);
	for (map<string, string>::iterator it = currentMimicJoints.begin();
			it != currentMimicJoints.end(); it++) {
		jointOffsetVertex->setMimicJoint(it->first, it->second);
		ROS_INFO("Mimic joints: %s <-> %s.", it->first.c_str(),
				it->second.c_str());
	}
	optimizer.addVertex(jointOffsetVertex);

	// instantiate the vertices for the marker transformations
	map<string, MarkerTransformationVertex*> markerTransformationVertices;
	map<string, KinematicChain> kinematicChainsMap;
	for (int i = 0; i < kinematicChains.size(); i++) {
		MarkerTransformationVertex* markerTransformationVertex;
		if ("single_point" == options.markerOptimizationType) {
			markerTransformationVertex = new TranslationVertex();
		} else {
			// full pose
			markerTransformationVertex = new TransformationVertex();
		}
		markerTransformationVertex->setId(++id);
		markerTransformationVertex->setFixed(!options.calibrateMarkerTransform);
		optimizer.addVertex(markerTransformationVertex);
		KinematicChain currentChain = kinematicChains[i];
		markerTransformationVertex->setEstimate(
				initialState.markerTransformations[currentChain.getName()]);
		markerTransformationVertices.insert(
				make_pair(currentChain.getName(), markerTransformationVertex));
		kinematicChainsMap.insert(
				make_pair(currentChain.getName(), currentChain));
	}

	// instantiate the vertex for the camera transformation
	CameraTransformationVertex* cameraToHeadTransformationVertex =
			new CameraTransformationVertex();
	//cameraToHeadTransformationVertex->setId(++id);
	cameraToHeadTransformationVertex->setId(++id);
	cameraToHeadTransformationVertex->setEstimate(
            this->initialState.cameraTransformation);
	cameraToHeadTransformationVertex->setFixed(
			!options.calibrateCameraTransform);
	if(optimizer.addVertex(cameraToHeadTransformationVertex))
		cout << "Vertex for CameraTransform added.\n" << endl;
	else
		cout << "NOT ADDED.\n" << endl;

	// instantiate the vertex for the camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex = new CameraIntrinsicsVertex(
			frameImageConverter.getCameraModel().cameraInfo());
	cameraIntrinsicsVertex->setId(++id);
	cameraIntrinsicsVertex->setToOrigin();
	cameraIntrinsicsVertex->setFixed(!options.calibrateCameraIntrinsics);
	optimizer.addVertex(cameraIntrinsicsVertex);

	// instantiate the vertices for the joint 6D transformations
	map<string, KDL::Frame> framesToTip; // contains all frames (=transformations) to optimize
	for (int i = 0; i < kinematicChains.size(); i++) {
		map<string, KDL::Frame> current = kinematicChains[i].getFramesToTip();
		framesToTip.insert(current.begin(), current.end());
	}
	map<string, g2o::HyperGraph::Vertex*> jointFrameVertices;
	for (map<string, KDL::Frame>::iterator it = framesToTip.begin();
			it != framesToTip.end(); it++) {
		TransformationVertex* vertex = new TransformationVertex();
		tf::Transform tfTransform;
		tf::transformKDLToTF(it->second, tfTransform);
		vertex->setEstimate(tfTransform);
		vertex->setId(++id);
		vertex->setFixed(jointOptType != JOINT_6D);
		jointFrameVertices[it->first] = vertex;
		optimizer.addVertex(vertex);
	}

//    vector<double> jointEst;
//    jointOffsetVertex->getEstimateData(jointEst);
//    cout << "JointOffset estimate size is " << jointEst.size() << endl;
//    for(int i =0; i < jointEst.size(); ++i)
//    {
//        cout << "JointOffsetVertex " << i << " : " << jointEst[i] << endl;
//    }
//    vector<double> intrinsEst;
//    cameraIntrinsicsVertex->getEstimateData(intrinsEst);
//    cout << "Intrinsics estimate size is " << intrinsEst.size() << endl;
//    for(int i =0; i < intrinsEst.size(); ++i)
//    {
//        cout << "IntrinsicsVertex" << i << " : " << intrinsEst[i] << endl;
//    }
	// add edges
    ROS_INFO("Use robust kernel is %d", optOptions.useRobustKernel);
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
		if (optOptions.useRobustKernel) {
			RobustKernel* rk = new RobustKernelHuber();
			//rk->setDelta(5.0);
			edge->setRobustKernel(rk);
		}
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
		dynamic_cast<EdgeWithJointFrameVertices*>(edge)->setJointOptimizationType(
				jointOptType);
		edge->computeError();
		optimizer.addEdge(edge);
	}
    if (false){
    // add jointOffsetConstraint edge
        //g2o::OptimizableGraph::Edge * graph_edge = new ConstrainJointOffsetsEdge();
        ConstrainJointOffsetsEdge * edge = new ConstrainJointOffsetsEdge();
        //ConstrainJointOffsetsEdge edge = graph_edge;
        edge->setId(++id);
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
        edge->setVertex(0, jointOffsetVertex);
        //edge->vertices()[0] = jointOffsetVertex;
        //edge->setMeasurementData();
        Eigen::Vector2d v;
        v[0] = 0.0;
        v[1] = 0.0;
        edge->setMeasurement(v);
        //edge->computeError();
        optimizer.addEdge(edge);
    }

	// add action to save the intermediate states
	if (saveIntermediateStates) {
		intermediateStates.push_back(this->initialState);
		optimizer.addPostIterationAction(
				new SaveStateHyperGraphAction(intermediateStates,
						jointOffsetVertex, markerTransformationVertices,
						cameraToHeadTransformationVertex,
						cameraIntrinsicsVertex, jointFrameVertices));
	}

	// early stopping
	if (optOptions.doEarlyStopping) {
		SparseOptimizerTerminateAction* terminateAction =
				new SparseOptimizerTerminateAction();
		terminateAction->setGainThreshold(optOptions.gainThreshold);
		optimizer.addPostIterationAction(terminateAction);
	}

	// optimize:
	ROS_INFO("Starting g2o optimization...");
	if(!optimizer.initializeOptimization()) {
		ROS_FATAL("Optimization could not be initialized!");
        return 0;
	}


	//optimizer.computeActiveErrors();
	optimizer.setVerbose(true);
    int cjIterations = optimizer.optimize(optOptions.maxIterations);
    ROS_INFO("Done after %d iterations!", cjIterations);

	// get results:

	// joint offsets
	optimizedState.jointOffsets =
			static_cast<map<string, double> >(jointOffsetVertex->estimate());

	// marker transformations
	for (map<string, MarkerTransformationVertex*>::iterator it =
			markerTransformationVertices.begin();
			it != markerTransformationVertices.end(); it++) {
		optimizedState.markerTransformations[it->first] =
				it->second->estimate();
	}

	// camera transformation
    optimizedState.cameraTransformation =
			cameraToHeadTransformationVertex->estimate();

	// camera intrinsics
    optimizedState.cameraInfo = cameraIntrinsicsVertex->estimate();

	// joint transformations
	map<string, tf::Transform> jointTransformations;
	for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
			jointFrameVertices.begin(); it != jointFrameVertices.end(); it++) {
		jointTransformations[it->first] =
				static_cast<TransformationVertex*>(it->second)->estimate();
	}
	optimizedState.jointTransformations = jointTransformations;

	// plot the error optimization
    //ROS_INFO("Writing chi2 plot...");
    this->statistics = optimizer.batchStatistics();
    //this->plotStatistics(statistics);

    ROS_INFO("end of optimze. about to return");
    return cjIterations;
}

BatchStatisticsContainer G2oJointOffsetOptimization::getStatistics() const {
	return this->statistics;
}

void G2oJointOffsetOptimization::setSaveIntermediateStates(
		bool saveIntermediateStates) {
	this->saveIntermediateStates = saveIntermediateStates;
}

void G2oJointOffsetOptimization::getIntermediateStates(
		vector<CalibrationState>& intermediateStates) const {
	intermediateStates = this->intermediateStates;
}

void G2oJointOffsetOptimization::plotStatistics(
		const BatchStatisticsContainer& statistics) const {
	FILE * gnuplotPipe;
	gnuplotPipe = popen("gnuplot -persistent", "w");
	fprintf(gnuplotPipe,
			"set terminal svg size 350,262 fname 'Verdana' fsize 10\n");
	fprintf(gnuplotPipe, "set output 'chi2.svg'\n");
	fprintf(gnuplotPipe, "set autoscale;\n set xzeroaxis\n set yzeroaxis\n");
	fprintf(gnuplotPipe,
			"plot '-' with linespoints pt 1 lc rgb 'red' title 'chi2' smooth csplines \n");
	;
	for (int i = 0; i < statistics.size(); i++) {
		fprintf(gnuplotPipe, "%i %f \n", statistics[i].iteration,
				statistics[i].chi2);
	}
	fprintf(gnuplotPipe, "e ,\n ");
	fflush(gnuplotPipe);
	fclose(gnuplotPipe);
}

SaveStateHyperGraphAction::SaveStateHyperGraphAction(
		vector<CalibrationState>& states,
		JointOffsetVertex* jointOffsetVertex,
		map<string, TransformationVertex*> markerTransformationVertices,
		TransformationVertex* cameraToHeadTransformationVertex,
		CameraIntrinsicsVertex* cameraIntrinsicsVertex,
		map<string, g2o::HyperGraph::Vertex*> jointFrameVertices) :
		states(states), jointOffsetVertex(jointOffsetVertex), markerTransformationVertices(
				markerTransformationVertices), cameraToHeadTransformationVertex(
				cameraToHeadTransformationVertex), cameraIntrinsicsVertex(
				cameraIntrinsicsVertex), jointFrameVertices(jointFrameVertices) {
}

SaveStateHyperGraphAction::~SaveStateHyperGraphAction() {
}

HyperGraphAction* SaveStateHyperGraphAction::operator ()(
		const HyperGraph* graph, Parameters* parameters) {
	CalibrationState state;

	// joint offsets
	state.jointOffsets =
			static_cast<map<string, double> >(jointOffsetVertex->estimate());

	// marker transformations
	for (map<string, MarkerTransformationVertex*>::iterator it =
			markerTransformationVertices.begin();
			it != markerTransformationVertices.end(); it++) {
		state.markerTransformations[it->first] =
				it->second->estimate();
	}

	// camera transformation
    state.cameraTransformation =
			cameraToHeadTransformationVertex->estimate();

	// camera intrinsics
	state.cameraInfo = cameraIntrinsicsVertex->estimate();

	// joint transformations
	map<string, tf::Transform> jointTransformations;
	for (map<string, g2o::HyperGraph::Vertex*>::iterator it =
			jointFrameVertices.begin(); it != jointFrameVertices.end(); it++) {
		jointTransformations[it->first] =
				static_cast<TransformationVertex*>(it->second)->estimate();
	}
	state.jointTransformations = jointTransformations;

	states.push_back(state);

    /*
    g2o::OptimizableGraph::Edge* edge;
    for( HyperGraph::EdgeSet::iterator it =  graph->edges().begin(); it != graph->edges().end(); ++it)
    {
        g2o::OptimizableGraph::Edge * e = ( g2o::OptimizableGraph::Edge *)(*it);
        cout << "Found an edge";
        //it->linearizeOplus();

        //edge = dynamic_cast<g2o::OptimizableGraph::Edge*> (it);
        //typedef std::set<Edge*>                           EdgeSet;
        const double * errovec = e->errorData();
        for(int i =0; i < e->dimension(); ++i)
        {
            cout << "error for dim " << i << " is ";
            cout << errovec[i] << endl;
        }
        cout << "chi2 is " << e->chi2() << endl;
        g2o::JacobianWorkspace jacws;
        jacws.updateSize(*graph);
        jacws.allocate();
        e->linearizeOplus(jacws);
        cout << "jacobian workspace:\n";
        for (int i =0; i < e->vertices().size(); ++i )
        {
            cout << "for vertex " << i << ":" << endl;
            double * workspaceForVertex = jacws.workspaceForVertex(i);
            for (int j =0; j < e->dimension(); ++j)
            {
                cout << workspaceForVertex[j] << endl;
            }
        }

    }
    */



	return this;
}

} /* namespace kinematic_calibration */

