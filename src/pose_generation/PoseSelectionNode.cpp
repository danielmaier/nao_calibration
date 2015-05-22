/*
 * PoseSelectionNode.cpp
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/PoseSelectionNode.h"

#include <boost/smart_ptr/make_shared.hpp>
//#include <kdl/tree.hpp>
//#include <opencv/cv.h>
//#include <ros/callback_queue.h>
#include <ros/console.h>
//#include <ros/init.h>
//#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <tf/tf.h>
//#include <urdf/model.h>
//#include <urdf_model/joint.h>
//#include <urdf_model/pose.h>
#include <fstream>
#include <iostream>
#include <map>
#include <boost/algorithm/string/join.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <sys/types.h>
#include <sys/stat.h>
//#include <string>

#include "../../include/common/PoseSet.h"
#include "../../include/pose_generation/PoseSampling.h"
#include "../../include/pose_generation/PoseSelectionStrategy.h"
#include "camera_calibration_parsers/parse.h"
#include "../../include/common/CalibrationContext.h"
#include "../../include/optimization/ValidationNode.h"
#include "../../include/pose_generation/ValidationPoseSelectionNode.h"
#include "../../include/common/ContextFactory.h"

namespace kinematic_calibration {

using namespace std;

PoseSelectionNode::PoseSelectionNode(PoseSource& poseSource) :
		poseSource(poseSource), nhPrivate("~"), selectionStrategyName("optimal") {

    initialize();


    // get the selection strategy name
    nhPrivate.param("selection_strategy", selectionStrategyName,
            selectionStrategyName);
    ROS_INFO("Selection strategy is %s. (Parameter: selection_strategy)",
            selectionStrategyName.c_str());
    // TODO: inject the following instances:
	string criterion = "NAI";
	nhPrivate.param("criterion", criterion, criterion);
	if (criterion=="NAI")
      this->observabilityIndex = boost::make_shared<NoiseAmplificationIndex>();
	else if (criterion=="DET")
      this->observabilityIndex = boost::make_shared<ProductSingularValuesIndex>();
    else if (criterion=="SUM")
      this->observabilityIndex = boost::make_shared<SumSingularValuesIndex>();
    else if (criterion=="MIN")
      this->observabilityIndex = boost::make_shared<MinimumSingularValueIndex>();
	else
	{
	  ROS_ERROR("Sorry, criteria %s is not implemented", criterion.c_str());
	  throw std::runtime_error("Unsupported criterion");
	}
	ROS_INFO("With criterion : %s", criterion.c_str());

	bool scaleJacobian = true;
	nhPrivate.param("scale_jacobian", scaleJacobian, scaleJacobian);
    this->observabilityIndex->setScaleJacobian(scaleJacobian);
	ROS_INFO("Scaling the jacobian matrix is %s. (Parameter: scale_jacobian)",
			scaleJacobian ? "ENABLED" : "DISABLED");

    // TODO: Should this be an interface?
    /*
    if(online_mode)
    {
        ROS_INFO("Using online mode");
        CalibrationContext* context = new RosCalibContext(nhPrivate);
        m_dataCapture.reset(new DataCapture(calibContext));
    }
    */
}

PoseSelectionNode::~PoseSelectionNode() {

}

void PoseSelectionNode::initialize() {
	this->initializeCamera();
	this->initializeKinematicChain();
	this->initializeState();
}

void PoseSelectionNode::initializeKinematicChain() {
	// instantiate the model loader
	KDL::Tree kdlTree;
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);

	// instantiate the kinematic chain
	string chainName, chainRoot, chainTip;
	vector<string> chainNames;
	ROS_INFO("Looking in namespace %s",nhPrivate.getNamespace().c_str());
	nhPrivate.param("chain_names", chainNames, chainNames);
	cout << "Got chain names: " << boost::algorithm::join(chainNames, ", ") << endl;
	for(int i =0; i < chainNames.size(); ++i)
	{
	  string chainName = chainNames[i];
	  cout << chainName<<", ";
      ROS_INFO("Looking in namespace %s",nh.getNamespace().c_str());
      nh.getParam(chainName+"/chain_root", chainRoot);
	  nh.getParam(chainName+"/chain_tip", chainTip);
	  this->kinematicChainsMapPtr[chainName] = boost::make_shared<KinematicChain>(kdlTree,
	                          chainRoot, chainTip, chainName);
	}
	if (chainNames.empty())
	{
	  ROS_ERROR("Received emtpy chain names. Quitting!");
	      throw std::runtime_error("Empty chains");
	}


}

void PoseSelectionNode::initializeState() {
	// TODO: (optionally) do not use "zero initialization" for the joint offsets and marker
	this->initialState = boost::make_shared<KinematicCalibrationState>();

	vector<string> jointNames;
	vector<string> ignoreJoints;
	nh.getParam("ignore_joints", ignoreJoints);

    // set all joint offsets to zero
	for ( map<string, KinematicChainType>::iterator kit = kinematicChainsMapPtr.begin(); kit != kinematicChainsMapPtr.end(); ++kit){
	  kit->second->getJointNames(jointNames);
	  for (vector<string>::const_iterator it = jointNames.begin();
	      it != jointNames.end(); it++) {
	    // add jointOffset Parameter for all joints in all kinematicChains.
	    // map makes this a unique set
	    if (std::find(ignoreJoints.begin(), ignoreJoints.end(), *it) != ignoreJoints.end())
	    {
	      ROS_INFO("Ignoring jointOffset %s", it->c_str());
	    }
	    else
	    {
	      this->initialState->jointOffsets[*it] = 0.0;
	      ROS_INFO("Initialized %s with %f", it->c_str(), 0.0);
	    }

	  }
	  // initialize marker transform
	  string chainTip;
	  string markerFrame;
	  string chainName = kit->first;
	  nh.getParam(chainName + "/marker_frame", markerFrame);
	  nh.getParam(chainName + "/chain_tip", chainTip);
	  this->initialState->addMarker(chainName, chainTip, markerFrame);
	}




	// initialize transform from camera to head
	string cameraJointName = "CameraBottom"; // todo: parameterize!
	urdf::Model model;
	this->modelLoader.getUrdfModel(model);
	urdf::Joint cameraJoint = *model.getJoint(cameraJointName);
	urdf::Pose headPitchToCameraPose =
			cameraJoint.parent_to_joint_origin_transform;
	tf::Transform headToCamera = tf::Transform(
			tf::Quaternion(headPitchToCameraPose.rotation.x,
					headPitchToCameraPose.rotation.y,
					headPitchToCameraPose.rotation.z,
					headPitchToCameraPose.rotation.w),
			tf::Vector3(headPitchToCameraPose.position.x,
					headPitchToCameraPose.position.y,
					headPitchToCameraPose.position.z));
	initialState->cameraToHeadTransformation = headToCamera;

	// initialize the camera intrinsics
	initialState->cameraInfo = cameraModel.cameraInfo();
}

void PoseSelectionNode::initializeCameraFromFile() {
  std::string filename;
  nhPrivate.param("camera_file_location", filename, filename);
  string camera_name("nao_camera");
  ROS_INFO("Setting camera from file %s", filename.c_str());

  boost::shared_ptr<sensor_msgs::CameraInfo> cam_info_ptr(new sensor_msgs::CameraInfo);
  camera_calibration_parsers::readCalibration(filename, camera_name, *cam_info_ptr);
  camerainfoCallback(cam_info_ptr);
}

void PoseSelectionNode::initializeCamera() {
  bool cam_from_file = false;
  nhPrivate.param("camera_from_file",cam_from_file, cam_from_file);
  if (cam_from_file)
    initializeCameraFromFile();
  else
  {
	// TODO: parameterize!
	string topic = "/nao_camera/camera_info";
	cameraInfoSubscriber = nh.subscribe(topic, 1,
			&PoseSelectionNode::camerainfoCallback, this);
	ROS_INFO("Waiting for camera info message...");
	while (ros::getGlobalCallbackQueue()->isEmpty()) {
		//ros::Duration(0.5).sleep();
	}
	ros::getGlobalCallbackQueue()->callAvailable();
	cameraInfoSubscriber.shutdown();
  }
}




void PoseSelectionNode::callOptimization(ValidationNode & node, boost::shared_ptr<MeasurementPoseSet> resultSet, ValidationPoseSource & poseSourceVal, int splitID, const KinematicCalibrationState & state ){
    if (!ros::ok())
        throw std::runtime_error("ros is not ok");
    node.getOptimizationData().clear();
    vector < string > optimizationIDs = poseSourceVal.getPoseIds(resultSet->getPoses());
    cout << "Setting optimization id " << endl;
    for(vector<measurementData>::iterator it = node.getMeasurementPool().begin(); it != node.getMeasurementPool().end(); ++it)
    {
      if (std::find(optimizationIDs.begin(), optimizationIDs.end(),
                    it->id) != optimizationIDs.end()) {
        node.getOptimizationData().push_back(measurementData(*it));
       cout << it->id << ",";
      }
    }
    int numOptIds = static_cast<int>(optimizationIDs.size());
    if (numOptIds != resultSet->getNumberOfPoses())
    {
        string errorstr("Optimization IDs size " + boost::lexical_cast<string>(numOptIds) + " does not equal number of poses" + boost::lexical_cast<string>(resultSet->getNumberOfPoses()));
        throw std::runtime_error(errorstr);
    }
    string file_prefix="DUMMY";
    nhPrivate.param("file_prefix", file_prefix, file_prefix);
    string splitstr = boost::lexical_cast<string>(splitID);
    node.folderName = "/home/maierd/kin_calib_experiments/results/" + file_prefix + "/all_except_" + splitstr + "/" + file_prefix + "_" + boost::lexical_cast<string>(node.getOptimizationData().size());
    cout << "\nWriting to " << node.folderName << endl;
    boost::filesystem::path dirpath(node.folderName);
    try
    {
        boost::filesystem::create_directories( dirpath );
    }
    catch(const boost::filesystem::filesystem_error& e)
    {
        cerr << e.what() << endl;
        cerr << e.code() << endl;
        throw std::runtime_error("Unable to create destination directory " + node.folderName );
    }
    // add noise to optimization data
    node.modifyData();
    ROS_INFO("Starting optimize...");
    // set OptimizationDataIds
    node.repeatCounter = 0;
    node.optimize(state);
    ROS_INFO("Not Printing results");
    //node.printResult();
    ROS_INFO("Starting validation...");
    double errorsum = node.validate();
    ROS_INFO("Errorsum  is %f", errorsum);
    ROS_INFO("Writing calibration results");
    node.writeCalibrationResults();
}


boost::shared_ptr<MeasurementPoseSet> PoseSelectionNode::getOptimalPoseSetAndValidate(const vector<string> & validationIDs, int splitID) {
      // TODO: This is a very mean HACK!
        CalibrationContext* context = new RosCalibContext(nhPrivate);
        ValidationNode node(context);
        // 0. initialState is set on initialization
        // 1. estimate pose set
        // 2. call optimization, get new state
        // 3. replace initialState (including cameraModel, JointOffsets, and Markers)
        // 4. re-generate PoseSet (i.e. PoseSet+State+Derivatives)
        // 5. compute new pose set

        //node.initialize();
        // now get validationData and optimizationData or set the ros params and call
        // splitData();


        for(vector<measurementData>::iterator it = node.getMeasurementPool().begin(); it != node.getMeasurementPool().end(); ++it)
        {
          if (std::find(validationIDs.begin(), validationIDs.end(),
                               it->id) != validationIDs.end()) {
            node.getValidationData().push_back(measurementData(*it));
          }
        } // tmp

        ValidationPoseSource & poseSourceVal = static_cast<ValidationPoseSource&>(poseSource);


        int maxPoses;
        nhPrivate.param("num_of_poses", maxPoses, maxPoses);
        // Params for optimal Selection Algorithms
        //string internalSelectionStrategy("INC");
        int initialPoseSetSize = 1;
        int iterations = 10;
        //nhPrivate.param("internal_selection_strategy", internalSelectionStrategy, internalSelectionStrategy );
        nhPrivate.param("num_initial_random_poses", initialPoseSetSize, initialPoseSetSize);
        nhPrivate.param("detmax_iterations", iterations, iterations);
        ROS_INFO("Params are %d and %d", initialPoseSetSize, iterations);

        ROS_INFO("Using (maximum) num_of_poses: %d", maxPoses);

        // initialize the pool of all available poses
        vector<MeasurementPose> posePool;

        string chainNames;
        map<string, KinematicChainType>::iterator it;
        // TODO: Implement automatic extraction of DOF per chain
        //vector<pair<uint, uint> > dof_distrib;
        map<string, int> dofs;
        dofs["larm"] = 4;
        dofs["rarm"] = 4;
        dofs["lleg"] = 5;
        dofs["rleg"] = 4; // In case there is no lleg, this should be 5 and RHipYawPitch needs to be removed from ignore
        map<string, int> poolIndex;

        for(it = kinematicChainsMapPtr.begin(); it!= kinematicChainsMapPtr.end(); ++it)
        {
          chainNames = chainNames + it->first + " ";
          // add all poses for the kinematicChain it->second to posePool
          this->poseSource.getPoses(*(it->second), posePool);
          poolIndex[it->first]=int(posePool.size());
          //uint dof = 0;
          //dof_distrib.push_back(make_pair<uint, uint>(dof, posePool.size() ));
        }

        // initialize the initial pose set
        boost::shared_ptr<MeasurementPoseSet> poseSet = make_shared<MeasurementPoseSet>(
                        *this->initialState);
        // copies all poses from posePool to poseSet member variable poseSet. Defines all possible poses
        // active poses are set to 0
        poseSet->addMeasurementPoses(posePool);


        ROS_INFO("Calculating optimal pose set for chains %s",
                        chainNames.c_str());
        ROS_INFO("Having %zu poses in posePool and %d in initial poseSet", posePool.size(), poseSet->getNumberOfPoses());

        // reference to optimal poseSet according to the chosen selection strategy
        boost::shared_ptr<MeasurementPoseSet> resultSet = poseSet;
        // value of the observability index
        double index;

        // return all
        if (selectionStrategyName == "return_all") {
                ROS_INFO("Returning all poses without optimizing...");
                resultSet->initializePoseSet(posePool.size());
                return resultSet;
        }

        // select random poses
        else if (selectionStrategyName == "random") {
            ROS_INFO("Generating random pose set...");
            KinematicCalibrationState optimizedState = *initialState;
            // this will not do any exchange, only appropriate pick initial samples
            DetmaxPoseSelectionStrategy  intializingStrategy(initialPoseSetSize, 1, dofs, poolIndex);
            vector<vector<int> > initialIndices = intializingStrategy.getRandomIndices(1, resultSet);
            ROS_INFO("Setting initial indices. Got vector of size %zu", initialIndices.size());
            if (initialIndices.size() < 1)
            {
                ROS_ERROR("Size is too small");
                throw std::runtime_error("Size too small");
            }
            for(int i =0; i < initialIndices[0].size(); ++i)
                cout << initialIndices[0][i] << " ";
            cout << endl;
            resultSet->setActivePoses(initialIndices[0]);
            ///////////// call optimization
            callOptimization(node, resultSet, poseSourceVal, splitID, optimizedState);
            optimizedState = node.getResult();
            int nPoses = resultSet->getNumberOfPoses();
            this->optimalPoses[nPoses] = resultSet;
            this->intermediateIndices[nPoses] = index;
            // Added temporarily for Detmax
            // poseSet->getNumberOfPoses() returns the number of active configurations (poses), initially it is 0 !?
            // observabilityIndex represents the employed index (NAI/minSV/etc.)
            for (int i = initialPoseSetSize+1; i <= maxPoses; ++i) {
                // compute a random set for i poses, based on the previous set (of size i-1)
                RandomPoseSelectionStrategy randomStrategy(i);
                resultSet = randomStrategy.getOptimalPoseSet(resultSet,
                                                             observabilityIndex, index);
                ///////////// call optimization
                callOptimization(node, resultSet, poseSourceVal, splitID, optimizedState);
                optimizedState = node.getResult();
                this->optimalPoses[i] = resultSet;
                this->intermediateIndices[i] = index;
            }
            return resultSet;
        }

        else if (selectionStrategyName == "random_exchange") {
                ROS_INFO("Generating random pose set and improving it by exchanging...");
                for (int i = 5; i <= maxPoses; i++) {
                        RandomExchangePoseSelectionStrategy strategy(i);
                        boost::shared_ptr<MeasurementPoseSet> initialSet = resultSet;
                        boost::shared_ptr<MeasurementPoseSet> optimizedSet = strategy.getOptimalPoseSet(
                                        initialSet, observabilityIndex, index);
                        this->optimalPoses[i] = optimizedSet; // This was set to initialSet before..
                        this->intermediateIndices[i] = index;
                }
                return resultSet;
        }
        else if (selectionStrategyName == "improve_given") {
                ROS_INFO("Improve given pose set by exchanging...");
                ROS_ERROR("improve_given strategy is Temporarily disabled because Daniel removed kinematicChainPtr.");
                /*
                for (int i = 5; i <= maxPoses; i++) {
                  // TODO: Anpassen fuer alle Ketten


                        shared_ptr<MeasurementPoseSet> initialSet =
                                        this->poseSource.getInitialPoseSet(*this->kinematicChainPtr,
                                                        *this->initialState, i);
                        ExchangePoseSelectionStrategy strategy;
                        shared_ptr<PoseSet> optimizedSet = strategy.getOptimalPoseSet(
                                        initialSet, observabilityIndex, index);
                        this->optimalPoses[i] = optimizedSet;
                        this->intermediateIndices[i] = index;
                        resultSet = optimizedSet;
                }
                */
                return resultSet;
        }
        else if (selectionStrategyName == "DETMAX") // sample & exchange
        {
          ROS_INFO("Generating pose set using DETMAX...");
          for (int i = initialPoseSetSize; i <= maxPoses; ++i )
          {
            DetmaxPoseSelectionStrategy detmaxStrategy(i, iterations, dofs, poolIndex);
            resultSet = detmaxStrategy.getOptimalPoseSet(resultSet, observabilityIndex, index);
            this->optimalPoses[i] = resultSet;
            this->intermediateIndices[i] = index;
          }
        }
        else if (selectionStrategyName == "DETMAXDEC") // sample & exchange, followed by decrement
           {
            // TODO: Add new innitialzing Strategy that uses virtual measurement around expected value
             ROS_INFO("Generating pose set using DETMAXDEC...");
             DetmaxPoseSelectionStrategy  intializingStrategy(maxPoses, iterations, dofs, poolIndex);
             resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
                                                               observabilityIndex, index);
             // Added temporarily for Detmax
             DecrementalPoseSelectionStrategy decrementStratgy(1);
             ExchangePoseSelectionStrategy exStrategyDec;

             boost::shared_ptr<MeasurementPoseSet> fullSet(new MeasurementPoseSet(*resultSet));
             vector<int> ap = resultSet->getActivePoses();

             ROS_INFO("best active pose set");
             for(int i = 0; i < ap.size(); ++i)
               cout << ap[i] << " ";

             ROS_INFO("Starting decrement...");

             for (int i = resultSet->getNumberOfPoses(); i >= initialPoseSetSize ; i--) {
               //ROS_INFO("Improve by exchange...");
               //resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
               //     index);
               this->optimalPoses[i] = resultSet; // this is ok, as getOptimalPoseSet returns a shared_ptr to a new object
               this->intermediateIndices[i] = index;
               ROS_INFO("Pose Set size %d has index %f", resultSet->getNumberOfPoses(), index);
               //ROS_INFO("Decrement pose set size...");
               resultSet = decrementStratgy.getOptimalPoseSet(resultSet,
                                                              observabilityIndex, index);
               resultSet = exStrategyDec.getOptimalPoseSet(resultSet, observabilityIndex,
                                                           index);
             }
             ap = fullSet->getActivePoses();
             ROS_INFO("best active pose set (repeat)");
             for(int i = 0; i < ap.size(); ++i)
               cout << ap[i] << " ";
             resultSet = fullSet;
           }
           else if (selectionStrategyName == "DETMAXINC"  )
           {
             ROS_INFO("Generating pose set using DETMAXINC...");
             DetmaxPoseSelectionStrategy  intializingStrategy(initialPoseSetSize, iterations, dofs, poolIndex); // minimum number of (useful) poses
             resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
                                                               observabilityIndex, index);
             // increment by one strategy
             IncrementalPoseSelectionStrategy incrementStrategy(1);
             //ExchangePoseSelectionStrategy exStrategy;
             int numActivePoses = resultSet->getNumberOfPoses();

             this->optimalPoses[numActivePoses] = resultSet;
             this->intermediateIndices[numActivePoses] = index;
             ///////////// call optimization
             callOptimization(node, resultSet, poseSourceVal, splitID, *(this->initialState));
             KinematicCalibrationState optimizationResult = node.getResult();
             resultSet->setKinematicCalibrationState(optimizationResult);



             double curIndexValue;
             observabilityIndex->calculateIndex(*resultSet, curIndexValue);
             ROS_INFO("Computed index again %f (before) vs %f (after optimization) for poses:", index, curIndexValue);
             vector<int> ap = resultSet->getActivePoses();
             for(int i = 0; i < ap.size(); ++i)
                 cout << ap[i] << " ";

             //// loop to add remaining poses
             ///
             for (int i = resultSet->getNumberOfPoses(); i < maxPoses; i++) {
                 if (!ros::ok())
                     throw std::runtime_error("ros is not ok");
                 ROS_INFO("Adding one pose");
                 resultSet = incrementStrategy.getOptimalPoseSet(resultSet,
                                                                 observabilityIndex, index);
                 observabilityIndex->calculateIndex(*resultSet, curIndexValue);
                 vector<int> ap = resultSet->getActivePoses();
                 ROS_INFO("After adding pose, index is: %f for %zu poses:", curIndexValue, ap.size());
                 for(int j = 0; j < ap.size(); ++j)
                     cout << ap[j] << " ";
                 // TODO: Is this allowed? Pose set is not incremental anymore..
                 //ROS_INFO("Improve by exchange...");
                 //resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
                 //                                         index);
                 ROS_INFO("Pose Set size %d has index %f", resultSet->getNumberOfPoses(), index);
                 ///////////// call optimization
                 callOptimization(node, resultSet, poseSourceVal, splitID, optimizationResult);
                 // TODO: Should the optimization use the last estimate as initial value?
                 optimizationResult = node.getResult();
                 resultSet->setKinematicCalibrationState(optimizationResult);
                 ////
                 double curIndexValue;
                 observabilityIndex->calculateIndex(*resultSet, curIndexValue);
                 ap = resultSet->getActivePoses();
                 ROS_INFO("Computed index again %f (before) vs %f (after optimization) for %zu poses:", index, curIndexValue, ap.size());
                 for(int j = 0; j < ap.size(); ++j)
                     cout << ap[j] << " ";
                 if( std::isnan(curIndexValue)){
                     ROS_WARN("Index is NaN!");
                     ROS_WARN_STREAM("CalibrationState is \n" << optimizationResult);
                     boost::shared_ptr<map<int, MeasurementPose> > pP = resultSet->getPosePool();
                     Eigen::MatrixXd jac;
                     resultSet->getJacobian(jac);
                     ROS_INFO("Jacobian is");
                     cout << jac << endl;
                     for(int j =0; j < ap.size(); ++j)
                     {
                         ROS_WARN("============= Information for pose %d (index %d)", ap[j], j);
                         MeasurementPose mp =  (*pP)[ap[j]];
                         cout << mp.getJointState() << endl;
                         cout << mp.getChainName() << endl;
                         Eigen::MatrixXd deriv;
                         mp.getPartialDerivatives(optimizationResult, deriv);
                         cout << deriv << endl;
                     }
                 }

                 this->optimalPoses[i+1] = resultSet;
                 this->intermediateIndices[i+1] = index;
             }
        }
        else if (selectionStrategyName == "INC") // this is the old "optimal" strategy
        {
            ROS_INFO("Using standard INC strategy");
            IncrementalPoseSelectionStrategy intializingStrategy(1);
            resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
                                                              observabilityIndex, index);
            //
            // increment by one strategy
            IncrementalPoseSelectionStrategy incrementStrategy(1);
            ExchangePoseSelectionStrategy exStrategy;
            int numActivePoses = resultSet->getNumberOfPoses();
            this->optimalPoses[numActivePoses] = resultSet;
            this->intermediateIndices[numActivePoses] = index;

            double curIndexValue;
            observabilityIndex->calculateIndex(*resultSet, curIndexValue);
            ROS_INFO("Computed index again: %f for poses:", curIndexValue);
            vector<int> ap = resultSet->getActivePoses();
            for(int i = 0; i < ap.size(); ++i)
                cout << ap[i] << " ";

            for (int i = resultSet->getNumberOfPoses(); i < maxPoses; i++) {
                ROS_INFO("Adding one pose");
                resultSet = incrementStrategy.getOptimalPoseSet(resultSet,
                                                                observabilityIndex, index);
                observabilityIndex->calculateIndex(*resultSet, curIndexValue);
                ROS_INFO("After adding pose, index is: %f for poses:", curIndexValue);
                vector<int> ap = resultSet->getActivePoses();
                for(int j = 0; j < ap.size(); ++j)
                    cout << ap[j] << " ";
                ROS_INFO("Improve by exchange...");
                resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
                                                         index);
                ROS_INFO("Pose Set size %d has index %f", resultSet->getNumberOfPoses(), index);
                this->optimalPoses[i+1] = resultSet;
                this->intermediateIndices[i+1] = index;
            }
           }
           else
           {
             ROS_ERROR("Unknown selection_strategy %s", selectionStrategyName.c_str());
             throw std::runtime_error("Unknown selection strategy");
           }
        // finally
        return resultSet;
}


boost::shared_ptr<MeasurementPoseSet> PoseSelectionNode::getOptimalPoseSet() {

	int maxPoses;
	nhPrivate.param("num_of_poses", maxPoses, maxPoses);
	// Params for optimal Selection Algorithms
	string internalSelectionStrategy("INC");
	int initialPoseSetSize = 1;
	int iterations = 10;
	//nhPrivate.param("internal_selection_strategy", internalSelectionStrategy, internalSelectionStrategy );

	nhPrivate.param("num_initial_random_poses", initialPoseSetSize, initialPoseSetSize);
	nhPrivate.param("detmax_iterations", iterations, iterations);
	ROS_INFO("Params are %d and %d", initialPoseSetSize, iterations);

	ROS_INFO("Using (maximum) num_of_poses: %d", maxPoses);

	// initialize the pool of all available poses
	vector<MeasurementPose> posePool;

    // initialize the initial pose set
    shared_ptr<MeasurementPoseSet> poseSet = make_shared<MeasurementPoseSet>(
            *this->initialState);




	string chainNames;
	map<string, KinematicChainType>::iterator it;
	// TODO: Implement automatic extraction of DOF per chain
	//vector<pair<uint, uint> > dof_distrib;
	map<string, int> dofs;
	dofs["larm"] = 4;
	dofs["rarm"] = 4;
	dofs["lleg"] = 5;
	dofs["rleg"] = 4; // In case there is no lleg, this should be 5 and RHipYawPitch needs to be removed from ignore
	map<string, int> poolIndex;

	for(it = kinematicChainsMapPtr.begin(); it!= kinematicChainsMapPtr.end(); ++it)
	{
	  chainNames = chainNames + it->first + " ";
	  // add all poses for the kinematicChain it->second to posePool
	  this->poseSource.getPoses(*(it->second), posePool);
	  poolIndex[it->first]=int(posePool.size());
	  //uint dof = 0;
	  //dof_distrib.push_back(make_pair<uint, uint>(dof, posePool.size() ));
	}


	// copies all poses from posePool to poseSet member variable poseSet. Defines all possible poses
	// active poses are set to 0
	poseSet->addMeasurementPoses(posePool);


	ROS_INFO("Calculating optimal pose set for chains %s",
			chainNames.c_str());
	ROS_INFO("Having %zu poses in posePool and %d in initial poseSet", posePool.size(), poseSet->getNumberOfPoses());

	// reference to optimal poseSet according to the chosen selection strategy
	shared_ptr<MeasurementPoseSet> resultSet = poseSet;
	// value of the observability index
	double index;

	// return all
	if (selectionStrategyName == "return_all") {
		ROS_INFO("Returning all poses without optimizing...");
		resultSet->initializePoseSet(posePool.size());
		return resultSet;
	}

	// select random poses
	else if (selectionStrategyName == "random") {
		ROS_INFO("Generating random pose set...");
		// poseSet->getNumberOfPoses() returns the number of active configurations (poses), initially it is 0 !?
		// observabilityIndex represents the employed index (NAI/minSV/etc.)
		for (int i = initialPoseSetSize; i <= maxPoses; ++i) {
		        // compute a random set for i poses, based on the previous set (of size i-1)
			RandomPoseSelectionStrategy randomStrategy(i);
			resultSet = randomStrategy.getOptimalPoseSet(resultSet,
                    observabilityIndex, index);
			this->optimalPoses[i] = resultSet;
			this->intermediateIndices[i] = index;
		}
		return resultSet;
	}

	else if (selectionStrategyName == "random_exchange") {
		ROS_INFO("Generating random pose set and improving it by exchanging...");
		for (int i = 5; i <= maxPoses; i++) {
			RandomExchangePoseSelectionStrategy strategy(i);
			shared_ptr<MeasurementPoseSet> initialSet = resultSet;
			shared_ptr<MeasurementPoseSet> optimizedSet = strategy.getOptimalPoseSet(
                    initialSet, observabilityIndex, index);
			this->optimalPoses[i] = optimizedSet; // This was set to initialSet before..
			this->intermediateIndices[i] = index;
		}
		return resultSet;
	}
	else if (selectionStrategyName == "improve_given") {
		ROS_INFO("Improve given pose set by exchanging...");
		ROS_ERROR("improve_given strategy is Temporarily disabled because Daniel removed kinematicChainPtr.");
		/*
		for (int i = 5; i <= maxPoses; i++) {
		  // TODO: Anpassen fuer alle Ketten


			shared_ptr<MeasurementPoseSet> initialSet =
					this->poseSource.getInitialPoseSet(*this->kinematicChainPtr,
							*this->initialState, i);
			ExchangePoseSelectionStrategy strategy;
			shared_ptr<PoseSet> optimizedSet = strategy.getOptimalPoseSet(
					initialSet, observabilityIndex, index);
			this->optimalPoses[i] = optimizedSet;
			this->intermediateIndices[i] = index;
			resultSet = optimizedSet;
		}
		*/
		return resultSet;
	}
	else if (selectionStrategyName == "DETMAX") // sample & exchange
	{
	  ROS_INFO("Generating pose set using DETMAX...");
	  for (int i = initialPoseSetSize; i <= maxPoses; ++i )
	  {
	    DetmaxPoseSelectionStrategy detmaxStrategy(i, iterations, dofs, poolIndex);
        resultSet = detmaxStrategy.getOptimalPoseSet(resultSet, observabilityIndex, index);
	    this->optimalPoses[i] = resultSet;
	    this->intermediateIndices[i] = index;
	  }
	}
	else if (selectionStrategyName == "DETMAXDEC") // sample & exchange, followed by decrement
           {
             ROS_INFO("Generating pose set using DETMAXDEC...");
             DetmaxPoseSelectionStrategy  intializingStrategy(maxPoses, iterations, dofs, poolIndex);
             resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
                                                               observabilityIndex, index);
             // Added temporarily for Detmax
             DecrementalPoseSelectionStrategy decrementStratgy(1);
             ExchangePoseSelectionStrategy exStrategyDec;

             shared_ptr<MeasurementPoseSet> fullSet(new MeasurementPoseSet(*resultSet));
             vector<int> ap = resultSet->getActivePoses();

             ROS_INFO("best active pose set");
             for(int i = 0; i < ap.size(); ++i)
               cout << ap[i] << " ";

             ROS_INFO("Starting decrement...");

             for (int i = resultSet->getNumberOfPoses(); i >= initialPoseSetSize ; i--) {
               //ROS_INFO("Improve by exchange...");
               //resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
               //     index);
               this->optimalPoses[i] = resultSet; // this is ok, as getOptimalPoseSet returns a shared_ptr to a new object
               this->intermediateIndices[i] = index;
               ROS_INFO("Pose Set size %d has index %f", resultSet->getNumberOfPoses(), index);
               //ROS_INFO("Decrement pose set size...");
               resultSet = decrementStratgy.getOptimalPoseSet(resultSet,
                                                              observabilityIndex, index);
               resultSet = exStrategyDec.getOptimalPoseSet(resultSet, observabilityIndex,
                                                           index);
             }
             ap = fullSet->getActivePoses();
             ROS_INFO("best active pose set (repeat)");
             for(int i = 0; i < ap.size(); ++i)
               cout << ap[i] << " ";
             resultSet = fullSet;
           }
           else if (selectionStrategyName == "DETMAXINC"  )
           {
             ROS_INFO("Generating pose set using DETMAXINC...");
             DetmaxPoseSelectionStrategy  intializingStrategy(initialPoseSetSize, iterations, dofs, poolIndex); // minimum number of (useful) poses
             resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
                                                               observabilityIndex, index);
             // increment by one strategy
             IncrementalPoseSelectionStrategy incrementStrategy(1);
             ExchangePoseSelectionStrategy exStrategy;
             int numActivePoses = resultSet->getNumberOfPoses();
             this->optimalPoses[numActivePoses] = resultSet;
             this->intermediateIndices[numActivePoses] = index;

             double curIndexValue;
             observabilityIndex->calculateIndex(*resultSet, curIndexValue);
             ROS_INFO("Computed index again: %f for poses:", curIndexValue);
             vector<int> ap = resultSet->getActivePoses();
             for(int i = 0; i < ap.size(); ++i)
               cout << ap[i] << " ";

             for (int i = resultSet->getNumberOfPoses(); i < maxPoses; i++) {
               ROS_INFO("Adding one pose");
               resultSet = incrementStrategy.getOptimalPoseSet(resultSet,
                                                               observabilityIndex, index);
               observabilityIndex->calculateIndex(*resultSet, curIndexValue);
               ROS_INFO("After adding pose, index is: %f for poses:", curIndexValue);
               vector<int> ap = resultSet->getActivePoses();
               for(int j = 0; j < ap.size(); ++j)
                 cout << ap[j] << " ";
               ROS_INFO("Improve by exchange...");
               resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
                                                        index);
               ROS_INFO("Pose Set size %d has index %f", resultSet->getNumberOfPoses(), index);
               this->optimalPoses[i+1] = resultSet;
               this->intermediateIndices[i+1] = index;
             }
           }
           else if (selectionStrategyName == "INC") // this is the old "optimal" strategy
           {
             ROS_INFO("Using standard INC strategy");
             IncrementalPoseSelectionStrategy intializingStrategy(1);
             resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
                                                               observabilityIndex, index);
             //
             // increment by one strategy
             IncrementalPoseSelectionStrategy incrementStrategy(1);
             ExchangePoseSelectionStrategy exStrategy;
             int numActivePoses = resultSet->getNumberOfPoses();
             this->optimalPoses[numActivePoses] = resultSet;
             this->intermediateIndices[numActivePoses] = index;

             double curIndexValue;
             observabilityIndex->calculateIndex(*resultSet, curIndexValue);
             ROS_INFO("Computed index again: %f for poses:", curIndexValue);
             vector<int> ap = resultSet->getActivePoses();
             for(int i = 0; i < ap.size(); ++i)
               cout << ap[i] << " ";

             for (int i = resultSet->getNumberOfPoses(); i < maxPoses; i++) {
               ROS_INFO("Adding one pose");
               resultSet = incrementStrategy.getOptimalPoseSet(resultSet,
                                   observabilityIndex, index);
               observabilityIndex->calculateIndex(*resultSet, curIndexValue);
               ROS_INFO("After adding pose, index is: %f for poses:", curIndexValue);
               vector<int> ap = resultSet->getActivePoses();
               for(int j = 0; j < ap.size(); ++j)
                 cout << ap[j] << " ";
                ROS_INFO("Improve by exchange...");
                resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
                      index);
                ROS_INFO("Pose Set size %d has index %f", resultSet->getNumberOfPoses(), index);
                this->optimalPoses[i+1] = resultSet;
                this->intermediateIndices[i+1] = index;
             }
           }
           else
           {
             ROS_ERROR("Unknown selection_strategy %s", selectionStrategyName.c_str());
             throw std::runtime_error("Unknown selection strategy");
           }
	// finally
	return resultSet;
}

map<int, boost::shared_ptr<MeasurementPoseSet> > PoseSelectionNode::getIntermediatePoseSets() {
	return this->optimalPoses;
}

map<int, double> PoseSelectionNode::getIntermediateIndices() {
	return this->intermediateIndices;
}

void PoseSelectionNode::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	}

	else
		ROS_FATAL("Camera model could not be set!");
}

} /* namespace kinematic_calibration */

