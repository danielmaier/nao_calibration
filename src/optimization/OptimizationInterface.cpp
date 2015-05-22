/*
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */

#include <optimization/OptimizationInterface.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <geometry_msgs/Transform.h>
#include <kinematic_calibration/calibrationResult.h>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <string>
#include <fstream>
#include <urdf_model/model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "camera_calibration_parsers/parse.h"


#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std;

namespace kinematic_calibration {

G2oOptimizationInterface::G2oOptimizationInterface(CalibrationContext * ctext) : context(ctext), m_nhPrivate("~")
{

    // instantiate the model loader
    modelLoader.initializeFromRos();
    modelLoader.getKdlTree(kdlTree);

    this->calibrationOptions = context->getCalibrationOptions();

}



bool G2oOptimizationInterface::addPosesFromBagFile(const std::vector<std::string> & filenames)
{
  string msg_topic = "/kinematic_calibration/measurement_data";
  //string cam_info_topic = "/nao_camera/camera_info";

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(msg_topic);
  //topics.push_back(cam_info_topic);

  for(unsigned int i =0; i < filenames.size(); ++i)
  {
    cout << "Reading from " << filenames[i] << endl;
    // Load bag
    rosbag::Bag bag;
    bag.open(filenames[i], rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == msg_topic || ("/" + m.getTopic() == msg_topic))
      {
        kinematic_calibration::measurementData::ConstPtr meas_msg = m.instantiate<kinematic_calibration::measurementData>();
        if (meas_msg != NULL)
          processMeasurementDataMsg(meas_msg);
      }
      /*
      if (m.getTopic() == cam_info_topic || ("/" + m.getTopic() == cam_info_topic))
      {
        sensor_msgs::CameraInfo::ConstPtr info_msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (info_msg != NULL)
          node.camerainfoCallback(info_msg);
      }
      */
    }
    bag.close();
  }
  //this->splitPoseSets(this->numOfPartitionsPerChain);
  return true;
}


bool G2oOptimizationInterface::initializeCameraFromFile() {
  std::string filename;
  m_nhPrivate.param("camera_file_location", filename, filename);
  string camera_name("nao_camera");
  ROS_INFO("OptimizationInterface sets camera from file %s", filename.c_str());

  boost::shared_ptr<sensor_msgs::CameraInfo> cam_info_ptr(new sensor_msgs::CameraInfo);
  if(!camera_calibration_parsers::readCalibration(filename, camera_name, *cam_info_ptr))
    return false;
  setCameraModelFromMsg(cam_info_ptr);
  return true;

}

void G2oOptimizationInterface::setCameraModelFromMsg(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (cameraModel.fromCameraInfo(msg)) {
        ROS_INFO("Camera model set.");
        cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
                << endl;
    }

    else
        ROS_FATAL("Camera model could not be set!");
    cameraInfoSubscriber.shutdown();
}

void G2oOptimizationInterface::addNewKinematicChain(const measurementDataConstPtr & msg)
{
    measurementData data = *msg;
    // TODO: Make kinematicChain a map --> Requires change in the way chains are passed to optimizer

   // Add a new kinematic chain if it is not cotained yet
   if (nameToChain.count(data.chain_name) < 1) {
    // TOOD: Check if receiving the right params works (i.e. using correct namespace)
    string chainName = data.chain_name;
    string chainRoot = data.chain_root;
    string chainTip = data.chain_tip;
    // instantiate the kinematic chain
    KinematicChain kinematicChain(kdlTree, chainRoot, chainTip, chainName);
    this->kinematicChains.push_back(kinematicChain);
    this->nameToChain[chainName] = &(this->kinematicChains.back());
    ROS_INFO("Receive data for chain %s.", chainName.c_str());
    // try to get the current transformation from chain tip to marker frame
    string markerFrame = "";
    if ("" != msg->marker_frame) {
        // message should contain the info about the marker frame
        markerFrame = msg->marker_frame;
    } else if (m_nhPrivate.hasParam(chainName + "/" + "marker_frame")) {
        // fallback mechanism
        m_nhPrivate.getParam(chainName + "/" + "marker_frame", markerFrame);
    }
    if (markerFrame.length() > 0) {
        // determine the source (default is from URDF)
        CalibrationState::TransformSource source = CalibrationState::ROSPARAM_URDF;
        string sourceString;
        m_nhPrivate.param(chainName + "/" + "marker_transform_source", sourceString, sourceString);
        if ("tf" == sourceString)
            source = CalibrationState::TF;
        // delegate the initialization
        this->initialState.addMarker(chainName, chainTip, markerFrame,
                source);
    }
   }
}

void G2oOptimizationInterface::processMeasurementDataMsg(const measurementDataConstPtr& msg) {
    measurementData data = *msg;
    if (!measurementOk(msg)) {
        ROS_WARN("Received an invalid measurementData message");
        return;
    }

    // add new kinematic chain if necessary
    addNewKinematicChain(msg);

    // save data
    data.image = sensor_msgs::Image();
    this->measurementsPool.push_back(data);

    ROS_INFO_ONCE("Measurement data received (%ld) [will only print once]", this->measurementsPool.size());
    ROS_DEBUG("Measurement data received (%ld).", this->measurementsPool.size());
}


void G2oOptimizationInterface::addValidationData(const measurementDataConstPtr & msg)
{
    if (!measurementOk(msg)) {
        ROS_WARN("Received an invalid measurementData message");
        return;
    }

    // add new kinematic chain if necessary
    addNewKinematicChain(msg);

    validataionData.push_back(*msg);

}


void G2oOptimizationInterface::addOptimizationData(const measurementDataConstPtr & msg)
{
    if (!measurementOk(msg)) {
        ROS_WARN("Received an invalid measurementData message");
        return;
    }

    // add new kinematic chain if necessary
    addNewKinematicChain(msg);

    optimizationData.push_back(*msg);

}



bool G2oOptimizationInterface::measurementOk(const measurementDataConstPtr& msg) {
    const measurementData data = *msg;
    if (data.jointState.name.empty()) {
        return false;
    }

    if (data.marker_data.empty()) {
        return false;
    }

    return true;
}



bool G2oOptimizationInterface::optimize(){

    if (! cameraModel.initialized())
    {
        ROS_ERROR("Camera Model is not initialized! Quitting.");
        throw std::runtime_error("Camera model not initialized in OptimizationNode::optimize()");
    }

    CalibrationState initialState = this->initialState;

    // initialize transform from camera to head
    string cameraJointName = "CameraBottom"; // TODO: Fix this
    urdf::Model model;
    ModelLoader modelLoader;
    modelLoader.initializeFromRos();
    modelLoader.getUrdfModel(model);
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
    initialState.cameraTransformation = headToCamera;

    // initialize the camera intrinsics
    initialState.cameraInfo = cameraModel.cameraInfo();

    return optimize(initialState);
}


bool G2oOptimizationInterface::optimizeRobust(const CalibrationState & state) {
    repeatCounter = 0;
    return optimize(state);
}

bool G2oOptimizationInterface::optimize(const CalibrationState & state) {
    if (! cameraModel.initialized())
    {
        ROS_ERROR("Camera Model is not initialized! Quitting.");
        throw std::runtime_error("Camera model not initialized in ValidatioNode::optimize()");
    }

    // instantiate the frame image converter
    FrameImageConverter frameImageConverter(cameraModel);

    // new
    frameImageConverter.getCameraModel().fromCameraInfo(state.cameraInfo);

    // initial state
    CalibrationState initialState = state;



    cout << "initial State is\n" << initialState << endl;

    ROS_INFO("Will optimize %zu poses", optimizationData.size());
    if (optimizationData.size() < 1)
    {
        ROS_ERROR("There is nothing to optimize! This will fail!");
        throw std::runtime_error("No optimizationData given!");
    }


    CalibrationOptions opt = context->getCalibrationOptions();
    OptimizationOptions optOptimization = context->getOptimizationOptions();

    cout << "optimization options are \n";
    cout << "maxIterations: " << optOptimization.maxIterations << endl;
    cout << opt.calibrateCameraIntrinsics << endl;
    cout << opt.markerOptimizationType << endl;
    cout << opt.calibrateJointOffsets << endl;


    // optimization instance
    m_optimization.reset( new G2oJointOffsetOptimization(*context, optimizationData, kinematicChains, frameImageConverter, initialState));
    m_optimization->setSaveIntermediateStates(true);
    ROS_INFO("Call g2o...");

    int cjIterations = m_optimization->optimize(result);
    cout << "calibrated State is\n" << result << endl;

    /*
    BatchStatisticsContainer statistics = m_optimization.statistics;

    cout << "Computed statistics. Size is " << statistics.size() << endl;
    for (int i =0; i < statistics.size(); ++i){
        cout << "--" << i << endl;
        G2OBatchStatistics bat = statistics[i];
        cout << bat.iteration << ", " << bat.levenbergIterations << "," << bat.chi2 << ", " << bat.iterationsLinearSolver << endl;
    }
    */
    if (cjIterations <= 8 && optimizationData.size() > 8 && optOptimization.maxIterations > 8 )
    {
        repeatCounter++;
        if (repeatCounter > 10)
        {
            ROS_WARN("Too many retries. Giving up on restarting optimization..");
            return false;
        }
        else
        {
            ROS_WARN("It seems as if the optimization failed %d times! Restarting...",repeatCounter);
            return optimize(initialState);
        }
    }
    // otherwise everything is fine
    return true;

    // get intermdiate states
    //ROS_INFO("Getting intermediate States");
    //optimization.getIntermediateStates(intermediateStates);

    // TODO: Compute covariance somehow and return result for covariance in sweet spot
    /*
    for (int s = 0; s < intermediateStates.size(); ++s)
    {
        KinematicCalibrationState kcs = intermediateStates[s];
        MeasurementPoseSet poseSet(kcs);
        vector<MeasurementPose> activePoses;
        for(int i =0; i < optimizationData.size(); ++i)
        {
            measurementData d = optimizationData[i];

            KinematicChain chain;
            for (int j = 0; j < this->chainNames.size(); ++j)
            {
                if (chainNames[j] == d.chain_name)
                    chain =this->kinematicChains[j];
            }
            sensor_msgs::JointState js = d.jointState;
            MeasurementPose m (chain, js, opt);
            activePoses.push_back(m);
        }
        poseSet.addActiveMeasurementPoses(activePoses);
        Eigen::MatrixXd jac;
        poseSet.getJacobian(jac);
        cout << "Covariance for intermediate state " << s << "is" << endl;
        //cout << jac << endl;
        Eigen::MatrixXd cov = (jac.transpose() * jac).inverse();
        //cout << "covariance is\n " << cov << endl;
        double prod = 1.0;
        double maxi = 0.0;
        for(int j = 0; j < cov.rows(); ++ j)
        {
            //cout << cov(j,j) << ",";
            prod *= cov(j,j);
            maxi = max(maxi, cov(j,j));
        }
        cout << " prod: " << prod << " maxi " << maxi ;
        cout << endl;
    }
    */
}


} /* kinematic_calibration */
