/*
 *  Kinematic Calibration: A calibration framework for humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */


// this file is the implementation of CalibrationFramework<T> declared in
#include <calibration/CalibrationFramework.h>

#include <camera_calibration_parsers/parse.h>
#include <kinematic_calibration/calibrationResult.h>
#include <common/MeasurementPose.h>
#include <common/helpers.h>
#include <pose_generation/DetmaxPoseSelectionStrategy.h>

namespace kinematic_calibration {


template <typename PoseSourceType>
CalibrationFramework<PoseSourceType>::CalibrationFramework() : m_nhPrivate("~")
{
    // publishes a calibration state, latched = true
    calibrationPublisher = m_nhPrivate.advertise<kinematic_calibration::calibrationResult>("calibration_result", 1, true);
}

template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initialize()
{
    // order of member initialization is important
    initPoseSource();
    initializeCamera();
    initializeKinematicChain();
    initializeState();
    initPoseSet();
    initPoseSelectionStrategy();
    initOptimization();
}

template <typename PoseSourceType>
CalibrationFramework<PoseSourceType>::~CalibrationFramework()
{
}


template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initializeKinematicChain() 
{
    // each KinematicChain extracts information from a KDLTree
    KDL::Tree kdlTree;
    // instantiate the model loader
    m_modelLoader.initializeFromRos();
    // fill the KDLTree of the robot's kinematic structure
    m_modelLoader.getKdlTree(kdlTree);

    // retrieve the names of the kinematic chains to be considered in the calibration process
    string chainName, chainRoot, chainTip;
    vector<string> chainNames;
    m_nhPrivate.param("chain_names", chainNames, chainNames);
    if (chainNames.empty())
    {
        ROS_ERROR("Received emtpy chain names. Quitting!");
        throw std::runtime_error("Empty chains");
    }

    // create the kinematic chains from specified root and tip
    for(unsigned int i =0; i < chainNames.size(); ++i)
    {
        chainName = chainNames[i];
        ROS_DEBUG("Looking for chain_root/chain_tip in namespace %s/[chainName]/",m_nhPrivate.getNamespace().c_str());
        m_nhPrivate.getParam(chainName+"/chain_root", chainRoot);
        m_nhPrivate.getParam(chainName+"/chain_tip", chainTip);
        // create actual KinematicChain
        m_kinematicChains[chainName] = boost::make_shared<KinematicChain>(kdlTree, chainRoot, chainTip, chainName);
    }

}


template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initializeState() 
{

    // camera model needs to be initialized in order to set the state, so check it
    if(!m_cameraModel.initialized() || m_cameraModel.tfFrame() == "" )
        throw std::runtime_error("Camera model not initialized correctly");

    // create a new CalibrationState containing joint offsets, camera parameters, etc.
    m_initialState = boost::make_shared<CalibrationState>();

    // initialize the camera intrinsics
    m_initialState->cameraInfo = m_cameraModel.cameraInfo();


    // list of joints to be ignored in calibration (typically redundant joints)
    vector<string> ignoreJoints;
    m_nhPrivate.getParam("ignore_joints", ignoreJoints);

    // set all joint offsets to zero by iterating over the kinematic chains
    for ( map<string, KinematicChainType>::iterator kit = m_kinematicChains.begin(); kit != m_kinematicChains.end(); ++kit)
    {
        vector<string> jointNames;
        kit->second->getJointNames(jointNames);
        for (vector<string>::const_iterator it = jointNames.begin(); it != jointNames.end(); it++) {
            // add jointOffset Parameter for all joints in all kinematicChains to m_initialState
            if (std::find(ignoreJoints.begin(), ignoreJoints.end(), *it) == ignoreJoints.end())
            {
                m_initialState->jointOffsets[*it] = 0.0;
                ROS_INFO("+ Initialized %s with %f", it->c_str(), 0.0);
            }
            else // unless they are in the ignore list
                ROS_INFO("- Ignoring jointOffset %s", it->c_str());
        }
    }

    // add marker for each chain to  m_initialState
    for ( map<string, KinematicChainType>::iterator kit = m_kinematicChains.begin(); kit != m_kinematicChains.end(); ++kit)
    {
        string markerFrame;
        string chainName = kit->first;
        ROS_DEBUG("Looking for marker_frame in namespace %s/[chainName]/",m_nhPrivate.getNamespace().c_str());
        m_nhPrivate.getParam(chainName + "/marker_frame", markerFrame);
        // this call initializes the marker transformation
        m_initialState->addMarker(chainName, kit->second->getTip(), markerFrame);
    }


    // initialize transform from camera its parent frame (typically the neck)
    urdf::Model model;
    m_modelLoader.getUrdfModel(model);
    // retrieve the parent joint of the camera link from the urdf (remove leading slash if necessary)
    boost::shared_ptr<const urdf::Link> link = model.getLink(withoutLeadingSlash(m_cameraModel.tfFrame()));
    if (!link)
        throw std::runtime_error("Could not find parent link of camera in urdf");
    boost::shared_ptr<const urdf::Joint> cameraJoint = link->parent_joint;
    urdf::Pose neckToCameraPose = cameraJoint->parent_to_joint_origin_transform;
    // convert urdf::Pose to a tf::Transform
    tf::Quaternion quat (neckToCameraPose.rotation.x, neckToCameraPose.rotation.y, neckToCameraPose.rotation.z, neckToCameraPose.rotation.w);
    tf::Vector3 trans (neckToCameraPose.position.x, neckToCameraPose.position.y, neckToCameraPose.position.z);
    tf::Transform neckToCameraTransform = tf::Transform(quat, trans);
    // update the state
    m_initialState->cameraTransformation = neckToCameraTransform;

}


template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initializeCameraFromFile() 
{
    // initialize the camera model from a opencv calibration (yaml/ini) file
    std::string filename;
    m_nhPrivate.param("camera_file_location", filename, filename);
    // name (id) of the camera
    string cameraName("nao_camera");
    ROS_INFO("Setting camera from file %s", filename.c_str());

    // using a camera info object allows to use the callback API to initialize the camera model
    boost::shared_ptr<sensor_msgs::CameraInfo> camInfoPtr(new sensor_msgs::CameraInfo);
    // parse the file
    camera_calibration_parsers::readCalibration(filename, cameraName, *camInfoPtr);
    // calibration file does not contain TF frame, so it needs to be manually specified
    std::string tfFrame;
    if(!m_nhPrivate.getParam("camera_tf_frame", tfFrame))
        throw std::runtime_error("No camera_tf_frame specified (when initializing camera from a file)");
    camInfoPtr->header.frame_id = tfFrame;
    // initialize the camera model
    setCameraModel(camInfoPtr);

}


template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::setCameraModel(const sensor_msgs::CameraInfoConstPtr& msg) 
{
    if (m_cameraModel.fromCameraInfo(msg)) {
        ROS_INFO("Camera model set.");
    }
    else
        ROS_FATAL("Camera model could not be set!");
}


template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::publishState(const CalibrationState & state) const
{
    kinematic_calibration::calibrationResult msg;

    // joint offsets
    for (map<string, double>::const_iterator jointNameAndOffset = state.jointOffsets.begin();
         jointNameAndOffset != state.jointOffsets.end(); jointNameAndOffset++) 
    {
        msg.jointNames.push_back(jointNameAndOffset->first);
        msg.jointOffsets.push_back(jointNameAndOffset->second);
    }

    // chain names and marker transformations
    for (map<string, tf::Transform>::const_iterator markerTransform = state.markerTransformations.begin();
         markerTransform != state.markerTransformations.end(); markerTransform++) 
    {
        msg.chainNames.push_back(markerTransform->first);
        geometry_msgs::Transform transform;
        tf::transformTFToMsg(markerTransform->second, transform);
        msg.endeffectorToMarker.push_back(transform);
    }

    // camera intrinsics
    msg.cameraInfo = state.cameraInfo;

    // camera transform
    geometry_msgs::Transform cameraTransform;
    tf::transformTFToMsg(state.cameraTransformation.inverse(), cameraTransform);
    msg.cameraTransform = cameraTransform;

    // calling the publisher
    calibrationPublisher.publish(msg);
}


template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initializeCamera() 
{
    bool initCamFromFile = false;
    m_nhPrivate.param("init_camera_from_file",initCamFromFile, initCamFromFile);
    if (initCamFromFile)
        initializeCameraFromFile();
    else
    {
        m_cameraInfoSubscriber = m_nh.subscribe("/camera/camera_info", 1, &CalibrationFramework::setCameraModel, this);
        ROS_INFO("Waiting for camera info message...");
        while (!m_cameraModel.initialized()) {
            ros::Rate(10.0).sleep();
            ros::spinOnce();
        }
        m_cameraInfoSubscriber.shutdown();
    }
}




template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initPoseSet()
{
    // temporary pool of poses
    std::vector<MeasurementPose> posePool;

    vector<string> chainNames; // just needed for the ROS_INFO
    // get all poses for all kinematic chains and store in m_posePool
    for(map<string, KinematicChainType>::iterator nameAndChainIt = m_kinematicChains.begin(); nameAndChainIt!= m_kinematicChains.end(); ++nameAndChainIt)
    {
        chainNames.push_back(nameAndChainIt->first);
        // add all poses for the kinematicChain it->second to m_posePool
        m_poseSource->getPoses(*(nameAndChainIt->second), posePool);
    }
    ROS_INFO("Calculating optimal pose set for chains %s", boost::algorithm::join(chainNames, ", ").c_str());

    // initialize the initial pose set, active poses are set to the empty set
    m_resultSet = make_shared<MeasurementPoseSet>(*m_initialState);

    // copies all poses from posePool to poseSet member variable poseSet. Defines all possible poses
    m_resultSet->addMeasurementPoses(posePool);

    ROS_DEBUG("There are %zu poses in the posePool and %d in the initial poseSet", posePool.size(), m_resultSet->getNumberOfPoses());

}

template <typename PoseSourceType>
void CalibrationFramework<PoseSourceType>::initPoseSelectionStrategy()
{
    // maximum and minimum number of poses to select (NodeHandle does not supports size_t in getParam)
    int maxNumPoses, minNumPoses;
    m_nhPrivate.getParam("maximum_number_of_poses", maxNumPoses);
    m_nhPrivate.getParam("minimum_number_of_poses", minNumPoses);
    if (maxNumPoses < 0 || minNumPoses < 0 )
        throw std::runtime_error("Invalid number of poses specified");
    m_maximumNumberOfPoses = maxNumPoses;
    m_minimumNecessaryPoses = minNumPoses;
    string selectionStrategyName;
    // selection is performed using a certain strategy
    m_nhPrivate.param("selection_strategy", selectionStrategyName, selectionStrategyName);
    if (selectionStrategyName == "DETMAXINC")
    {
        ROS_INFO("Using DETMAXINC pose selection strategy");
        // use multiple iterations of the DETMAX algorithm to initialize the set with the minimum number of poses
        m_initializationStrategy.reset(new DetmaxPoseSelectionStrategy(m_minimumNecessaryPoses));
        // after initialization incrementally add 1 pose (in a greedy fashion) to the set
        m_incrementalStrategy.reset(new IncrementalPoseSelectionStrategy(1)); // increment by one
    }
    else if (selectionStrategyName == "random")
    {
        ROS_INFO("Using random pose selection strategy");
        // randomly initialize the set of selected poses with the minimum number
        m_initializationStrategy.reset(new RandomPoseSelectionStrategy(m_minimumNecessaryPoses));
        // incrementally add 1 randomly sampled pose
        m_incrementalStrategy.reset(new RandomSuccessorPoseSelectionStrategy(1));
    }
    else
        ROS_ERROR("Unknown pose selection strategy: %s", selectionStrategyName.c_str());

    // little factory for the observability index used in the selection strategy
    string criterion("DET");
    m_nhPrivate.param("criterion", criterion, criterion);
    if (criterion=="NAI")
        m_observabilityIndex = boost::make_shared<NoiseAmplificationIndex>();
    else if (criterion=="DET") //  product of singular values (corresponds to the determinant of the covariance)
        m_observabilityIndex = boost::make_shared<ProductSingularValuesIndex>();
    else if (criterion=="SUM") // sum of singular values
        m_observabilityIndex = boost::make_shared<SumSingularValuesIndex>();
    else if (criterion=="MIN") // minimum singular value
        m_observabilityIndex = boost::make_shared<MinimumSingularValueIndex>();
    else
    {
        throw std::runtime_error("Unsupported criterion " + criterion);
    }
    ROS_INFO("Using criterion : %s", criterion.c_str());

    bool scaleJacobian = true;
    m_nhPrivate.param("scale_jacobian", scaleJacobian, scaleJacobian);
    m_observabilityIndex->setScaleJacobian(scaleJacobian);
    ROS_DEBUG("Scaling the jacobian matrix is %s. (Parameter: scale_jacobian)",
            scaleJacobian ? "ENABLED" : "DISABLED");

}


template <typename PoseSourceType>
CalibrationState CalibrationFramework<PoseSourceType>::calibrate()
{
    // initialize the set of selected poses using the m_initializationStrategy, then optimize to update the state
    selectAndOptimize(m_initializationStrategy);

    // using m_poseSelectionStrategy to select new poses and optimize until a target number of poses is reached
    while(ros::ok() && m_resultSet->getNumberOfPoses() < m_maximumNumberOfPoses)
    {
        selectAndOptimize(m_incrementalStrategy);
    }

    // return the final calibrated state
    return m_resultSet->getCalibrationState();

}



template <typename PoseSourceType>
bool CalibrationFramework<PoseSourceType>::createMeasurementDataFromMeasurementPose(const MeasurementPose & pose, measurementData & measurement)
{
    // check for matching observation
    std::string id = pose.getID();
    if (m_poseIDToObservation.count(id) < 1)
    {
        ROS_WARN("Could not find an observation for id %s", id.c_str());
        return false;
    }
    // check if the correct kinematic chain is known
    string chain_name = pose.getChainName();
    if (m_kinematicChains.count(chain_name) < 1)
    {
        ROS_WARN("Kinematic chain %s for id %s not found", chain_name.c_str(), id.c_str());
        return false;
    }

    // fill the data structure from observation, kinematic chain and pose
    measurement.camera_frame = m_poseIDToObservation[id].camera_frame;
    measurement.chain_name = chain_name;
    measurement.chain_root = m_kinematicChains[chain_name]->getRoot();
    measurement.chain_tip  = m_kinematicChains[chain_name]->getTip();
    measurement.jointState = pose.getJointState();
    measurement.header.frame_id = measurement.jointState.header.frame_id;
    measurement.header.stamp = m_poseIDToObservation[id].stamp ;
    measurement.id = id;
    measurement.marker_data = m_poseIDToObservation[id].marker_data;
    measurement.marker_type = m_poseIDToObservation[id].marker_type;
    measurement.marker_frame = m_poseIDToObservation[id].marker_frame;

    return true;
}


template <typename PoseSourceType>
bool CalibrationFramework<PoseSourceType>::optimize()
{  
    // reset current optimization data and set according to current pose set
    m_optimizationInterface->clearOptimizationData();
    // get the currently selected poses
    vector<MeasurementPose> selectedPoses = m_resultSet->getActiveMeasurementPoses();

    // convert each selected pose into the optimizer's data structure by adding the corresponding observation
    for (vector<MeasurementPose>::const_iterator poseIt = selectedPoses.begin(); poseIt != selectedPoses.end(); ++poseIt)
    {
        measurementDataPtr m(new measurementData());
        if(!createMeasurementDataFromMeasurementPose(*poseIt, *m))
        {
            ROS_ERROR("Could not create a measurementData for id %s. ", poseIt->getID().c_str());
            continue; // just leave it out then..
        }
        m_optimizationInterface->addOptimizationData(m);
    }

    // check if enough poses (constraints) are provided to the optimizer
    if (m_optimizationInterface->getOptimizationData().size() < m_minimumNecessaryPoses)
        return false;

    // optimize (starting with state in current pose set)
    if(!m_optimizationInterface->optimizeRobust( m_resultSet->getCalibrationState() ))
        return false;

    // replace the calibration state if optimization succeeded
    m_resultSet->setCalibrationState( m_optimizationInterface->getResult() );

    return true;
}

}
