/*
 *  Kinematic Calibration: A calibration framework for humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */

#include <calibration/OnlineCalibration.h>
#include <kinematic_calibration/Capture.h> // auto generated msg
#include <pose_generation/FilePoseSource.h>

namespace kinematic_calibration {

using namespace boost;
using namespace ros;
using namespace std;


OnlineCalibration::OnlineCalibration() {
    // call initialization methods
    this->initialize();

    // create DataCapture service client and wait for its existance
    m_dataCapture = m_nhPrivate.serviceClient<Capture>("observe");
    ROS_INFO("Waiting 10 s for dataCapture service");
    if (!m_dataCapture.waitForExistence(ros::Duration(10.0)))
        throw std::runtime_error("Fatal: dataCapture service is not available.");
    ROS_INFO("OnlineCalibration ready");
}



void OnlineCalibration::initPoseSource()
{
    std::vector<string> filepaths;
    m_nhPrivate.getParam("pose_files", filepaths);
    if (filepaths.empty())
        throw std::runtime_error("You need to specify at least one file containing robot poses [param pose_files]");
    // FilePoseSource is derived from MeasurementMsgPoseSource and reads the MeasurementMsgs from file(s) upon creation
    m_poseSource.reset(new FilePoseSource(filepaths));

}



void OnlineCalibration::initOptimization()
{
    // RosCalibContext gets the calibration options via ROS params
    CalibrationContext* context = new RosCalibContext(m_nhPrivate);
    // G2oOptimizationInterface wraps the g2o-optimizer (Kuemmerle et al., 2011)
    m_optimizationInterface.reset(new G2oOptimizationInterface(context));
}

OnlineCalibration::~OnlineCalibration() {
}

int OnlineCalibration::observe()
{
    int observedPoses = 0; // the number of succesfully observed new poses (ignores observed ones and failures)

    ROS_DEBUG("Waiting for dataCapture service");
    if (!m_dataCapture.waitForExistence(ros::Duration(10.0)))
    {
        ROS_ERROR("dataCapture service does not exist!");
        return 0;
    }

    ROS_INFO("Observing %d poses ", m_resultSet->getNumberOfPoses());
    vector<string> activePoses = m_resultSet->getActivePoses();
    // iterate over selected poses
    for (vector<string>::iterator it = activePoses.begin(); it != activePoses.end(); ++it)
    {
        string ID = *it;
        MeasurementPose pose; // get a copy of the selected MeasurementPose
        if(!m_resultSet->getPoseByID(ID, pose))// This would be a bug in the implementation of MeasurmentPoseSet
            throw std::runtime_error("Fix me: ActivePose not found in the result set pool in OnlineCalibration::observe()");

        if (m_poseIDToObservation.count(ID) > 0)
        {
            ROS_DEBUG("Already observed %s. Not observing again..", ID.c_str());
            continue;
        }
        ROS_INFO("Observing %s", ID.c_str());

        // get the relevant joint names from the kinematic chain
        string chainName = pose.getKinematicChainName();
        vector<string> jointNames;
        if(m_kinematicChains.count(chainName)<0)
        {
            ROS_ERROR("Could not find matching kinematic chain %s", chainName.c_str());
            continue;
        }
        m_kinematicChains[chainName]->getJointNames(jointNames);
        if (jointNames.empty())
        {
            ROS_ERROR("Empty joint names for kinematic chain %s in OnlineCalibration::observe. Failed to observe pose %s", chainName.c_str(), ID.c_str());
            continue;
        }
        // create a Capture request/response object
        Capture capture;
        capture.request.chainName = chainName;
        capture.request.poseName = ID;
        capture.request.jointNames = jointNames;


        // call the service
        if(!m_dataCapture.call(capture))
        {
            ROS_ERROR("DataCapture service call failed when observing pose %s.", ID.c_str());
            continue;
        }

        // update the pool with new joint readings for the pose and add the corresponding observation
        measurementData meas;
        meas = capture.response.measurement;

        // create a new pose with updated joint states
        pose = MeasurementPose(pose.getKinematicChain(), meas.jointState, pose.getID());

        if(!m_resultSet->replacePose(ID, pose)) // this would be a bug
            throw std::runtime_error("Fix me: Pose with ID " + ID + " could not be replaced in pose set");

        // add the corresponding observation
        Observation obs;
        obs.marker_data = meas.marker_data;
        obs.marker_type = meas.marker_type;
        obs.marker_frame = meas.marker_frame;
        obs.camera_frame = meas.camera_frame;
        m_poseIDToObservation[ID] = obs;
        observedPoses++;
    }

    return observedPoses;

}

int OnlineCalibration::removeUnobservedPosesFromPoseSet()
{
    int removedPosesCounter = 0;
    // get only the active (i.e. selected) poses
    vector<string> activePoses = m_resultSet->getActivePoses();
    // iterate over them
    for (vector<string>::iterator activePoseKey = activePoses.begin(); activePoseKey != activePoses.end(); ++activePoseKey )
    {
        string ID = *activePoseKey;
        MeasurementPose pose; // get a copy of the selected MeasurementPose
        if(!m_resultSet->getPoseByID(ID, pose))// This would be a bug in the implementation of MeasurmentPoseSet
            throw std::runtime_error("Fix me: ActivePose not found in the result set pool");


        // pose is selected but has not been observed yet?
        if (m_poseIDToObservation.count(ID) < 1)
        {
            // so remove it
            ROS_WARN("Removing pose %s from pose set because there is no matching observation. This pose will not be observed again.", ID.c_str());
            m_resultSet->deleteMeasurementPose(ID);
            removedPosesCounter++;
        }
        else
        {
            ROS_DEBUG("Pose %s has been observed", ID.c_str());
        }
    }
    return removedPosesCounter;

}

void OnlineCalibration::selectAndOptimize(boost::shared_ptr<PoseSelectionStrategy> strategy)
{
    double indexValue;
    // get the optimal pose set (accoring strategy)
    ros::Time t0 = ros::Time::now();
    m_resultSet = strategy->getOptimalPoseSet(m_resultSet, m_observabilityIndex, indexValue);
    ROS_INFO("Pose Selection took %f s", (ros::Time::now() - t0).toSec());


    t0 = ros::Time::now();
    // observe new poses
    int newObservedPoses = observe();
    ROS_INFO("Observation of %d new poses took %f s", newObservedPoses, (ros::Time::now() - t0).toSec());

    // some poses could not be observed. Remove those from the pose set as they have no matching observation
    int removedPoses = removeUnobservedPosesFromPoseSet();
    if(removedPoses > 0)
        ROS_WARN("Removed %d poses from pose set ", removedPoses);

    t0 = ros::Time::now();
    // optimize from current pose set and set the state accordingly
    // (but only if new observations are available and a minimum number of poses have been observed)
    if (newObservedPoses > 0  && minimumNumberOfPosesReached() )
    {
        if(!optimize()) // optimize() also sets the new state
            ROS_ERROR("Could not optimize selected poses!"); // not fatal because we can continue trying with more poses
        ROS_INFO("Optimization took %f s for %d poses", (ros::Time::now() - t0).toSec(), m_resultSet->getNumberOfPoses());
    }
}



bool OnlineCalibration::minimumNumberOfPosesReached()
{
    vector<MeasurementPose> selectedPoses = m_resultSet->getActiveMeasurementPoses();

    unsigned int numObservedPoses = 0;
    for (vector<MeasurementPose>::const_iterator poseIt = selectedPoses.begin(); poseIt != selectedPoses.end(); ++poseIt)
    {
        std::string id = poseIt->getID();
        if (m_poseIDToObservation.count(id) > 0)
            numObservedPoses++;
    }
    if (numObservedPoses >= m_minimumNecessaryPoses)
    {
        return true;
    }
    return false;
}



void OnlineCalibration::calibrateAndPublish()
{
    // publish result of calibration immediately after completion
    publishState(calibrate());
}


} /* namespace kinematic_calibration */


int main(int argc, char** argv) {
	// initialize the node
    ros::init(argc, argv, "onlineCalibration");

    kinematic_calibration::OnlineCalibration onlineCalibration;
    onlineCalibration.calibrateAndPublish();
    ROS_INFO("Calibration finished!");

	return 0;
}
