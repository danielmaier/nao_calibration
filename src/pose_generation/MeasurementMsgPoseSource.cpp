#include <pose_generation/MeasurementMsgPoseSource.h>


namespace kinematic_calibration{
MeasurementMsgPoseSource::MeasurementMsgPoseSource(bool auto_subscribe) :
        collectingData(false) {
    this->topic = "/kinematic_calibration/measurement_data";
    this->nh.setCallbackQueue(&callbackQueue);
    this->measurementSubscriber.shutdown();
    if (auto_subscribe)
      this->subscribe();

    this->msgService.shutdown();
    this->msgService = nh.advertiseService(
            "/kinematic_calibration/stop_collecting",
            &MeasurementMsgPoseSource::stopCollectingCallback, this);
    ROS_INFO("MeasurementMsgPoseSource initialized");
}


void MeasurementMsgPoseSource::subscribe()
{
    // TODO: Why is the queue set to 10000?
  this->measurementSubscriber = nh.subscribe(topic, 10000,
                                             &MeasurementMsgPoseSource::measurementCb, this);
}

MeasurementMsgPoseSource::~MeasurementMsgPoseSource() {
    // nothing to do
}

void MeasurementMsgPoseSource::getPoses(const KinematicChain& kinematicChain,
        vector<MeasurementPose>& poses) {
    // process all pending messages
  ROS_INFO("MeasurementMsgPoseSource::getPoses, poses size is %zu", poses.size());
    if (this->poses.size() == 0) {
        collectData();
    }
     // TODO: Shouldn't the MeasurementPose use the options according to the ros params?
    // add all available poses for the requested chain

    PoseSourcePool pool =
            this->poses[kinematicChain.getName()];
    for (PoseSourcePool::const_iterator it = pool.begin();
            it != pool.end(); it++) {
        poses.push_back(MeasurementPose(kinematicChain, it->second, it->first));
    }
}

bool MeasurementMsgPoseSource::getPoseById(const string &id, MeasurementPose &pose)
{
    ROS_ERROR("THIS FUNCTION IS NOT IMPLEMENTED");
    return false;
    return true;
}



void MeasurementMsgPoseSource::getPosesByIds(
        const KinematicChain& kinematicChain, const vector<string>& ids,
        vector<MeasurementPose>& poses) {
    // get all poses
    vector<MeasurementPose> allPoses;
    this->getPoses(kinematicChain, allPoses);

    // get the one with the requested ids
    for (vector<MeasurementPose>::iterator it = allPoses.begin();
            it != allPoses.end(); it++) {
        if (std::find(ids.begin(), ids.end(),
                this->ids[it->getJointState().header.stamp]) != ids.end()) {
            poses.push_back(*it);
        }
    }
}

void MeasurementMsgPoseSource::measurementCb(
        const measurementDataConstPtr& msg) {
    // add the joint states (i.e. the pose) for the respective chain
    this->poses[msg->chain_name].addPose(msg->id, msg->jointState);
    this->ids[msg->jointState.header.stamp] = msg->id;
    ROS_INFO("Measurement data received (#%u).",
            this->poses[msg->chain_name].size());
    //ROS_INFO("There are now %zu poses in set", poses.size());
}

void MeasurementMsgPoseSource::collectData() {
    this->collectingData = true;
    while (this->collectingData && ros::ok()) {
        this->callbackQueue.callAvailable();
    }
}

bool MeasurementMsgPoseSource::stopCollectingCallback(
        std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response) {
    this->collectingData = false;
    this->measurementSubscriber.shutdown();
    return true;
}

vector<string> MeasurementMsgPoseSource::getPoseIds(
        vector<sensor_msgs::JointState> jointStates) {
    vector<string> ids;
    for (vector<sensor_msgs::JointState>::iterator it = jointStates.begin();
            it != jointStates.end(); it++) {
        if (this->ids.count(it->header.stamp)) {
            ids.push_back(this->ids[it->header.stamp]);
        }
    }

    return ids;
}
}
