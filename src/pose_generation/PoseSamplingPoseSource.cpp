#include <pose_generation/PoseSamplingPoseSource.h>

namespace kinematic_calibration{

PoseSamplingPoseSource::PoseSamplingPoseSource() :
        nhPrivate("~") {
    // nothing to do
}

PoseSamplingPoseSource::~PoseSamplingPoseSource() {
    // nothing to do
}

void PoseSamplingPoseSource::getPoses(const KinematicChain& kinematicChain,
        vector<MeasurementPose>& poses) {
    int posePoolSize = 500;
    nhPrivate.param("pose_pool_size", posePoolSize, posePoolSize);
    ROS_ERROR("PoseSamplingSource is disabled because of a moveit bug! FIX ME");
    throw std::runtime_error("PoseSamplingSource::getPoses unavailable!");
    //PoseSampling poseSampling;
    //poseSampling.getPoses(posePoolSize, poses);
}

}
