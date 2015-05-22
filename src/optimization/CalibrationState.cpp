/*
 * CalibrationState.cpp
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#include <optimization/CalibrationState.h>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_kdl.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>

#include <common/ModelLoader.h>

namespace kinematic_calibration {

using namespace ros;

CalibrationState::CalibrationState() :
		jointOffsets(map<string, double>()), markerTransformations(
                map<string, tf::Transform>()), cameraTransformation(
                tf::Transform()), jointTransformations(map<string, tf::Transform>()) {
}

CalibrationState::CalibrationState(
		map<string, double>& jointOffsets, map<string, tf::Transform>,
		tf::Transform& cameraToHeadTransformation) :
        jointOffsets(jointOffsets), markerTransformations(markerTransformations),
        cameraTransformation(cameraToHeadTransformation) {
}

CalibrationState::~CalibrationState() {
}


std::vector<std::string> CalibrationState::getJointNames() const
{
  std::vector<std::string> jointNames;
  for (map<string,double>::const_iterator it = jointOffsets.begin(); it != jointOffsets.end(); ++it)
  {
    jointNames.push_back(it->first);
  }
  return jointNames;
}


void CalibrationState::addMarker(const string chainName, const string root, const string tip, TransformSource source)
{
    tf::Transform markerTransform;
    string sourceString = "";
    if (TF == source)
    {
        sourceString = "TF";
        tf::TransformListener transformListener;
        usleep(1e6);
        ros::Time now = ros::Time::now();
        tf::StampedTransform transform;
        transformListener.waitForTransform(root, tip, now, ros::Duration(1.0));
        transformListener.lookupTransform(root, tip, now, transform);
        markerTransform.setRotation(transform.getRotation());
        markerTransform.setOrigin(transform.getOrigin());
    } else if (ROSPARAM_URDF == source) {
        sourceString = "ROSPARAM_URDF";
        KinematicChain markerChain;
        markerChain.initializeFromRos(root, tip, "marker");
        KDL::Frame kdlFrame;
        markerChain.getRootToTip(map<string, double>(), kdlFrame);
        tf::transformKDLToTF(kdlFrame, markerTransform);
    }
    this->markerTransformations[chainName] = markerTransform;
    ROS_INFO("Initial transform for marker of chain %s (%s -> %s): %f %f %f (source: %s)",
             chainName.c_str(), root.c_str(), tip.c_str(),
             markerTransform.getOrigin().getX(),
             markerTransform.getOrigin().getY(),
             markerTransform.getOrigin().getZ(), sourceString.c_str());
}

map<string, KDL::Frame> CalibrationState::getJointFrames() const {
	map<string, KDL::Frame> jointFrames;
	for (map<string, tf::Transform>::const_iterator it = jointTransformations.begin();
			jointTransformations.end() != it; ++it) {
		KDL::Frame frame;
		string name = it->first;
		tf::transformTFToKDL(it->second, frame);
		jointFrames[name] = frame;
	}
	return jointFrames;
}

ostream & operator<<(ostream &output, const CalibrationState &state) {

    output << "cameraToHeadTransformation: ";
    output << state.cameraTransformation.getOrigin().x() << " ";
    output << state.cameraTransformation.getOrigin().y() << " ";
    output << state.cameraTransformation.getOrigin().z() << " ";
    output << state.cameraTransformation.getRotation().x() << " ";
    output << state.cameraTransformation.getRotation().y() << " ";
    output << state.cameraTransformation.getRotation().z() << " ";
    output << state.cameraTransformation.getRotation().w() << " ";
    output << "cameraInfo: ";
    for (int i = 0; i < 12; i++)
        output << state.cameraInfo.P[i] << " ";
    for (int i = 0; i < 5; i++)
        output << state.cameraInfo.D[i] << " ";
    output << "marker transformations: ";
    for (map<string, tf::Transform>::const_iterator it =
         state.markerTransformations.begin();
         it != state.markerTransformations.end(); it++) {
        output << it->second.getOrigin().x() << " ";
        output << it->second.getOrigin().y() << " ";
        output << it->second.getOrigin().z() << " ";
        output << it->second.getRotation().x() << " ";
        output << it->second.getRotation().y() << " ";
        output << it->second.getRotation().z() << " ";
        output << it->second.getRotation().w() << " ";
    }
    output << "joint offsets: ";
    for (map<string, double>::const_iterator it =
         state.jointOffsets.begin(); it != state.jointOffsets.end();
         it++) {
        output << it->first << ": ";
        output << it->second;
    }
    return output;
}

} /* namespace kinematic_calibration */

