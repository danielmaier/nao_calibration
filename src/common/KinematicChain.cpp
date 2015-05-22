/*
 * KinematicChain.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../../include/common/KinematicChain.h"
#include <cstddef>
#include "../../include/common/ModelLoader.h"

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>
#include <utility>
#include <vector>

using namespace std;
using namespace KDL;

namespace kinematic_calibration {

KinematicChain::KinematicChain(const KDL::Tree& tree, std::string rootIn,
        std::string tipIn, std::string nameIn) :
        root(rootIn), tip(tipIn), name(nameIn) {
	initialize(tree, root, tip, name);
}

KinematicChain::~KinematicChain() {

}

bool KinematicChain::initialize(const KDL::Tree& tree, std::string root,
		std::string tip, std::string name) {
	this->root = root;
	this->tip = tip;
	this->name = name;
	this->tree = KDL::Tree(tree);

    ROS_DEBUG("Extracting chain from %s to %s...", root.c_str(), tip.c_str());
	if (!tree.getChain(root, tip, this->chain)) {
        ROS_ERROR("Could not extract the chain from %s to %s!", root.c_str(), tip.c_str());
		return false;
	}
	return true;
}

bool KinematicChain::initializeFromRos() {
	// instantiate the kinematic chain
	string chainName, chainRoot, chainTip;
	ros::NodeHandle nh;
	nh.getParam("chain_name", chainName);
	nh.getParam("chain_root", chainRoot);
	nh.getParam("chain_tip", chainTip);

	// delegate
	return this->initializeFromRos(chainRoot, chainTip, chainName);
}

bool KinematicChain::initializeFromRos(const std::string root, const std::string tip,
		const std::string name) {
	// instantiate the KDL tree
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(tree);

	// delegate
	return this->initialize(tree, root, tip, name);
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		KDL::Frame& out) const {
	KDL::Frame rootToTip = KDL::Frame::Identity();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		const KDL::Segment& segment = chain.getSegment(i);
		const KDL::Joint& joint = segment.getJoint();

		double position = 0.0;
		std::map<std::string, double>::const_iterator jnt =
				joint_positions.find(joint.getName());
		if (jnt != joint_positions.end()) {
			position = jnt->second;
		}
		// Note: Frame F_A_C = F_A_B * F_B_C;
		// (see http://www.orocos.org/kdl/usermanual/geometric-primitives#toc20)
		rootToTip = rootToTip * segment.pose(position);
	}
	out = rootToTip.Inverse();
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets, KDL::Frame& out) const {
	map<string, double> sum;

	// iterate through all positions
	for (map<string, double>::const_iterator outer_jnt_it =
			joint_positions.begin(); outer_jnt_it != joint_positions.end();
			outer_jnt_it++) {
		double position = outer_jnt_it->second;
		// add (optional) joint offset
		if (joint_offsets.count(outer_jnt_it->first)) {
			position += joint_offsets.find(outer_jnt_it->first)->second;
        }
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
        sum.insert(make_pair(outer_jnt_it->first, position));
#else
        sum.insert(make_pair<string, double>(outer_jnt_it->first, position));
#endif
	}

	// delegate call
	getRootToTip(sum, out);
}

void KinematicChain::getJointWithOffset(const KDL::Joint& old_joint,
		double offset, KDL::Joint& new_joint) const {
	new_joint = KDL::Joint(old_joint.getName(), old_joint.JointOrigin(),
			old_joint.JointAxis(), old_joint.getType(), 1, offset);
}

void KinematicChain::getSegmentWithJointOffset(const KDL::Segment& old_segment,
		double offset, KDL::Segment new_segment) const {
	// create the new joint object
	KDL::Joint old_joint = old_segment.getJoint();
	KDL::Joint new_joint;
	getJointWithOffset(old_joint, offset, new_joint);

	// create the new segment object
	new_segment = KDL::Segment(old_segment.getName(), new_joint,
			old_segment.getFrameToTip(), old_segment.getInertia());
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		tf::Transform& out) const {
	KDL::Frame frame;
	getRootToTip(joint_positions, frame);
	kdlFrameToTfTransform(frame, out);
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets, tf::Transform& out) const {
	KDL::Frame frame;
	getRootToTip(joint_positions, joint_offsets, frame);
	kdlFrameToTfTransform(frame, out);
}

KinematicChain KinematicChain::withFrames(
		const map<string, KDL::Frame> framesToTip) {
	KinematicChain copy;
	copy = *this;

	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		string name = joint.getName();

		// no change for this segment
		if (framesToTip.count(name) == 0)
			continue;

		KDL::Segment newSegment(string(segment.getName()),
				Joint(segment.getJoint()),
				Frame(framesToTip.find(name)->second),
				RigidBodyInertia(segment.getInertia()));
		copy.chain.segments[i] = newSegment;
	}
	return copy;
}

KinematicChain KinematicChain::withTransformations(
		const map<string, tf::Transform> transformationsToTip) {
	// convert from TF to KDL
	map<string, KDL::Frame> framesToTip;
	for (map<string, tf::Transform>::const_iterator it =
			transformationsToTip.begin(); it != transformationsToTip.end();
			it++) {
		tf::Transform tfTransform = it->second;
		KDL::Frame kdlFrame;
        tf::transformTFToKDL(tfTransform, kdlFrame);
		framesToTip[it->first] = kdlFrame;
	}

	// delegate
	return withFrames(framesToTip);
}

map<string, Frame> KinematicChain::getFramesToTip() const {
	map<string, Frame> frames;
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		string name = joint.getName();
		Frame frameToTip = segment.getFrameToTip();
		frames[name] = frameToTip;
	}
	return frames;
}

void KinematicChain::kdlFrameToTfTransform(const KDL::Frame& in,
		tf::Transform& out) const {
	tf::transformKDLToTF(in, out);
}

void KinematicChain::getJointNames(vector<string>& jointNames) const {
	jointNames.clear();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		if (joint.getType() != KDL::Joint::None) {
			jointNames.push_back(joint.getName());
			//ROS_INFO("Added joint name %s", joint.getName().c_str());
		} else {
			//ROS_INFO("Fixed joint %s", joint.getName().c_str());
		}
	}
}

} /* namespace kinematic_calibration */
