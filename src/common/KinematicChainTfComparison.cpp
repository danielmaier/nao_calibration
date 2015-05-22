#include "../../include/common/KinematicChainTfComparison.h"

#include <ros/duration.h>
#include <ros/init.h>
#include <tf/tf.h>
#include <map>
#include <vector>

int main(int argc, char** argv) {
	ros::init(argc, argv, "KinematicChainTfComparison");
	kinematic_calibration::KinematicChainTfComparison node;
	ROS_INFO("Sleeping...");
	usleep(3*1e6);
	ROS_INFO("Done!");
	ros::spin();
	return 0;
}

namespace kinematic_calibration {

KinematicChainTfComparison::KinematicChainTfComparison(string root,
		string tip) :
		root(root), tip(tip), tfListener(ros::Duration(30.0)) {
	jsSubscriber = nh.subscribe("/joint_states", 1,
			&KinematicChainTfComparison::jsCb, this);
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);
	this->kinematicChain = new KinematicChain(tree, root, tip, "");
}

inline KinematicChainTfComparison::~KinematicChainTfComparison() {
	delete kinematicChain;
}

void KinematicChainTfComparison::jsCb(
		const sensor_msgs::JointStateConstPtr& js) {
	// calculate the transform using the KinematicChain object
	tf::Transform kcTransform;
	map<string, double> jointStates;
	for (int i = 0; i < js->name.size(); i++) {
		jointStates[js->name[i]] = js->position[i];
	}
	this->kinematicChain->getRootToTip(jointStates, kcTransform);
	// get the transform from TF
	tf::StampedTransform tfTransform;
	ros::Time time = js->header.stamp;
	//cout << this->tfListener.allFramesAsString() << "\n";
	ROS_INFO("Waiting for transform...");
	this->tfListener.waitForTransform(tip, root, time, ros::Duration(10.0), ros::Duration(1.0));
	ROS_INFO("Done! Now, get the transform...");
	this->tfListener.lookupTransform(tip, root, time, tfTransform);
	ROS_INFO("Done!");

	cout << "KC: (x, y, z) " << kcTransform.getOrigin().x() << " "
			<< kcTransform.getOrigin().y() << " "
			<< kcTransform.getOrigin().z() << " ";
	cout << "(q0, q1, q2, q3) " << kcTransform.getRotation().x()
			<< " " << kcTransform.getRotation().y() << " "
			<< kcTransform.getRotation().z() << " "
			<< kcTransform.getRotation().w() << " ";
	cout << endl;

	cout << "TF: (x, y, z) " << tfTransform.getOrigin().x() << " "
			<< tfTransform.getOrigin().y() << " "
			<< tfTransform.getOrigin().z() << " ";
	cout << "(q0, q1, q2, q3) " << tfTransform.getRotation().x()
			<< " " << tfTransform.getRotation().y() << " "
			<< tfTransform.getRotation().z() << " "
			<< tfTransform.getRotation().w() << " ";
	cout << endl;
}

}
