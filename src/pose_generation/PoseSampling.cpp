/*
 * PoseSampling.cpp
 *
 *  Created on: 16.04.2014
 *      Author: stefan
 */

#include <pose_generation/PoseSampling.h>
#include <common/helpers.h>

#include <boost/smart_ptr/make_shared.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iterator>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/version.h>
#if ROS_VERSION_MINIMUM(1,10,2)
#include <urdf_parser/urdf_parser.h>
#endif

using namespace boost;
using namespace std;

namespace kinematic_calibration {

PoseSampling::PoseSampling() :
		debug(false), nhPrivate("~"), xMin(20), xMax(620), yMin(20), yMax(460), cameraFrame(
				"CameraBottom_frame"), viewCylinderRadius(0.01), srdfAvailable(
				false), torsoFrame("torso"), initialPoseAvailable(false), testPoseStability(
				false), keepEndEffectorPose(false), jointLimitsDistance(0.1), jointLimitsLinearWidth(
				0.3) {
	// initialize stuff
	this->initialize();

	// register the joint state publisher
	this->jointStatePub = nh.advertise<sensor_msgs::JointState>("/joint_states",
			1000);
}

PoseSampling::~PoseSampling() {

}

void PoseSampling::initialize() {
	this->initializeCamera();
	this->initializeKinematicChain();
	this->initializeState();
	this->initializeJointLimits();
	this->initializeInitialPose();

	// initialize parameter from ROS via nhPrivate or nh
	nhPrivate.param("xMin", xMin, xMin);
	nhPrivate.param("xMax", xMax, xMax);
	nhPrivate.param("yMin", yMin, yMin);
	nhPrivate.param("yMax", yMax, yMax);
	nhPrivate.param("camera_frame", cameraFrame, cameraFrame);
	nhPrivate.param("torso_frame", torsoFrame, torsoFrame);
	nhPrivate.param("view_cylinder_radius", viewCylinderRadius,
			viewCylinderRadius);
	nhPrivate.param("test_pose_stability", testPoseStability,
			testPoseStability);
	nhPrivate.param("keep_end_effector_pose", keepEndEffectorPose,
			keepEndEffectorPose);
	nhPrivate.param("joint_limits_distance", jointLimitsDistance,
			jointLimitsDistance);
	nhPrivate.param("joint_limits_linear_width", jointLimitsLinearWidth,
			jointLimitsLinearWidth);

	// print some info about the used parameters
	ROS_INFO("Allowed camera window size: [%.2f, %.2f] - [%.2f, %.2f]", xMin,
			yMin, xMax, yMax);
	ROS_INFO("Using camera frame: %s", cameraFrame.c_str());
	ROS_INFO("Using torso frame: %s", torsoFrame.c_str());
	ROS_INFO("Using view cylinder radius: %f", viewCylinderRadius);
	ROS_INFO("Pose stability WILL %s tested.",
			testPoseStability ? "BE" : "NOT BE");
	ROS_INFO("End effector WILL %s fixed.",
			keepEndEffectorPose ? "BE" : "NOT BE");
	ROS_INFO("joint_limits_distance is %f", jointLimitsDistance);
	ROS_INFO("joint_limits_linear_width is %f", jointLimitsLinearWidth);

	if (testPoseStability) {
		this->testStabilityPtr = boost::make_shared<
				hrl_kinematics::TestStability>();
	}

	if (keepEndEffectorPose) {
		// initialize the IK request
		initializeEndEffectorState();
	}

	// initialize the RNG
	this->rng = boost::make_shared<MarginDiscriminatingUniformRNG>(
			jointLimitsLinearWidth);
}

void PoseSampling::initializeCamera() {
	string topic = "/nao_camera/camera_info"; // TODO: parameterize!
	cameraInfoSubscriber = nh.subscribe(topic, 1,
			&PoseSampling::camerainfoCallback, this);
	ROS_INFO("Waiting for camera info message...");
	while (ros::getGlobalCallbackQueue()->isEmpty() && ros::ok()) {
	}
	ROS_INFO("Done!");
	ros::getGlobalCallbackQueue()->callAvailable();
	cameraInfoSubscriber.shutdown();
}

void PoseSampling::initializeKinematicChain() {
	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);

	// instantiate the kinematic chain
	string chainName, chainRoot, chainTip;
	nh.getParam("chain_name", chainName);
	nh.getParam("chain_root", chainRoot);
	nh.getParam("chain_tip", chainTip);

	this->kinematicChainPtr = boost::make_shared<KinematicChain>(kdlTree,
			chainRoot, chainTip, chainName);
}

void PoseSampling::initializeState() {
	this->initialState = boost::make_shared<CalibrationState>();
	this->modelLoader.getUrdfModel(this->robotModel);


        // retrieve the parent joint of the camera link from the urdf (remove leading slash if necessary)
        boost::shared_ptr<const urdf::Link> link = this->robotModel.getLink(withoutLeadingSlash(cameraModel.tfFrame()));
        if (!link)
           throw std::runtime_error("Could not find parent link of camera in urdf");
        boost::shared_ptr<const urdf::Joint> cameraJoint = link->parent_joint;
        urdf::Pose neckToCameraPose = cameraJoint->parent_to_joint_origin_transform;
        // convert urdf::Pose to a tf::Transform
        tf::Quaternion quat (neckToCameraPose.rotation.x, neckToCameraPose.rotation.y, neckToCameraPose.rotation.z, neckToCameraPose.rotation.w);
        tf::Vector3 trans (neckToCameraPose.position.x, neckToCameraPose.position.y, neckToCameraPose.position.z);
        tf::Transform neckToCameraTransform = tf::Transform(quat, trans);
        // update the state
        this->initialState->cameraTransformation = neckToCameraTransform;


	// initialize the joint offsets (with 0.0)
	// this->initialState->addKinematicChain(*this->kinematicChainPtr);

	// initialize transform from camera to head
	//this->initialState->cameraJointName = "CameraBottom";
	//this->initialState->initializeCameraTransform();

	// initialize the camera intrinsics
	//initialState->cameraInfo = cameraModel.cameraInfo();

	// initialize marker transformation, if available
	if (nh.hasParam("marker_frame")) {
		string markerFrame;
		nh.getParam("marker_frame", markerFrame);
		this->initialState->addMarker(this->kinematicChainPtr->getName(),
				this->kinematicChainPtr->getTip(), markerFrame,
				CalibrationState::ROSPARAM_URDF);
	} else {
		ROS_WARN(
				"No initialization for marker frame transformation available!");
	}
}

void PoseSampling::initializeJointLimits() {
	// initialize the joint limits of the interesting joints (i.e. that ones of the current chain)
	this->kinematicChainPtr->getJointNames(jointNames);
	for (int i = 0; i < jointNames.size(); i++) {
		string jointName = jointNames[i];
		double upperLimit = this->robotModel.getJoint(jointName)->limits->upper;
		double lowerLimit = this->robotModel.getJoint(jointName)->limits->lower;
		this->lowerLimits[jointName] = lowerLimit + jointLimitsDistance;
		this->upperLimits[jointName] = upperLimit - jointLimitsDistance;
		if (this->lowerLimits[jointName] >= this->upperLimits[jointName]) {
			ROS_FATAL(
					"Joint range of %s is empty: Make sure that the limits given "
							"in the URDF are valid and the specified parameter"
							"'joint_limits_distance' is not too big!",
					jointName.c_str());
		}
	}
}

void PoseSampling::initializeUrdf() {
	this->urdfModelPtr = make_shared<urdf::Model>();
	this->modelLoader.initializeFromRos();
	this->modelLoader.getUrdfModel(*urdfModelPtr);
}

void PoseSampling::initializeSrdf(const string& robotName,
		const vector<string>& joints, const vector<string>& links) {
	srdfString = "";
	const string robotNamePlaceholder = "ROBOTNAME";
	const string groupPlaceholder = "CURRENTCHAINGROUP";

	if (nh.hasParam("robot_description_semantic")) {
		// get the SRDF string from the parameter server
		nh.getParam("robot_description_semantic", srdfString);
		this->srdfAvailable = true;
	} else {
		// as fallback, create an empty srdf
		stringstream srdfStringStream;
		srdfStringStream << "<robot name=\"" << robotNamePlaceholder << "\">";
		srdfStringStream << "CURRENTCHAINGROUP" << "\n";
		srdfStringStream << "</robot>";
		srdfString = srdfStringStream.str();
	}

	if (debug) {
		cout << "SRDF (unmodified): \n" << srdfString << endl;
	}

	// replace the ROBOT placeholder
	int namePos = srdfString.find(robotNamePlaceholder);
	srdfString.replace(namePos, robotNamePlaceholder.length(), robotName);

	// insert the interesting joints/links as group
	stringstream currentChainStream;
	currentChainStream << "<group name=\"current_chain\">";
	for (int i = 0; i < joints.size(); i++) {
		currentChainStream << "<joint name=\"" << joints[i] << "\"/>";
	}
	for (int i = 0; i < links.size(); i++) {
		currentChainStream << "<link name=\"" << links[i] << "\"/>";
	}
	currentChainStream << "</group>";
	string currentChainString = currentChainStream.str();
	int groupPos = srdfString.find(groupPlaceholder);
	srdfString.replace(groupPos, groupPlaceholder.length(), currentChainString);

	if (debug) {
		cout << "SRDF (modified): \n" << srdfString << endl;
	}

	// initialize the SRDF model object from the string
	srdf::Model srdfModel;
	srdfModel.initString(*urdfModelPtr, srdfString);
	this->srdfModelPtr = make_shared<const srdf::Model>(srdfModel);
}

void PoseSampling::initializeInitialPose() {
	// get the name of the initial pose
	string initialPoseName;
	if (!nhPrivate.hasParam("initial_pose_name")) {
		// no pose name given
		this->initialPoseAvailable = false;
		return;
	}
	nhPrivate.getParam("initial_pose_name", initialPoseName);

	// check whether this pose actually exists
	if (!nhPrivate.hasParam("poses/" + initialPoseName)) {
		// pose not available
		this->initialPoseAvailable = false;
		ROS_WARN("Initial pose name '%s' given, but no such pose found!",
				initialPoseName.c_str());
		return;
	}

	// get the initial pose
	nhPrivate.param("poses/" + initialPoseName + "/joint_names",
			this->initialPose.name, this->initialPose.name);
	nhPrivate.param("poses/" + initialPoseName + "/positions",
			this->initialPose.position, this->initialPose.position);

	if (this->initialPose.name.size() == 0
			|| this->initialPose.position.size() == 0) {
		this->initialPoseAvailable = false;
		ROS_WARN("Initial pose '%s' found, but something is wrong with it.",
				initialPoseName.c_str());
		ROS_WARN("The pose shoud be under the namespace 'poses'"
				" and should have two arrays name 'joint_names' "
				"and 'positions' of the same size.");
		return;
	}

	ROS_INFO("Initial pose successfully loaded.");
	this->initialPoseAvailable = true;
}

void PoseSampling::initializeEndEffectorState() {
	// initialize the kinematic chain from torso to tip
	KinematicChain torsoToTip;
	torsoToTip.initializeFromRos(this->torsoFrame,
			this->kinematicChainPtr->getTip(), "torsoToTip");

	map<string, double> jointPositions;
	for (int i = 0; i < initialPose.name.size(); i++) {
		jointPositions[initialPose.name[i]] = initialPose.position[i];
	}

	torsoToTip.getRootToTip(jointPositions, this->endEffectorState);
}

void PoseSampling::getPoses(const int& numOfPoses,
		vector<MeasurementPose>& poses) {
	ros::Publisher robot_state_publisher;
	if (debug) {
		robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>(
				"moveit_robot_state", 1);
	}

	// prepare the additional link/joint for between camera and marker
    boost::shared_ptr<urdf::Joint> markerJoint = make_shared<urdf::Joint>();
	markerJoint->parent_link_name = cameraFrame;
	markerJoint->child_link_name = "virtualMarkerLink";
	markerJoint->name = "virtualMarkerJoint";
	markerJoint->type = urdf::Joint::FIXED;

    boost::shared_ptr<urdf::Link> markerLink = make_shared<urdf::Link>();
	markerLink->parent_joint = markerJoint;
	markerLink->name = "virtualMarkerLink";

    // TODO: Use a better shape?
    boost::shared_ptr<urdf::Collision> markerLinkCollision = make_shared<
			urdf::Collision>();
    boost::shared_ptr<urdf::Cylinder> cylinder = make_shared<urdf::Cylinder>();
	cylinder->type = urdf::Geometry::CYLINDER;
	cylinder->radius = viewCylinderRadius;
	cylinder->length = 0.0; // real length is updated later
	markerLinkCollision->geometry = cylinder;

	urdf::Pose urdfCollisionPose;
	markerLinkCollision->origin = urdfCollisionPose;

	// initialize the URDF model
	ROS_INFO("Initialize the URDF model from ROS...");
	initializeUrdf();

	// modify the URDF model
	ROS_INFO(
			"Adding additional (virtual) joint and link from camera to marker frame to the model...");
	//urdfModelPtr->links_[cameraFrame]->collision = markerLinkCollision;
	this->urdfModelPtr->links_[cameraFrame]->child_joints.push_back(
			markerJoint);
	this->urdfModelPtr->links_[cameraFrame]->child_links.push_back(markerLink);
	this->urdfModelPtr->joints_[markerJoint->name] = markerJoint;
	this->urdfModelPtr->links_[markerLink->name] = markerLink;
	markerLink->setParent(urdfModelPtr->links_[cameraFrame]);

	// initialize the SRDF model
	ROS_INFO("Initialize the SRDF model from runtime information...");
	vector<string> groupJoints, groupLinks;
	groupJoints = this->jointNames;
	groupJoints.push_back("CameraBottom");
	groupLinks.push_back(cameraFrame);
	initializeSrdf(urdfModelPtr->getName(), groupJoints, groupLinks);

	// initialize the moveit robot model
	robot_model_loader::RobotModelLoader::Options options(
			this->modelLoader.getUrdfXml(), srdfString);
	robot_model_loader::RobotModelLoader robot_model_loader(options);
	robot_model_loader.loadKinematicsSolvers();
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	//robot_model::RobotModelPtr kinematic_model = make_shared<
	//		robot_model::RobotModel>(urdfModelPtr, srdfModelPtr);

	if (debug) {
		kinematic_model->printModelInfo(cout);
	}

    // sample as many poses as requested
	ROS_INFO("Sampling %d poses now...", numOfPoses);
	while (poses.size() < numOfPoses && ros::ok()) {
        ROS_INFO("sampling another pose");

		// sample joint states within the allowed limit
		sensor_msgs::JointState jointState;
		for (int i = 0; i < jointNames.size(); i++) {
			string jointName = jointNames[i];
			double lowerLimit = lowerLimits[jointName];
			double upperLimit = upperLimits[jointName];
			double currentJointState = this->rng->getRandom(lowerLimit,
					upperLimit);
			jointState.name.push_back(jointName);
			jointState.position.push_back(currentJointState);
		}

		// use IK to generate a pose, using the sampled pose as seed state
		if (this->keepEndEffectorPose) {
			// initialize the IK solver (TODO: only once, in own method)
			KDL::Chain chain;
			this->kdlTree.getChain(this->torsoFrame,
					this->kinematicChainPtr->getTip(), chain);
			unsigned int nj = chain.getNrOfJoints();

			KDL::JntArray q_min(nj);
			KDL::JntArray q_max(nj);
			for (int i = 0; i < nj; i++) {
				q_min(i) =
						this->lowerLimits[chain.getSegment(i).getJoint().getName()];
				q_max(i) =
						this->upperLimits[chain.getSegment(i).getJoint().getName()];
			}
			this->kdlTree.getChain(this->torsoFrame,
					this->kinematicChainPtr->getTip(), chain);
			KDL::ChainFkSolverPos_recursive fkSolverPos(chain);	//Forward position solver
			KDL::ChainIkSolverVel_pinv ikSolverVel(chain);//Inverse velocity solver
			KDL::ChainIkSolverPos_NR_JL ikSolver(chain, q_min, q_max,
					fkSolverPos, ikSolverVel, 10000, 0.01);

			// seed state
			KDL::JntArray seedState(nj);

			// initialize with initial pose
			for (int i = 0; i < nj; i++) {
				string currentJoint = chain.getSegment(i).getJoint().getName();
				for (int j = 0; j < initialPose.name.size(); j++) {
					if (currentJoint == initialPose.name[j]) {
						seedState(i) = initialPose.position[j];
					}
				}
			}

			// use this as target
			KDL::Frame frame;
			fkSolverPos.JntToCart(seedState, frame);

			// add random values
			for (int i = 0; i < nj; i++) {
				string currentJoint = chain.getSegment(i).getJoint().getName();
				for (int j = 0; j < jointState.name.size(); j++) {
					if (currentJoint == jointState.name[j]) {
						seedState(i) += 0.1 * jointState.position[j];
					}
				}
			}

			// inverse kinematics
			KDL::JntArray targetState(nj);
			int ikSuccess = ikSolver.CartToJnt(seedState, frame, targetState);

			if (debug) {
				cout << "ikSuccess: " << ikSuccess << endl;
				cout << "seed state: ";
				for (int i = 0; i < nj; i++)
					cout << seedState(i) << ", ";
				cout << endl;
				cout << "target state: ";
				for (int i = 0; i < nj; i++)
					cout << targetState(i) << ", ";
				cout << endl;
				cout << "target transformation: " << frame.p[0] << " "
						<< frame.p[1] << " " << frame.p[2] << endl;
				fkSolverPos.JntToCart(targetState, frame);
				cout << "IK transformation: " << frame.p[0] << " " << frame.p[1]
						<< " " << frame.p[2] << endl;
			}

			if (ikSuccess < 0)
				continue;

			// save the results
			for (int i = 0; i < nj; i++) {
				string currentJoint = chain.getSegment(i).getJoint().getName();
				for (int j = 0; j < jointState.name.size(); j++) {
					if (currentJoint == jointState.name[j]) {
						jointState.position[j] = targetState(i);
					}
				}
			}
		}

		// --------------------------------------------------------------------------------
		// check #1: The predicted coordinates must be within
		// 			[xMin,yMin] and [xMax,yMax], e.g. [0,640]-[0,480].
		// --------------------------------------------------------------------------------
		double x, y;
        // TODO: Set ID before adding to pool
        MeasurementPose pose(*this->kinematicChainPtr, jointState, "");

		pose.predictImageCoordinates(*this->initialState, x, y);

		if (debug) {
			cout << "Predicted image coordinates: \t" << x << "\t" << y;
		}

		if (xMin < x && xMax > x && yMin < y && yMax > y) {
			if (debug)
				cout << "--> Good!" << endl;
		} else {
			if (debug)
				cout << "--> Not good!" << endl;
			continue;
		}

		// --------------------------------------------------------------------------------
		// check #2: Is the marker is visible to the camera?
		//			To check this, add an object (cylinder) between the camera and the
		//			marker and look for collisions with the robot.
		// --------------------------------------------------------------------------------

		// a) update the URDF model (virtual direct joint/link between camera and marker)

		// pose of the (virtual) marker joint
		tf::Transform cameraToMarker;
		pose.predictEndEffectorPose(*this->initialState, cameraToMarker);
		cameraToMarker = cameraToMarker.inverse();

		urdf::Pose urdfJointPose;
		urdfJointPose.position = urdf::Vector3(
				cameraToMarker.getOrigin().getX(),
				cameraToMarker.getOrigin().getY(),
				cameraToMarker.getOrigin().getZ());
		urdfJointPose.rotation = urdf::Rotation(0, 0, 0, 1);

		markerJoint->parent_to_joint_origin_transform = urdfJointPose;
		cylinder->length = std::max(cameraToMarker.getOrigin().length() - 0.01,
				0.01);

		// collision object: rotation (relative to the camera frame!)
		Eigen::Quaternion<double> q;
		q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1),
				Eigen::Vector3d(-urdfJointPose.position.x,
						-urdfJointPose.position.y, -urdfJointPose.position.z));
		markerLinkCollision->origin.rotation = urdf::Rotation(q.x(), q.y(),
				q.z(), q.w());

		// collision object: origin (relative to the camera frame!)
		markerLinkCollision->origin.position = urdf::Vector3(
				urdfJointPose.position.x / 2, urdfJointPose.position.y / 2,
				urdfJointPose.position.z / 2);

		// b) insert the collision object into the moveit model/state

		// get the planning scene
        planning_scene::PlanningScene planning_scene(kinematic_model);

		// get the state
		robot_state::RobotState& current_state =
				planning_scene.getCurrentStateNonConst();

		// add the collision shape as attached body
		std::vector<shapes::ShapeConstPtr> shapes;
		boost::shared_ptr<shapes::Shape> shape = boost::make_shared<
				shapes::Cylinder>(cylinder->radius, cylinder->length);
		shapes.push_back(shape);

		EigenSTL::vector_Affine3d attachTrans;
		Eigen::Affine3d eigenCollisionTrans;
		tf::transformTFToEigen(
				tf::Transform(
						tf::Quaternion(markerLinkCollision->origin.rotation.x,
								markerLinkCollision->origin.rotation.y,
								markerLinkCollision->origin.rotation.z,
								markerLinkCollision->origin.rotation.w),
						tf::Vector3(markerLinkCollision->origin.position.x,
								markerLinkCollision->origin.position.y,
								markerLinkCollision->origin.position.z)),
				eigenCollisionTrans);
		attachTrans.push_back(eigenCollisionTrans);

		std::set<std::string> touchLinks;
		std::string attachLinkName = cameraFrame;
		string attachedBodyName = "marker";
		current_state.clearAttachedBodies();
		current_state.attachBody(attachedBodyName, shapes, attachTrans,
				touchLinks, attachLinkName);

		// set the joint state
#if MOVEIT_VERSION_MINOR >= 5 && MOVEIT_VERSION_PATCH >= 8 // moveit from hydro and newer
		current_state.setVariablePositions(jointState.name, jointState.position);
#else  // moveit from groovy and below
		current_state.setStateValues(jointState.name, jointState.position);
#endif
		if (debug)
			current_state.printStateInfo(cout);

		// c) check for self-collisions
		if (debug)
			ROS_INFO("check for self-collisions");
		collision_detection::CollisionRequest collision_request;
		collision_detection::CollisionResult collision_result;
		collision_result.clear();
		collision_request.group_name = "current_chain";
		collision_request.contacts = true;
		collision_request.max_contacts = 1000;
		collision_request.verbose = debug;
		planning_scene.checkSelfCollision(collision_request, collision_result);
		if (debug) {
			ROS_INFO_STREAM(
					"Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
		}
		bool collision = false;
		for (collision_detection::CollisionResult::ContactMap::const_iterator it =
				collision_result.contacts.begin();
				it != collision_result.contacts.end(); ++it) {
			// only contacts with the "virtual" link (i.e. collision object) are interesting
			string linkToCheck = attachedBodyName;
			if (this->srdfAvailable || linkToCheck == it->first.first.c_str()
					|| linkToCheck == it->first.second.c_str()) {
				if (debug)
					ROS_INFO("Contact between: %s and %s",
							it->first.first.c_str(), it->first.second.c_str());
				collision = true;
			}
		}

		// collision detected
		if (collision) {
			continue;
		}

		// --------------------------------------------------------------------------------
		// check #3: pose stability (optional)
		// --------------------------------------------------------------------------------
		if (this->testPoseStability) {
			bool poseStable = false;
			poseStable = isPoseStable(jointState);
			if (!poseStable)
				continue;
		}

		// add to the pose set
        // TODO: Set the ID of the pose here, e.g. [chain][number]-[time]
        stringstream ss;
        ss << boost::format("%s%3d") % this->kinematicChainPtr->getName(), poses.size();
        pose.setID(ss.str());
		poses.push_back(pose);

		// print some info
		cout << "Number of sampled poses: " << poses.size() << "\t";
		cout << "Predicted image coordinates: \t" << x << "\t" << y << endl;

		// debug: publish the urdf model, moveit state and joint state
		if (debug) {
			// urdf
			stringstream urdfStringStream;
			urdfStringStream << *urdf::exportURDF(*urdfModelPtr);
			string urdfString = urdfStringStream.str();
			nh.setParam("/robot_description", urdfString);
			tf::TransformBroadcaster broadcaster;
			string cameraFrame = markerJoint->parent_link_name;
			string markerFrame = markerJoint->child_link_name;

			// moveit
			moveit_msgs::DisplayRobotState msg;
			robot_state::robotStateToRobotStateMsg(current_state, msg.state);

			ros::Rate rate(30);
			while (ros::ok()) {
				robot_state_publisher.publish(msg);
				publishJointState(jointState);
				tf::StampedTransform stampedCameraToMarker(cameraToMarker,
						ros::Time::now(), cameraFrame, markerFrame);
				broadcaster.sendTransform(stampedCameraToMarker);
				rate.sleep();
			}
		}
        ROS_INFO("finished one iteration");

	}
}

bool PoseSampling::isPoseStable(const sensor_msgs::JointState& msg) const {
	// "merge": initialPose + msg -> poseToCheck
	map<string, double> jointPositions;
	for (int i = 0; i < initialPose.name.size(); i++) {
		jointPositions[initialPose.name[i]] = initialPose.position[i];
	}
	for (int i = 0; i < msg.name.size(); i++) {
		jointPositions[msg.name[i]] = msg.position[i];
	}

	// delegate stability test
	bool stable = this->testStabilityPtr->isPoseStable(jointPositions,
			hrl_kinematics::TestStability::SUPPORT_DOUBLE);

	if (debug) {
		ROS_INFO("Pose is %s!", stable ? "STABLE" : "UNSTABLE");
	}

	return stable;
}

void PoseSampling::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	} else {
		ROS_FATAL("Camera model could not be set!");
	}
}

void PoseSampling::publishJointState(sensor_msgs::JointState& msg) const {
	msg.header.stamp = ros::Time::now();
	this->jointStatePub.publish(msg);
}

void RandomNumberGenerator::plot(const double min, const double max,
		const int numOfSamples) {
	FILE * gnuplotPipe;
	gnuplotPipe = popen("gnuplot -p", "w");
	fprintf(gnuplotPipe, "n=100;\n");
	fprintf(gnuplotPipe, "max=%f;\n", max);
	fprintf(gnuplotPipe, "min=%f;\n", min);
	fprintf(gnuplotPipe, "width=(max-min)/n;\n");
	fprintf(gnuplotPipe, "hist(x,width)=width*floor(x/width)+width/2.0;\n");
	fprintf(gnuplotPipe, "set term png;\n");
	fprintf(gnuplotPipe, "set output \"histogram.png\";\n");
	fprintf(gnuplotPipe, "set xrange [min:max];\n");
	fprintf(gnuplotPipe, "set yrange [0:];\n");
	fprintf(gnuplotPipe, "set offset graph 0.05,0.05,0.05,0.0;\n");
	fprintf(gnuplotPipe, "set xtics min,(max-min)/5,max;\n", min, max, min,
			max);
	fprintf(gnuplotPipe, "set boxwidth width*0.9;\n");
	fprintf(gnuplotPipe, "set style fill solid 0.5;\n");
	fprintf(gnuplotPipe, "set tics out nomirror;\n");
	fprintf(gnuplotPipe, "set xlabel \"x\";\n");
	fprintf(gnuplotPipe, "set ylabel \"Frequency\";\n");
	fprintf(gnuplotPipe,
			"plot '-'  u (hist($1,width)):(1.0) smooth freq w boxes lc rgb\"green\" notitle\n");

	double p;
	for (int i = 0; i < numOfSamples; i++) {
		p = getRandom(min, max);
		//cout << p << endl;
		fprintf(gnuplotPipe, "%f \n", p);
	}

	fprintf(gnuplotPipe, "e ,\n ");
	fflush(gnuplotPipe);
	fclose(gnuplotPipe);
}

UniformRNG::UniformRNG() {
	// initialize the seed
	srand(time(NULL));
}

double UniformRNG::getRandom(const double min, const double max) {
	double randValue = min
			+ static_cast<double>(rand())
					/ (static_cast<double>(RAND_MAX / (max - min)));
	return randValue;
}

MarginDiscriminatingUniformRNG::MarginDiscriminatingUniformRNG(
		double linearWidth) :
		linearWidth(linearWidth) {
	// nothing to do
}

double MarginDiscriminatingUniformRNG::getRandom(const double min,
		const double max) {
	// how much is the function "shifted" from the symmetric function?
	const double shift = (max + min) / 2;

	// shifted margins s.t. the probability function is symmetric
	const double margin = (fabs(max) + fabs(min)) / 2;

	// width of linear part at the margins
	const double linear = this->linearWidth;

	// const probability of the uniform part
	const double pMax = 1 / (2 * margin - linear);

	// m = dy / dx
	const double m = pMax / linear;

	// rejection sampling
	double x, y, p;
	do {
		x = this->uniformRng.getRandom(-margin, margin);
		y = this->uniformRng.getRandom(0.0, pMax);

		// calculate the probablity at x i.e. p(x)
		if (-margin <= x && x < (-margin + linear)) {
			// left linear part
			p = m * (x + margin);
		} else if ((-margin + linear) <= x && x <= (margin - linear)) {
			// uniform part
			p = pMax;
		} else if ((margin - linear) < x && x < margin) {
			// right linear part
			p = -m * (x - margin);
		} else {
			// zero
			p = 0.0;
		}
	} while (!(y <= p));

	// re-shift
	x = x + shift;

	return x;
}

void MarginDiscriminatingUniformRNG::setLinearWidth(double linearWidth) {
	this->linearWidth = linearWidth;
}

} /* namespace kinematic_calibration */

