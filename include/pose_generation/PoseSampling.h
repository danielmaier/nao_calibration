/*
 * PoseSampling.h
 *
 *  Created on: 16.04.2014
 *      Author: stefan
 */

#ifndef POSESAMPLING_H_
#define POSESAMPLING_H_

#include <boost/shared_ptr.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/CameraInfo.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <map>
#include <string>
#include <vector>

#include <hrl_kinematics/TestStability.h>

#include <optimization/CalibrationState.h>
#include "../common/KinematicChain.h"
#include "../common/MeasurementPose.h"
#include "../common/ModelLoader.h"

namespace kinematic_calibration {
class KinematicChain;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {


// forward declarations (complete declarations at the end of the file)
class RandomNumberGenerator;
class UniformRNG;
class MarginDiscriminatingUniformRNG;

/**
 * Class for sampling poses.
 */
class PoseSampling {
public:
	/**
	 * Constructor.
	 */
	PoseSampling();

	/**
	 * Desctructor.
	 */
	virtual ~PoseSampling();

	/**
	 * Samples poses and returns as soon as the number of requested poses is reached.
	 * @param[in] numOfPoses The number of requested poses.
	 * @param[out] poses The collection of sampled poses.
	 */
	void getPoses(const int& numOfPoses, vector<MeasurementPose>& poses);

	/**
	 * Returns whether we are in debug mode.
	 * @return Whether we are in debug mode.
	 */
	bool isDebug() const {
		return debug;
	}

	/**
	 * Enables or disables the debug mode.
	 * If set to 'true', prints out some info
	 * and publishes the first valid pose (joint states, tf, urdf, moveit state)
	 * @param debugMode
	 */
	void setDebug(bool debugMode) {
		this->debug = debugMode;
	}

	/**
	 * Initializes all necessary components.
	 * Is already called by the constructor.
	 * Manual call is only necessary for re-initialization
	 */
	virtual void initialize();

protected:
	/**
	 * Initializes the kinematic chain for which the poses should be selected.
	 */
	virtual void initializeKinematicChain();

	/**
	 * Initializes the initial state.
	 */
	virtual void initializeState();

	/**
	 * Initializes the camera model.
	 */
	virtual void initializeCamera();

	/**
	 * Initializes the joint limits.
	 */
	virtual void initializeJointLimits();

	/**
	 * Initializes the URDF from ROS parameter.
	 */
	virtual void initializeUrdf();

	/**
	 * Initializes the SRDF from ROS parameter and modifies it.
	 * @param robotName The robot name (replaces the robot name from the SRDF at the ROS parameter server).
	 * @param joints The joint names of the current kinematic chain.
	 * @param links The link names of the current kinematic chain.
	 */
	virtual void initializeSrdf(const string& robotName,
			const vector<string>& joints, const vector<string>& links);

	/**
	 * Load the initial pose from the ROS parameter if given.
	 */
	virtual void initializeInitialPose();

	/**
	 * Initializes the IK request.
	 */
	virtual void initializeEndEffectorState();

	/**
	 * Camera info message callback.
	 * @param[in] msg Camera info message,
	 */
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

	/**
	 * Publishes the joint state. Sets the current timestamp before.
	 * @param msg The joint state to be published.
	 */
	void publishJointState(sensor_msgs::JointState& msg) const;

	/**
	 * Checks whether the initial pose and the given joint state is stable.
	 * @param msg The joint state of the pose to check.
	 * @return True if the pose is stable, false otherwise.
	 */
	bool isPoseStable(const sensor_msgs::JointState& msg) const;

	/**
	 * Pointer to the initial/current calibration state.
	 */
    boost::shared_ptr<CalibrationState> initialState;

	/**
	 * Pointer to the kinematic chain currently used.
	 */
	boost::shared_ptr<KinematicChain> kinematicChainPtr;

	/**
	 * Pointer to the stability test instance.
	 */
	boost::shared_ptr<hrl_kinematics::TestStability> testStabilityPtr;

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

	/**
	 * Private NodeHandle instance;
	 */
	NodeHandle nhPrivate;

	/**
	 * Subscriber for camera info messages.
	 */
	Subscriber cameraInfoSubscriber;

	/**
	 * Publisher for joint states.
	 */
	Publisher jointStatePub;

	/**
	 * Source for loading the robot model.
	 */
	ModelLoader modelLoader;

	/**
	 * The urdf robot model.
	 */
	urdf::Model robotModel;

	/**
	 * Camera model.
	 */
	image_geometry::PinholeCameraModel cameraModel;

	/**
	 * The KDL tree of the robot model.
	 */
	KDL::Tree kdlTree;

	/**
	 * Pointer to the URDF model.
	 */
    boost::shared_ptr<urdf::Model> urdfModelPtr;

	/**
	 * Pointer to the SRDF model.
	 */
    boost::shared_ptr<const srdf::Model> srdfModelPtr;

	/**
	 * String containing the SRDF model.
	 */
	string srdfString;

	/**
	 * Flag that indicates whether a SRDF model is available.
	 * If true, collision checking is enabled for the whole chain,
	 * expecting that the loaded SRDF model contains information about
	 * the collisions that can be ignored.
	 */
	bool srdfAvailable;

	/**
	 * Lower joint limits.
	 */
	map<string, double> lowerLimits;

	/**
	 * Upper joint limits.
	 */
	map<string, double> upperLimits;

	/**
	 * The joint names.
	 */
	vector<string> jointNames;

	/**
	 * Name of the camera frame.
	 */
	string cameraFrame;

	/**
	 * Name of the torso frame.
	 */
	string torsoFrame;

	/**
	 * Define a rectangle within which the predicted
	 * marker position of the sampled poses should be.
	 */
	double xMin, xMax, yMin, yMax;

	/**
	 * Radius of the "view cylinder".
	 */
	double viewCylinderRadius;

	/**
	 * Flag that indicates whether an initial pose is available.
	 */
	bool initialPoseAvailable;

	/**
	 * Initial pose. If given:
	 * - initialPoseAvailable = true
	 * - stability test can be done
	 * - poses can be sampled which preserve the end effector position
	 */
	sensor_msgs::JointState initialPose;

	/**
	 * Flag that indicates whether the stability of the sampled pose should be tested.
	 * Requires that an initial pose is given.
	 */
	bool testPoseStability;

	/**
	 * Flag that indicates whether the poses should be sampled s.t. the end effector is fixed.
	 * Requires that an initial pose is given.
	 */
	bool keepEndEffectorPose;

	/**
	 * The minimum distance from the joint limits.
	 */
	double jointLimitsDistance;

	/**
	 * Width of the joint limits range within which the probability
     * distribution is linear increasing / decreasing.
	 */
	double jointLimitsLinearWidth;

	/**
	 * State of the end effector (i.e. torso -> tip)
	 */
	tf::Transform endEffectorState;

	/**
	 * Pointer to a random number generator.
	 */
	boost::shared_ptr<RandomNumberGenerator> rng;

	/**
	 * Flag which indicates whether we are in debug mode.
	 */
	bool debug;
};

/**
 * Abstract base class for generating random numbers in the specified range.
 */
class RandomNumberGenerator {
public:
	/// Constructor.
	RandomNumberGenerator() {
	}

	/// Destructor.
	virtual ~RandomNumberGenerator() {
	}

	/**
	 * Strategy method which returns a random number within the specified range.
	 * @param[in] min Specifies the lower limit of the range (including).
	 * @param[in] max Specifies the upper limit of the range (including)
	 * @return A random number within the specified range.
	 */
	virtual double getRandom(const double min, const double max) = 0;

	/**
	 * Plots the distribution using the specified number of samples.
	 * @param[in] min Specifies the lower limit of the range (including).
	 * @param[in] max Specifies the upper limit of the range (including)
	 * @param[in] numOfSamples Number of samples to be used for plotting.
	 */
	void plot(const double min, const double max, const int numOfSamples);
};

/**
 * Uniform random number generator.
 * Uses the rand() function from the stdlib.
 */
class UniformRNG: public RandomNumberGenerator {
public:
	/// Constructor.
	UniformRNG();

	/// Destructor.
	virtual ~UniformRNG() {
	}
	virtual double getRandom(const double min, const double max);
};

/**
 * Samples a random number from a distribution which behaves linear
 * at the margins and uniform in the "middle" range.
 */
class MarginDiscriminatingUniformRNG: public RandomNumberGenerator {
public:
	/// Constructor.
	MarginDiscriminatingUniformRNG(double linearWidth = 0.3);

	/// Destructor.
	virtual ~MarginDiscriminatingUniformRNG() {
	}
	virtual double getRandom(const double min, const double max);

	/**
	 * Sets the width of the linear part.
	 * @param[in] linearWidth The width of the linear part.
	 */
	void setLinearWidth(double linearWidth);

protected:
	/**
	 * The underlying uniform RNG.
	 */
	UniformRNG uniformRng;

private:
	/**
	 * Width of the linear parts at the margins.
	 */
	double linearWidth;
};

} /* namespace kinematic_calibration */

#endif /* POSESAMPLING_H_ */
