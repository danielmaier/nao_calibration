/*
 * DataCapture.h
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#ifndef DATACAPTURE_H_
#define DATACAPTURE_H_

#include <actionlib/client/simple_action_client.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <kinematic_calibration/measurementData.h>
#include <naoqi_msgs/BodyPoseAction.h>
#include <naoqi_msgs/BodyPoseGoal.h>
#include <naoqi_msgs/JointTrajectoryAction.h>
#include <naoqi_msgs/JointTrajectoryActionResult.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <string>
#include <vector>
#include "MarkerDetection.h"
#include "PauseManager.h"
#include <kinematic_calibration/Capture.h>
#include "../common/CalibrationContext.h"
#include "../common/MarkerContext.h"
#include <common/Observation.h>

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class DataCapture {
public:
	enum Region {
		LEFT_TOP, LEFT_BOTTOM, RIGHT_TOP, RIGHT_BOTTOM, CENTER
	};

	DataCapture(CalibrationContext& context);
	virtual ~DataCapture();

    void enableStiffnesses();
    void disableStiffnesses();
	void enableHeadStiffness();
	void disableHeadStiffness();
	void enableChainStiffness();
	void disableChainStiffness();



	void playChainPoses();

    void publishMeasurementLoop();

    bool capturePose(const std::string & poseName, double x_exp, double y_exp, measurementData & obs) ;


	void setHeadPose(double headYaw, double headPitch, bool relative = false,
			vector<string> additionalJoints = vector<string>(),
			vector<double> additionalPositions = vector<double>());

	void findMarker();
	void moveMarkerToImageRegion(Region region);
    void retrieveParamsForChain(const std::string & chainName);

protected:
    virtual const vector<string>& getJointNames() = 0;
	virtual const string getPosePrefix();

protected:

	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate;
	ros::Subscriber camerainfoSub;
	ros::Subscriber jointStateSub;
	ros::CallbackQueue jointStatesQueue;
	image_transport::Subscriber imageSub;
	image_transport::ImageTransport it;
    image_transport::Publisher captureVisPub;
	ros::Publisher measurementPub;
    boost::shared_ptr<MarkerDetection> markerDetection;
	vector<double> markerData;
    boost::shared_ptr<MarkerContext> markerContext;
	string markerType;
	string chainName;
	PauseManager pauseManager;
	bool markerFound;
	bool receivedJointStates;
	bool receivedImage;
    actionlib::SimpleActionClient<naoqi_msgs::JointTrajectoryAction> stiffnessClient;
    actionlib::SimpleActionClient<naoqi_msgs::JointTrajectoryAction> trajectoryClient;
    actionlib::SimpleActionClient<naoqi_msgs::BodyPoseAction> bodyPoseClient;
	vector<string> headJointNames;
	image_geometry::PinholeCameraModel cameraModel;
	tf::TransformListener transformListener;
	string cameraFrame;
	sensor_msgs::JointState jointState;
	ros::Time curTime;
	string imageTopic;
	CalibrationContext& context;
	sensor_msgs::Image image;
	string currentPoseName;
	void setStiffness(const vector<string>& jointNames, double stiffness);
	void enableStiffness(const vector<string>& jointNames);
	void disableStiffness(const vector<string>& jointNames);
	void disableStiffnessSlowly(const vector<string>& jointNames);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
    void updateMarker();
	void updateMarkerOnce();
	void updateMarkerRobust();
	void updateJointStates();
	void updateJointStatesOnce();
	void updateJointStatesRobust();
    void publishMeasurement(const measurementData & data);
    void publishMeasurement(const std::string & poseName);
    bool getMeasurementData(const std::string & poseName, measurementData & data);
	vector<double> generateRandomPositions(const vector<string>& joints);
    double headYawMin, headYawMax, headYawStep;
    double headPitchMin, headPitchMax, headPitchStep;

};

class GeneralDataCapture : public DataCapture {
public:
    GeneralDataCapture(CalibrationContext & context);
    ~GeneralDataCapture() ;

    bool observe( Capture::Request & req, Capture::Response & resp);
    void setJointNames(const vector<string> &jointNames);

protected:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    ros::ServiceServer m_observe;
    virtual const vector<string> & getJointNames();
    std::vector<std::string> jointNames;
};

class LeftArmDataCapture: public DataCapture {
public:
	LeftArmDataCapture(CalibrationContext& context);
	virtual ~LeftArmDataCapture();

protected:
	virtual const vector<string>& getJointNames();

private:
	vector<string> jointNames;
};

class RightArmDataCapture: public DataCapture {
public:
	RightArmDataCapture(CalibrationContext& context);
	virtual ~RightArmDataCapture();

protected:
	virtual const vector<string>& getJointNames();

private:
	vector<string> jointNames;
};

class LegDataCapture: public DataCapture {
public:
	LegDataCapture(CalibrationContext& context);
	virtual ~LegDataCapture();

protected:
	virtual const vector<string>& getJointNames();
	vector<string> jointNamesLeft, jointNamesRight, jointNames;
};

class LeftLegDataCapture: public LegDataCapture {
public:
	LeftLegDataCapture(CalibrationContext& context);
	virtual ~LeftLegDataCapture();

protected:
	virtual const vector<string>& getJointNames();
};

class RightLegDataCapture: public LegDataCapture {
public:
	RightLegDataCapture(CalibrationContext& context);
	virtual ~RightLegDataCapture();

protected:
	virtual const vector<string>& getJointNames();
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
