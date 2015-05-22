/*
 * CalibrationState.h
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <iostream>
#include <map>
#include <string>
#include <utility>

#include <common/KinematicChain.h>

using namespace std;

namespace kinematic_calibration {

/**
 * Represents the calibration parameters, i.e. joint offsets, camera extrinsics and intrinsics, etc.
 */
class CalibrationState {
public:
	enum TransformSource {
		ROSPARAM_URDF, TF
	};

	/**
	 * Default constructor.
	 */
    CalibrationState();

	/**
	 * Parameterized constructor.
	 * @param jointOffsets initial joint offsets
	 * @param markerTransformation initial marker transformation
	 */
    CalibrationState(map<string, double>& jointOffsets,
			map<string, tf::Transform> markerTransformations,
            tf::Transform& cameraTransformation);

    /**
     * Destructor.
     */
    virtual ~CalibrationState();

    /**
     * @brief Initializes the marker transformation.
     * @param name Name of the kinematic chain.
     * @param root Root frame name.
     * @param tip Tip frame name.
     */
    void addMarker(const string name, const string root, const string tip,
          TransformSource source = ROSPARAM_URDF);

    /**
     * @brief get the names of all known joint (offsets)
     * @return A vector with the names of all joints of which the state stores the offsets
     */
     std::vector<std::string> getJointNames() const;

     /**
      * @brief Returns a map of all joint transformations using KDL frames.
      * @return A map of all joint transformations using KDL frames.
      */
     map<string, KDL::Frame> getJointFrames() const ;

     /**
      * Current joint offsets.
      */
     map<string, double> jointOffsets;

     /**
      * Estimation for the marker (to end effector) transformations for the kinematic chains.
      */
     map<string, tf::Transform> markerTransformations;

     /**
      * Estimation for the transformation between camera and head.
      */
     tf::Transform cameraTransformation;

     /**
      * Estimation for the camera intrinsics (mono camera; K, P and D).
      */
     sensor_msgs::CameraInfo cameraInfo;

     /**
      * Estimations for the joint transformations.
      */
     map<string, tf::Transform> jointTransformations;


private:

    /** @brief overload streaming operator to allow printing the calibration state
     */
    friend ostream& operator<<(ostream& output, const CalibrationState& state);

};

} /* namespace kinematic_calibration */

#endif /* CALIBRATIONSTATE_H_ */
