/*
 * MeasurementPose.h
 *
 *  Created on: 02.03.2014
 *      Author: stefan
 */

#ifndef MEASUREMENTPOSE_H_
#define MEASUREMENTPOSE_H_


#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "KinematicChain.h"
#include "../../include/common/CalibrationContext.h"


namespace kinematic_calibration {
class CalibrationState;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {

using namespace std;

/**
 * Represents a pose for the kinematic chain to be optimized.
 */
class MeasurementPose {
public:
	/**
	 * Constructor.
	 * @param kinematicChain The kinematic model for the chain.
	 * @param jointState The joint states of the kinematic chain.
	 */
    MeasurementPose(KinematicChain kinematicChain, sensor_msgs::JointState jointState, std::string ID,
                    CalibrationOptions options = defaultOptions());

	MeasurementPose();

	/**
	 * Desctructor.
	 */
	virtual ~MeasurementPose();

	/**
	 * Predicts the end effector pose (e.g. the marker) given the model and the state.
	 * @param[in] state State to use.
	 * @param[out] pose The predicted pose.
	 */
	void predictEndEffectorPose(const CalibrationState& state,
			tf::Transform& pose);

	/**
	 * Predicts the image coordinates of the end effector, projected into the camera frame.
	 * @param[in] state State to use.
	 * @param[out] x X position within the camera projection frame.
	 * @param[out] y Y position within the camera projection frame.
	 *  TODO: Add option to observe multiple markers per configuration
	 */
	void predictImageCoordinates(const CalibrationState& state,
			double& x, double& y);

	void getPartialDerivatives(const CalibrationState& state,
			Eigen::MatrixXd& derivatives);

	MeasurementPose& operator=(const MeasurementPose& newval);

	const sensor_msgs::JointState& getJointState() const {
		return jointState;
	}


	std::string getChainName() const {
	  return kinematicChain.getName();
	}

    const string getKinematicChainName() const{
        return kinematicChain.getName();
    }

    const KinematicChain & getKinematicChain() const {
        return kinematicChain;
    }

    void resetDerivatives() {
        derivativesCalculated = false;
        derivativesX.resize(0);
        derivativesY.resize(0);
    }

    void setID(const string & ID) { id = ID;}
    std::string getID() const {return id;}

	/**
	 * Converts the pose into the pose manager string format.
	 * @param number The pose number to be used.
	 * @param stream The string stream to be appended.
	 */
	void toPoseManagerString(const int& number, stringstream& stream) const;

protected:
	void calcCameraIntrinsicsDerivatives(CalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	void calcCameraTransformDerivatives(CalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	void calcMarkerTransformDerivatives(CalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	void calcJointOffsetsDerivatives(CalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	double calculateDerivative(const double& plus, const double& minus,
			const double& h);

	static CalibrationOptions defaultOptions();

private:
    KinematicChain kinematicChain; // TODO: A pose does not need a KinematicChain. A reference to one should be sufficient (e.g. a name)
	sensor_msgs::JointState jointState;




	// vectors for the derivatives for x and y respectively
	Eigen::RowVectorXd derivativesX, derivativesY;
	bool derivativesCalculated;
    CalibrationOptions options;
    std::string id;
};

} /* namespace kinematic_calibration */

#endif /* MEASUREMENTPOSE_H_ */
