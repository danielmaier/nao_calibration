/*
 * MeasurementPose.cpp
 *
 *  Created on: 02.03.2014
 *      Author: stefan
 */

#include "../../include/common/MeasurementPose.h"

#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <map>
#include <utility>
#include <vector>
//#include "../../include/common/helpers.h"
#include <gsl/gsl_deriv.h>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include <optimization/CalibrationState.h>
#include "../../include/optimization/TransformationVertex.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include <common/helpers.h>
#include <iomanip>

//#define DEBUG_JACOBIAN

namespace kinematic_calibration {



MeasurementPose::MeasurementPose(KinematicChain kinematicChain,
        sensor_msgs::JointState jointState, std::string ID, CalibrationOptions options) :
        kinematicChain(kinematicChain), jointState(jointState), derivativesCalculated(false), options(options), id(ID) {

}

MeasurementPose::MeasurementPose() :
		kinematicChain(KinematicChain()), jointState(sensor_msgs::JointState()), derivativesCalculated(
				false), options(defaultOptions()) {
}

MeasurementPose& MeasurementPose::operator=(const MeasurementPose& newval) {
	kinematicChain = newval.kinematicChain;
	jointState = newval.jointState;
    id = newval.id;
	return *this;
}

MeasurementPose::~MeasurementPose() {
	// Nothing to do.
}

void MeasurementPose::predictEndEffectorPose(
		const CalibrationState& state, tf::Transform& cameraToMarker) {
	// initialize the three parts of the chain transformation:
	tf::Transform endEffectorToMarker, headToEndEffector, cameraToHead;
	// check whether a marker transformation is given and get it
	string chainName = kinematicChain.getName();
	//cout << "looking form marker transform in chain " << chainName  ;
	map<string, tf::Transform>::const_iterator it = state.markerTransformations.find(chainName);
	if (it != state.markerTransformations.end()) {
		endEffectorToMarker = it->second;
	//	cout << " got it" << endl;
	} else {
		endEffectorToMarker.setIdentity();
	//	cout << " couldn't get it" << endl;
		ROS_WARN("Could not find MarkerTransform for %s", it->first.c_str());
	}

	// get the camera transformation
    cameraToHead = state.cameraTransformation;

	// get joint positions and joint offsets
	map<string, double> jointPositions, jointOffsets;
    for (unsigned int i = 0; i < jointState.name.size(); i++) {
		jointPositions[jointState.name[i]] = jointState.position[i];
	}
	jointOffsets = state.jointOffsets;

	// get the transformation from camera to marker
	if (options.calibrateJoint6D) {
		// optimization of joint 6D offsets
        KinematicChain kc = this->kinematicChain.withTransformations(state.jointTransformations);
		kc.getRootToTip(jointPositions, jointOffsets, headToEndEffector);
	} else {
		// only joint angle offsets
        kinematicChain.getRootToTip(jointPositions, jointOffsets, headToEndEffector);
	}
	cameraToMarker = endEffectorToMarker * headToEndEffector * cameraToHead;
}

void MeasurementPose::predictImageCoordinates(
		const CalibrationState& state, double& x, double& y) {
	// initialize the camera model from state
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(state.cameraInfo);
	FrameImageConverter fic(cameraModel);

	// calculate the end effector pose
	tf::Transform cameraToMarker;
	predictEndEffectorPose(state, cameraToMarker);

	// project the point into image coordinates
	fic.project(cameraToMarker.inverse(), x, y);
}

void MeasurementPose::getPartialDerivatives(
		const CalibrationState& state, Eigen::MatrixXd& derivatives) {
	if (!derivativesCalculated) {
		// calculate the derivatives

		// initialize delta value
		double h = 1e-9; //1e-9;
		vector<double> partialDerivatesVectorX;
		vector<double> partialDerivatesVectorY;

		// TODO: parameterize which derivatives should be calculated

		// camera parameters: fx, fy, cx, cy, d[0-4]
		if (options.calibrateCameraIntrinsics)
			calcCameraIntrinsicsDerivatives(state, h, partialDerivatesVectorX,
					partialDerivatesVectorY);

		// camera transform parameters: tx, ty, tz, rr, rp, ry
		if (options.calibrateCameraTransform)
			calcCameraTransformDerivatives(state, h, partialDerivatesVectorX,
					partialDerivatesVectorY);

		// joint offsets: from root (head) to tip
		if (options.calibrateJointOffsets)
			calcJointOffsetsDerivatives(state, h, partialDerivatesVectorX,
					partialDerivatesVectorY);

		// marker transform parameters: tx, ty, tz, rr, rp, ry
		if (options.calibrateMarkerTransform)
			calcMarkerTransformDerivatives(state, h, partialDerivatesVectorX,
					partialDerivatesVectorY);

                // convert to Eigen vector
                derivativesX.resize(1,
				derivativesX.size() + partialDerivatesVectorX.size());
		derivativesY.resize(1,
				derivativesY.size() + partialDerivatesVectorY.size());
        for (unsigned int i = 0; i < partialDerivatesVectorX.size(); i++) {
			derivativesX.col(i) << partialDerivatesVectorX[i];
			derivativesY.col(i) << partialDerivatesVectorY[i];
		}
		derivativesCalculated = true;
		//cout << derivativesX << endl;
		//cout << derivativesY << endl;
	}

	// write two rows to the jacobian matrix
    derivatives.resize(derivatives.rows() + 2, derivativesX.cols());
	derivatives.row(derivatives.rows() - 2) << derivativesX;
	derivatives.row(derivatives.rows() - 1) << derivativesY;

}

void MeasurementPose::calcCameraIntrinsicsDerivatives(
		CalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {

	// camera parameters: fx, fy, cx, cy, d[0-4]
	CameraIntrinsicsVertex civ(state.cameraInfo);
	for (int index = 0; index < civ.dimension(); index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

		// initialize the delta vector
		double delta[civ.dimension()];
		for (int j = 0; j < civ.dimension(); j++)
			delta[j] = 0.0;

		// update delta (plus)
		civ.setToOrigin();
		delta[index] = h;
		civ.oplus(delta);
		state.cameraInfo = civ.estimate();

		// update error
		this->predictImageCoordinates(state, xPlus, yPlus);

		// update delta (minus)
		civ.setToOrigin();
		delta[index] = -h;
		civ.oplus(delta);
		state.cameraInfo = civ.estimate();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
	}
	// reset state
	civ.setToOrigin();
	state.cameraInfo = civ.estimate();
}

void MeasurementPose::calcCameraTransformDerivatives(
		CalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
    tf::Transform cameraTransform = state.cameraTransformation;

	// camera transform parameters: tx, ty, tz, rr, rp, ry
	TransformationVertex vertex;
	for (int index = 0; index < vertex.dimension(); index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

		// initialize the delta vector
		double delta[vertex.dimension()];
		for (int j = 0; j < vertex.dimension(); j++)
			delta[j] = 0.0;

		// update delta (plus)
		vertex.setEstimate(cameraTransform);
		delta[index] = h;
		vertex.oplus(delta);
        state.cameraTransformation = vertex.estimate();

		// update error
		this->predictImageCoordinates(state, xPlus, yPlus);

		// update delta (minus)
		vertex.setEstimate(cameraTransform);
		delta[index] = -h;
		vertex.oplus(delta);
        state.cameraTransformation = vertex.estimate();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
	}
	// reset state
    state.cameraTransformation = cameraTransform;
}

void MeasurementPose::calcMarkerTransformDerivatives(
		CalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
#ifdef DEBUG_JACOBIAN
        cout << "Computing marker transforms";
#endif
        for (map<string, tf::Transform>::iterator it = state.markerTransformations.begin();
            it != state.markerTransformations.end(); ++it)
        {
          tf::Transform markerTransform = it->second;
#ifdef DEBUG_JACOBIAN
          cout << "Computing Derivatives for Marker " << it->first  << " with transform " << endl;
          cout << markerTransform << endl;
#endif
          //    state.markerTransformations[kinematicChain.getName()];

          // marker transform parameters: tx, ty, tz, rr, rp, ry
          TransformationVertex vertex;
          // TODO: calcMarkerTransformDerivative seems to ignore the rotational components..
          for (int index = 0; index < vertex.dimension() - 3; index++) {
            // derivations around the current point
            double xMinus, yMinus, xPlus, yPlus;

            // initialize the delta vector
            double delta[vertex.dimension()];
            for (int j = 0; j < vertex.dimension(); j++)
              delta[j] = 0.0;

            // update delta (plus)
            vertex.setEstimate(markerTransform);
            delta[index] = h;
            vertex.oplus(delta);
            //state.markerTransformations[kinematicChain.getName()] =
            it->second =
                vertex.estimate();
            //cout << "State "

            // update error
            this->predictImageCoordinates(state, xPlus, yPlus);

            // update delta (minus)
            vertex.setEstimate(markerTransform);
            delta[index] = -h;
            vertex.oplus(delta);
            it->second = vertex.estimate();

            // update error
            this->predictImageCoordinates(state, xMinus, yMinus);

            // calculate the derivative
            double derivativeX = calculateDerivative(xPlus, xMinus, h);
            derivativesX.push_back(derivativeX);
            double derivativeY = calculateDerivative(yPlus, yMinus, h);
            derivativesY.push_back(derivativeY);
#ifdef DEBUG_JACOBIAN
            cout << "Derivatives are " << derivativeX << "," << derivativeY << endl;
#endif
          }
          // reset state
          //state.markerTransformations[kinematicChain.getName()] = markerTransform;
          it->second = markerTransform;
        }
}

void MeasurementPose::calcJointOffsetsDerivatives(
		CalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
	map<string, double> jointOffsets = state.jointOffsets;

	// joint offsets: from root (head) to tip
	vector<string> jointNames = state.getJointNames();
	//kinematicChain.getJointNames(jointNames);
	JointOffsetVertex vertex(jointNames);
#ifdef DEBUG_JACOBIAN
	int precision = cout.precision();
	cout.precision(11);
#endif
	int dimension = vertex.estimateDimension();
	for (int index = 0; index < dimension ; index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

		// initialize the delta vector
		double delta[dimension];
		for (int j = 0; j < dimension; j++)
			delta[j] = 0.0;

#ifdef DEBUG_JACOBIAN
		this->predictImageCoordinates(state, xPlus, yPlus);
		cout << "calculate partial derivatives for " << jointNames[index] << endl;
		cout << "initial state\n" << state << endl;
		cout << "initial eef img coordinate: [" << xPlus << ", " << yPlus << "]" <<endl;
#endif
		// update delta (plus)
		vertex.setToOrigin();
		delta[index] = h;
		vertex.oplus(delta);
		state.jointOffsets = vertex.estimate();

		// update error
		this->predictImageCoordinates(state, xPlus, yPlus);
#ifdef DEBUG_JACOBIAN
		cout << "state+delta\n" << state << endl;
		cout << "state+delta eef img coordinate: [" << xPlus << ", " << yPlus << "]" << endl;
#endif
		// update delta (minus)
		vertex.setToOrigin();
		delta[index] = -h;
		vertex.oplus(delta);
		state.jointOffsets = vertex.estimate();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);
#ifdef DEBUG_JACOBIAN
		cout << "state-delta\n" << state << endl;
		cout << "state-delta eef img coordinate: [" << xMinus << ", " << yMinus << "]" << endl;
#endif

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
#ifdef DEBUG_JACOBIAN
		cout << "derivative is " << derivativeX << "," << derivativeY << endl;
#endif
	}
#ifdef DEBUG_JACOBIAN
	cout.precision(precision);
#endif
	// reset state
	state.jointOffsets = jointOffsets;
}

double MeasurementPose::calculateDerivative(const double& plus,
		const double& minus, const double& h) {
	// calculate the derivative
	double derivative = (plus - minus) / (2 * h);
//	fexcept_t flagp = 0;

//	if (fetestexcept(FE_ALL_EXCEPT) || plus < 1e-20 || minus < 1e-20
//			|| isnan(derivative) || derivative < 1e-20 || derivative > 1e5) {
//		cout << "plus: " << plus << " minus: " << minus << " derivative: " << derivative << endl;
//		// TODO: How to handle this correctly?!
//		derivative = 0;
//	}

	//if(derivative < 1e-3)
	//	return 0.0;

	//cout << std::setprecision (15) <<  "(+) " << plus << ", (-) " << minus << ", " << "(d) " << derivative << "; ";
	return derivative;
}

void MeasurementPose::toPoseManagerString(const int& number,
		stringstream& stream) const {
	vector<string> jointNames;
	this->kinematicChain.getJointNames(jointNames);

	// print name
	stringstream ss;
	ss << this->kinematicChain.getName();
	ss << std::setfill('0') << std::setw(3);
	ss << number;
	string poseName = ss.str();
	stream << poseName << ":\n";

	// print joint names
	stream << "  joint_names: [";
    unsigned int numOfJoints = 0;
    for (unsigned int i = 0; i < this->jointState.name.size(); i++) {
		const string jointName = this->jointState.name[i];
		if (find(jointNames.begin(), jointNames.end(), jointName)
				!= jointNames.end()) {
			numOfJoints++;
			// print the joint name
			stream << "\"" << jointName << "\"";
			if (numOfJoints != jointNames.size()) {
				// print comma if not the last
				stream << ", ";
			}
		}
	}
	stream << "]\n";

	// print time from start
	stream << "  time_from_start: 1.0\n";

	// print positions
	stream << "  positions: [";
	numOfJoints = 0;
    for (unsigned int i = 0; i < this->jointState.name.size(); i++) {
		const string jointName = this->jointState.name[i];
		if (find(jointNames.begin(), jointNames.end(), jointName)
				!= jointNames.end()) {
			const double position = this->jointState.position[i];
			numOfJoints++;
			// print the position
			stream << position;
			if (numOfJoints != jointNames.size()) {
				// print comma if not the last
				stream << ", ";
			}
		}
	}
	stream << "]\n";
	stream << endl;
}

CalibrationOptions MeasurementPose::defaultOptions() {
	CalibrationOptions options;
	options.calibrateJointOffsets = true;
	options.calibrateCameraTransform = true;
	options.calibrateCameraIntrinsics = true;
	options.calibrateMarkerTransform = true;
	options.calibrateJoint6D = false;
	return options;
}

} /* namespace kinematic_calibration */

