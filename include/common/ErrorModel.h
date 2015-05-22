/*
 * ErrorModel.h
 *
 *  Created on: 24.02.2014
 *      Author: stefan
 */

#ifndef ERRORMODEL_H_
#define ERRORMODEL_H_

#include <eigen3/Eigen/Dense>
#include <kinematic_calibration/measurementData.h>
#include <vector>

namespace kinematic_calibration {
class CalibrationState;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {
class KinematicChain;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {

using namespace std;

/**
 * Error model for a kinematic chain.
 */
class ErrorModel {
public:

	/**
	 * Constructor.
	 * @param kinematicChain
	 */
	ErrorModel(KinematicChain& kinematicChain);
	virtual ~ErrorModel();

	virtual void getImageCoordinates(const CalibrationState& state,
			const measurementData& measurement, double& x, double& y);

	virtual void getError(const CalibrationState& state,
			const measurementData& measurement, vector<double>& delta) = 0;

	virtual void getSquaredError(const CalibrationState& state,
			const measurementData& measurement, vector<double>& error) = 0;

protected:
	KinematicChain& kinematicChain;

	void appendVector(vector<double>& v1, const vector<double>& v2);
};

/**
 * Error model for optimizing one point of a marker (e.g. chessboard).
 */
class SinglePointErrorModel: public ErrorModel {
public:

	SinglePointErrorModel(KinematicChain& kinematicChain);
	virtual ~SinglePointErrorModel();

	virtual void getError(const CalibrationState& state,
			const measurementData& measurement, vector<double>& delta);

	virtual void getSquaredError(const CalibrationState& state,
			const measurementData& measurement, vector<double>& error);

};

/**
 * Error model for optimizing a circle marker.
 */
class CircleErrorModel: public ErrorModel {
public:

	CircleErrorModel(KinematicChain& kinematicChain);
	virtual ~CircleErrorModel();

	virtual void getError(const CalibrationState& state,
			const measurementData& measurement, vector<double>& delta);

	virtual void getSquaredError(const CalibrationState& state,
			const measurementData& measurement, vector<double>& error);

};

} /* namespace kinematic_calibration */

#endif /* ERRORMODEL_H_ */
