/*
 * ObservabilityIndex.cpp
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/ObservabilityIndex.h"

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <cmath>

namespace kinematic_calibration {

using namespace Eigen;
using namespace std;

ObservabilityIndex::ObservabilityIndex() : scaleJacobian(true) {

}

ObservabilityIndex::~ObservabilityIndex() {

}

VectorXd ObservabilityIndex::getEigenValues(
                const MeasurementPoseSet& poseSet) {
        MatrixXd jacobian;
        poseSet.getJacobian(jacobian);

        cout << "computing EigenValues" << endl;

        // scaling
        if(scaleJacobian) {
                MatrixXd scaling;
                scaling.resize(jacobian.cols(),jacobian.cols());
                for(int col = 0; col < jacobian.cols(); col++) {
                        for(int row = 0; row < jacobian.cols(); row++) {
                                if(row == col) {
                                        double norm = jacobian.col(col).norm();
                                        if(norm < 1e-6 )
                                                scaling(row, col) = 1.0;
                                        else
                                                scaling(row, col) = 1/norm;
                                } else {
                                        scaling(row, col) = 0.0;
                                }
                        }
                }
                jacobian = jacobian * scaling;
        }

        cout << "Jacobian is " << jacobian << endl;

        MatrixXd cov = (jacobian.transpose() * jacobian).inverse();
        cout << "Covariance is " << cov << endl;


        EigenSolver<MatrixXd> es(cov, false);
        VectorXd ev = es.eigenvalues().real(); //.head(nmin);
        cout << "Eigenvalues are " << ev << endl;


        return ev;
}
VectorXd ObservabilityIndex::getSingularValues(
		const MeasurementPoseSet& poseSet) {
	MatrixXd jacobian;
	poseSet.getJacobian(jacobian);

	// scaling
	if(scaleJacobian) {
		MatrixXd scaling;
		scaling.resize(jacobian.cols(),jacobian.cols());
		for(int col = 0; col < jacobian.cols(); col++) {
			for(int row = 0; row < jacobian.cols(); row++) {
				if(row == col) {
					double norm = jacobian.col(col).norm();
					if(norm < 1e-6 )
						scaling(row, col) = 1.0;
					else
						scaling(row, col) = 1/norm;
				} else {
					scaling(row, col) = 0.0;
				}
			}
		}
		jacobian = jacobian * scaling;
	}

	//cout << jacobian << endl;

	JacobiSVD<MatrixXd> svd(jacobian);

	//cout << jacobian << endl << endl;
	//cout << svd.singularValues() << endl << endl;
	//assert(false);
	return svd.singularValues();
}


void SumSingularValuesIndex::calculateIndex(const MeasurementPoseSet& poseSet,
        double& index) {
    const VectorXd singularValues = getSingularValues(poseSet);
    //double prod = singularValues.prod();
    double sum = 0.0;
    for(int i = 0; i < singularValues.rows()  ; i++)
    {
      sum += (1.0/singularValues[i]);
    }
    index = 1.0/sum;
}



void ProductSingularValuesIndex::calculateIndex(const MeasurementPoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	//double prod = singularValues.prod();
	double prod = 1.0;
	for(int i = 0; i < singularValues.rows()  ; i++)
	{
	  prod*=singularValues[i];
	}
	double m = poseSet.getNumberOfPoses();
	// TODO: Shouldn't this be 1/L, where L=singularValues.rows(), instead of 1/m?
	index = pow(prod, 1.0 / m) / sqrt(m);
}

void InverseConditionNumberIndex::calculateIndex(const MeasurementPoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1] / singularValues[0];
}

void MinimumSingularValueIndex::calculateIndex(const MeasurementPoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1];
}

void NoiseAmplificationIndex::calculateIndex(const MeasurementPoseSet& poseSet,
		double& index) {
   //TODO: Determine rank of the jacobian to find L, as is suggested in Nahvi & Hollerbach
	const VectorXd singularValues = getSingularValues(poseSet);
	bool warn = false;
	for(int i =0; i < singularValues.rows(); ++i){
	    if (singularValues[i] < 1e-9)
	    {
	      warn = true;
	      ROS_WARN("Singular Value %d is very small : %f", i, singularValues[i]);
	    }
	}
	if(warn)
	  cout << "singular Values are " << singularValues << endl;

	//double sMin = singularValues[singularValues.rows() - 1];
	int sMinRow = singularValues.rows() - 1 - 0; // hier kann man den NAI zu L-1 / L-2 aendern
	double sMin = singularValues[sMinRow > 0 ? sMinRow : 0];
	double sMax = singularValues[0];
	index = sMin * sMin / sMax;

}

void DOptimalityIndex::calculateIndex(const MeasurementPoseSet& poseSet,
                double& index) {
// TODO: This does not work right now
        const VectorXd ev = getEigenValues(poseSet);
        double prod = ev.prod();
        double m = poseSet.getNumberOfPoses();
        index = pow(prod, 1.0/m);

}

} /* namespace kinematic_calibration */

