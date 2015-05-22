/*
 * G2oJointOffsetOptimization.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef G2OJOINTOFFSETOPTIMIZATION_H_
#define G2OJOINTOFFSETOPTIMIZATION_H_

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/batch_stats.h>
#include <kinematic_calibration/measurementData.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/optimization/JointOffsetOptimization.h"
#include <optimization/CalibrationState.h>
#include "../../include/optimization/JointOffsetVertex.h"

using namespace g2o;

namespace kinematic_calibration {

class ConstrainJointOffsetsEdge : public BaseUnaryEdge<1, Eigen::Vector2d, JointOffsetVertex>
{
public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Constructor TODO: Add option to add nominal values != 0
    ConstrainJointOffsetsEdge() {}


    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    // Error function
    //void setError();

    void computeError(){
        //ROS_INFO("Computing error for ConstrainJointOffsetsEdge");
        JointOffsetVertex* jointOffsetVertex =
                //static_cast<JointOffsetVertex*>(this->_vertices[0]);
                static_cast<JointOffsetVertex*>(vertex(0));
        map<string, double> jointOffsets = jointOffsetVertex->estimate();
        double errorSum = 0.0;
        for (map<string, double>::const_iterator it = jointOffsets.begin();
             it != jointOffsets.end(); ++it)
        {
            errorSum += fabs(it->second);
        }

        _error(0) = errorSum;
    }


};

/**
 * Class that optimizes the joint offsets using g2o.
 */
class G2oJointOffsetOptimization: public JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	G2oJointOffsetOptimization(CalibrationContext& context,
			vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter,
			CalibrationState initialState =
					CalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~G2oJointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
     * returns the number of outer iterations
	 */
    int optimize(CalibrationState& optimizedState);

	/**
	 * Returns the batch statistics of the optimization.
	 * return The statistics of the optimization.
	 */
	BatchStatisticsContainer getStatistics() const;

	/**
	 * Sets whether the intermediate calibration states should be saved.
	 * @param saveIntermediateStates Determines whether the intermediate calibration states should be saved.
	 */
	void setSaveIntermediateStates(bool saveIntermediateStates);

	void getIntermediateStates(vector<CalibrationState>& intermediateStates) const;

    BatchStatisticsContainer statistics;

protected:
	void plotStatistics(const BatchStatisticsContainer& statistics) const;

	bool saveIntermediateStates;

	vector<CalibrationState> intermediateStates;
};

}
/* namespace kinematic_calibration */
#endif /* G2OJOINTOFFSETOPTIMIZATION_H_ */
