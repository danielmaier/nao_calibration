/*
 *  Kinematic Calibration: A calibration framework for humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */



#ifndef ONLINECALIBRATION_H
#define ONLINECALIBRATION_H

#include <calibration/CalibrationFramework.h>
#include <pose_generation/MeasurementMsgPoseSource.h>

namespace kinematic_calibration {

/**
 * @brief An online calibration framework
 */
class OnlineCalibration : public CalibrationFramework<MeasurementMsgPoseSource> {
public:
    OnlineCalibration();

    virtual ~OnlineCalibration();

    /** @brief calls the calibration routine and publishes the resulting state */
    void calibrateAndPublish();

protected:
    /** @brief selects new robot poses using a provided strategy,
     * updates m_resultSet and optimizes m_resultSet
     *
     * Poses will be observed using m_dataCapture prior to optimization
     * @param[in] strategy the PoseSelectionStrategy
     */
    void selectAndOptimize(boost::shared_ptr<PoseSelectionStrategy> strategy);

    /** @brief observes the corresponding markers for the selected poses
     *
     * Poses that are already observed (in m_poseIDToObservation) will be ignored.
     * The poses in poseSet will be updated with the actual joint sensor measurements
     * and m_poseIDToObservation will be updated accordingly
     * @return number of new observations
     */
    int observe();

    /**
     * @brief remove poses from the set of all (selected) poses that have not been observed yet
     * @return the number of removed poses
     */
    int removeUnobservedPosesFromPoseSet();

    /**
     * @brief check if the minimum required number of poses have been selected and observed
     * @return true if minimum number is reached, false otherwise
     */
    bool minimumNumberOfPosesReached( );


    /** @name overridden initialization members */
    ///@{
    void initOptimization(); // override abstract parent method
    void initPoseSource(); // override abstrcat parent method
    ///@}


    /* member variables */
    ros::ServiceClient m_dataCapture; ///< A client to a marker observation service

};




} /* namespace kinematic_calibration */

#endif /* ONLINECALIBRATION_H */
