/*
 *  Kinematic Calibration: A calibration framework for humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */



#ifndef CALIBRATIONFRAMEWORK_H
#define CALIBRATIONFRAMEWORK_H

#include <map>
#include <string>
#include <vector>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include <pose_generation/ObservabilityIndex.h>
#include <pose_generation/PoseSource.h>
#include <optimization/OptimizationInterface.h>
#include <optimization/CalibrationState.h>
#include <data_capturing/DataCapture.h>
#include <pose_generation/PoseSelectionStrategy.h>
#include <common/KinematicChain.h>
#include <common/ModelLoader.h>

namespace kinematic_calibration {


/** @brief An abstract calibration framework \n
 *  Abstract because the pose source type (i.e. how the robot configurations are obtained) is a template
 *  and depending on the pose source, some of the functionality needs to be changed.
 *  One goal for this approach is to share as much code as possible between the actual online calibration and the evaluation with pre-recorded data.
 */

template <typename PoseSourceType >
class CalibrationFramework {
public:
  typedef boost::shared_ptr<KinematicChain> KinematicChainType; // For convenience

public:
    CalibrationFramework();
    // abstract class
    virtual ~CalibrationFramework() = 0;

    /**
     *  @brief initializer function needs to be called by the user after construction.
     *  This avoids calling virtual functions in constructor
     */
    void initialize();

    /** @brief main user interface function: starts selecting poses and optimizing
     * @return the calibrated state of the robot kinematics and camera
     */
    CalibrationState calibrate();

protected:


    /**
     * @brief select configurations prior to calling the optimizer
     * @param[in] strategy a strategy how to select new poses
     */
    virtual void selectAndOptimize(boost::shared_ptr<PoseSelectionStrategy> strategy) = 0;

    /** @brief call the optimizer (automatically done by selectAndOptimize) with the currently selected poses
     *  @return true if a new state could be obtained via optimization
     */
    bool optimize();



    /**
     * @brief observe the currently selected set of poses
     * This is an abstract function that needs to be impemented in the derived class and depends heavily on the choice of pose source
     * @return number of new observations
     */
    virtual int observe() = 0;



    /** @name Initialization functions
     */
    ///@{
    virtual void initializeState();
    virtual void initializeCamera();
    virtual void initializeCameraFromFile();
    virtual void initializeKinematicChain();
    /** needs to be replaced in derived class as it depends on the actual pose source */
    virtual void initPoseSource() = 0;
    virtual void initPoseSet();
    virtual void initPoseSelectionStrategy();
    /** needs to be replaced in derived class as it depends on the actually used optimization interfaced */
    virtual void initOptimization() = 0;
    ///@}


    /** @brief set the camera model for the initial state */
    void setCameraModel( const sensor_msgs::CameraInfoConstPtr& msg);


    /** @brief combine robot pose and marker observation to measurementData structure used by optimization interface
     * @param[in] pose MeasurementPose
     * @param[out] measurement measurementData structure containing pose and observation
     * @return true if measurementData could be created, false otherwise
     */
    bool createMeasurementDataFromMeasurementPose(const MeasurementPose & pose, measurementData & measurement);

    /** @brief publish a calibration state
     * @param[in] state the state to be published
     */
    void publishState(const CalibrationState & state) const;


    /* member variables */
    boost::shared_ptr<MeasurementPoseSet> m_resultSet; ///< the set of selected poses along with the corresponding calibration state
    std::map<string, Observation > m_poseIDToObservation; ///< mapping from poseId to observation
    boost::shared_ptr<PoseSourceType> m_poseSource; ///< the method to fill the pool
    std::map<std::string, KinematicChainType > m_kinematicChains ; ///< the kinematic chains
    boost::shared_ptr<CalibrationState> m_initialState; ///<  needed in order to (re)-initialize the m_resultSet
    boost::shared_ptr<G2oOptimizationInterface> m_optimizationInterface; ///< interface to an optimizer

    ModelLoader m_modelLoader; ///< load a robot model from ROS parameter server
    ros::Subscriber m_cameraInfoSubscriber; ///<  subscriber for camera images
    image_geometry::PinholeCameraModel m_cameraModel; ///<  the camera model used to set the CalibrationState m_initialState
    ros::Publisher calibrationPublisher; ///<  publisher for the calibration result
    ros::NodeHandle m_nhPrivate, m_nh; ///<  to access variables/subscribe to topics in global namespace and private namespace



    /** @name actual pose selection */
    ///@{
    boost::shared_ptr<PoseSelectionStrategy> m_initializationStrategy; ///< initial strategy to select poses
    boost::shared_ptr<PoseSelectionStrategy> m_incrementalStrategy; ///< incremental selection strategy
    boost::shared_ptr<ObservabilityIndex> m_observabilityIndex; ///< observability index used in pose selection
    size_t m_maximumNumberOfPoses; ///< maximum size of poses set
    size_t m_minimumNecessaryPoses; ///< minimum number of poses needed for optimization
    ///@}

};

}

#include <calibration/CalibrationFramework.hxx>

#endif // CALIBRATIONFRAMEWORK_H
