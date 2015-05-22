/*
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */

#ifndef OPTIMIZATIONINTERFACE_H
#define OPTIMIZATIONINTERFACE_H


#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <kinematic_calibration/measurementData.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>

#include <common/KinematicChain.h>
#include <common/ModelLoader.h>
#include <common/CalibrationContext.h>
#include <optimization/CalibrationState.h>
#include <optimization/G2oJointOffsetOptimization.h>


namespace kinematic_calibration {


/** @brief Interface to an Optimizer
 *
 */
class G2oOptimizationInterface {
public:

    G2oOptimizationInterface(CalibrationContext * context); // TODO: Add context?
    virtual ~G2oOptimizationInterface() {}

    virtual bool initializeCameraFromFile();
    //virtual void initialize() {}
    virtual bool optimizeRobust(const CalibrationState & state);
    virtual bool optimize();
    virtual bool optimize(const CalibrationState & state);
    void processMeasurementDataMsg(const measurementDataConstPtr& msg);

    CalibrationState getResult(){return result;}
    vector<measurementData> &  getOptimizationData() {return optimizationData;}
    vector<measurementData> & getMeasurementPool(){return measurementsPool;}
    vector<measurementData> & getValidationData(){return validataionData;}

    int repeatCounter; // TODO: This should be set automatically and be protected

    void clearOptimizationData() { optimizationData.clear(); }
    void addOptimizationData(const measurementDataConstPtr & msg);

    void clearValidationData() { validataionData.clear(); }
    void addValidationData(const measurementDataConstPtr & msg);

protected:
    virtual bool addPosesFromBagFile(const std::vector<std::string> & filenames);
    void setCameraModelFromMsg(const sensor_msgs::CameraInfoConstPtr& msg);

    void addNewKinematicChain(const measurementDataConstPtr& msg);
    bool measurementOk(const measurementDataConstPtr& msg);

    // TODO: Rename validationData
    // TODO: validataionData and measurementsPool should not be part of the general interface (only for validation)
    // TODO: Also optimizatioNData of type measurementData is not really necessary, is it?
    // TODO: Maybe a template to kinematicCalibration would solve this issue
    vector<measurementData> optimizationData, validataionData, measurementsPool;
    CalibrationState result, initialState;
    image_geometry::PinholeCameraModel cameraModel;
    boost::shared_ptr<CalibrationContext> context;
    vector<KinematicChain> kinematicChains;
    boost::shared_ptr<G2oJointOffsetOptimization> m_optimization; // todo: inject
    ModelLoader modelLoader;
    KDL::Tree kdlTree;
    ros::Subscriber cameraInfoSubscriber;
    ros::Subscriber measurementSubsriber;
    ros::NodeHandle m_nhPrivate;
    string chainName;
    std::map<std::string, KinematicChain*> nameToChain;
    CalibrationOptions calibrationOptions;
};

}

#endif // OPTIMIZATIONINTERFACE_H
