/*
 *  Kinematic Calibration: A calibration framework for the humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */


#ifndef VALIDATIONOFFLINECALIBRATION_H
#define VALIDATIONOFFLINECALIBRATION_H


#include <calibration/CalibrationFramework.h>
#include <pose_generation/ValidationPoseSource.h>
#include <optimization/ValidationNode.h>

namespace kinematic_calibration {


class ValidationOfflineCalibration : public CalibrationFramework<ValidationPoseSource> {
public:
    ValidationOfflineCalibration();
    virtual ~ValidationOfflineCalibration();
    void validate(); // main function to call
protected:
    int observe();
    int observe(const std::vector<MeasurementPose> & poses);
    int observe(const std::vector<std::string> & poseIds);
    void selectAndOptimize(boost::shared_ptr<PoseSelectionStrategy> strategy);
    void initOptimization(); // overriden
    bool optimize();
    void initPoseSource();
    void setSplitId(int i) {m_splitID = i;}
    //void observe();


protected:

    /**
     * Map from pose set size to optimal poses given the pose set size.
     */
    std::map<int, boost::shared_ptr<MeasurementPoseSet> > m_optimalPoses;

    /**
     * Map from pose set size to intermediate pose set indices.
     */
    std::map<int, double> m_intermediateIndices;

    double m_splitID; // TODO: do we really need this?
    int m_num_of_splits;


};

}
#endif // VALIDATIONOFFLINECALIBRATION_H
