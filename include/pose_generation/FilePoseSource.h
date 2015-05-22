#ifndef FILEPOSESOURCE_H
#define FILEPOSESOURCE_H

#include <pose_generation/MeasurementMsgPoseSource.h>

namespace kinematic_calibration {
/**
  * Reads from (multiple) file(s) to obtain pool of possible poses.
  */
class FilePoseSource : public MeasurementMsgPoseSource {
public:

    /*
     * Constructor expects a vector of filepaths to yaml files
     */
    FilePoseSource(const std::vector<std::string> & filepaths);

    // Destructor
    virtual ~FilePoseSource();



    // TODO: a KinematicChain is not necessary, name should be sufficient
    virtual void getPoses(const KinematicChain& kinematicChain,
            vector<MeasurementPose>& poses);




    /*
    virtual shared_ptr<MeasurementPoseSet> getInitialPoseSet(
            const KinematicChain& kinematicChain,
            KinematicCalibrationState& state, const int& n);
            */

protected:
    virtual void addPosesFromBagFile(const std::vector<std::string> & filenames);
    virtual void addPosesFromYaml(const std::vector<std::string> & filenames);
    virtual void addPosesFromYaml(const std::string & filenames);


};


}

#endif // FILEPOSESOURCE_H
