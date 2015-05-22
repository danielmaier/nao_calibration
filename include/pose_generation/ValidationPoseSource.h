
#ifndef VALIDATIONPOSESOURCE_H_
#define VALIDATIONPOSESOURCE_H_


#include <common/Observation.h>
#include <pose_generation/MeasurementMsgPoseSource.h>
#include <boost/shared_ptr.hpp>

using namespace ros;
using namespace std;
using namespace boost;




namespace kinematic_calibration {

class ValidationPoseSource: public MeasurementMsgPoseSource {
public:
    ValidationPoseSource(int numOfPartitionsPerChain, const ros::NodeHandle & privateNH, bool use_bag=false );
    virtual ~ValidationPoseSource();

    void addPosesFromBagFile(const std::vector<std::string> & filenames );

    void setCurrentValidationPartition(unsigned int validationPartition);

    // fills the poses for the current split and the provided chain
    virtual void getPoses(const KinematicChain& kinematicChain,
            vector<MeasurementPose>& poses);

    /// override
    virtual boost::shared_ptr<MeasurementPoseSet> getInitialPoseSet(
            const KinematicChain& kinematicChain,
            CalibrationState& state, const int& n);

    // mapping chainName--> map< partition --> vector<JointStates> >
    map<string, map<int, PoseSourcePool> > & getSplits()
    {
      return splits;
    }
    bool getObservationForId( const std::string & id, Observation & obs)
    {
        if (observations.count(id) == 0)
            return false;
        obs = observations[id];
        return true;
    }

protected:
    virtual void measurementCb(const measurementDataConstPtr& msg);
    void splitPoseSets(int numOfPartitionsPerChain);
    int writeIds(vector<sensor_msgs::JointState>, string folderName,
            string fileName);
    int writeIds(const PoseSourcePool & psp, string folderName, string fileName);
    unsigned int numOfPartitionsPerChain;
    unsigned int validationPartition;
    std::string filePrefix;
    std::string dirPrefix;
    map<string, map<int, PoseSourcePool > > splits;
    NodeHandle nh;

    map<string, Observation > observations;
};
}

#endif
