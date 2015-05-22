/*
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */

#ifndef DETMAXPOSESELECTIONSTRATEGY_H
#define DETMAXPOSESELECTIONSTRATEGY_H


#include <pose_generation/PoseSelectionStrategy.h>


namespace kinematic_calibration {

class DetmaxPoseSelectionStrategy: public PoseSelectionStrategy{
public:
    typedef map<string, vector<string> > CHAINTOPOSEID_TYPE;
  DetmaxPoseSelectionStrategy(unsigned int numOfPoses);
  virtual ~DetmaxPoseSelectionStrategy() {
  }
  boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
                          boost::shared_ptr<ObservabilityIndex> observabilityIndex,
                          double& index);
  vector<vector<MeasurementPoseSet::PoolKeyType> > getRandomIndices(int n, const CHAINTOPOSEID_TYPE & chainNameToPoseID);

protected:
  unsigned int numOfPoses;
  int numIterations;
  std::map<std::string, int> dofs;
};

}

#endif // DETMAXPOSESELECTIONSTRATEGY_H
