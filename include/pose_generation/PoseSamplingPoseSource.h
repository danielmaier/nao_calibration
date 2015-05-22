#ifndef POSESAMPLINGPOSESOURCE_H
#define POSESAMPLINGPOSESOURCE_H

#include <pose_generation/PoseSource.h>

namespace kinematic_calibration
{

/**
 * Source of poses. Creates poses by sampling them.
 */
class PoseSamplingPoseSource: public PoseSource {
public:
    PoseSamplingPoseSource();
    virtual ~PoseSamplingPoseSource();

    /**
     * Returns poses by sampling.
     * @param[in] kinematicChain The kinematic chain for which new
     * 			measurement poses should be added.
     * @param[out] poses The list to which the new poses should be added.
     * TODO: Adjust for multiple chains?
     */
    virtual void getPoses(const KinematicChain& kinematicChain,
                          vector<MeasurementPose>& poses);

private:
    NodeHandle nhPrivate;
};
}

#endif // POSESAMPLINGPOSESOURCE_H
