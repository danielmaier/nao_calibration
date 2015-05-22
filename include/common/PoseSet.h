/*
 * PoseSet.h
 *
 *  Created on: 03.03.2014
 *      Author: stefan
 */

#ifndef POSESET_H_
#define POSESET_H_


#include <boost/shared_ptr.hpp>

#include <kinematic_calibration/measurementData.h>
#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include "ErrorModel.h"
#include <optimization/CalibrationState.h>
#include "MeasurementPose.h"

namespace kinematic_calibration {

using namespace std;
using namespace boost;

/**
 * MeasurementPoseSet contains a container of type MeasurementPose along.
 * It provides measures to mark/unmark "selected" poses
 * as well as means to compute the Jacobian of the selected poses
 */
class MeasurementPoseSet   {
public:
    typedef string PoolKeyType;
    typedef std::map<std::string, std::set<PoolKeyType> > KinematicChainToIDType;
public:

        /**
           * Constructor.
           */


	MeasurementPoseSet(const CalibrationState& state);

    MeasurementPoseSet(const CalibrationState& state, boost::shared_ptr<map<PoolKeyType, MeasurementPose> > poseSet, const std::set<PoolKeyType> & activePoses, const KinematicChainToIDType & kinChainToId);

	// Copy constructor
	MeasurementPoseSet(const MeasurementPoseSet & other);
	/**
	 * Destructor.
	 */
	virtual ~MeasurementPoseSet();

	/**
     * Adds a single pose to the pose set, if its ID is not contained yet (no overwriting)
	 * @param measurementPose Pose to be added.
     * @return true if pose has been added, false otherwise
	 */
    bool addMeasurementPose(MeasurementPose measurementPose);

    /**
     * @brief deleteMeasurementPose from the pose pool
     * @param ID of the pose to be deleted
     */
    void deleteMeasurementPose(const std::string & ID);

	/**
	 * Adds a set of poses to the pose set.
	 * @param measurementPosees Poses to be added.
	 */
    bool addMeasurementPoses(vector<MeasurementPose> measurementPoses);

	/**
	 * Adds a set of poses to the active pose set.
	 * @param measurementPosees Poses to be added.
	 */
    bool addActiveMeasurementPoses(vector<MeasurementPose> measurementPoses);

	/**
	 * Adds the given pose to the active poses if containted in the pose pool.
	 * @param[in] pose The pose to set active.
	 * @return True if success, false othwerwise.
	 */
	bool setActive(const MeasurementPose& pose);

    /**
     * @brief unsetActive removes a pose from the set of active poses by its KEY
     * @param key of the pose (corresponds to activePoses)
     */
    void unsetActive(const PoolKeyType & key);


    void setActivePoses(vector<PoolKeyType> & poses){
      activePoses.clear();
      for(vector<PoolKeyType>::const_iterator it = poses.begin(); it != poses.end(); ++it)
          activePoses.insert(*it);
	}

    const std::vector<PoolKeyType> getActivePoses() const{
        vector<PoolKeyType> keys;
        keys.reserve(activePoses.size());
        for(std::set<PoolKeyType>::const_iterator it = activePoses.begin(); it != activePoses.end(); ++it)
            keys.push_back(*it);
      return keys;
    }

    unsigned int getPoolSize() const {
        return getPosePool()->size();
    }

    bool getPoseByID(const PoolKeyType & id, MeasurementPose & pose)
    {

        if (poseSet->count(id))
        {
            pose = (*poseSet)[id];
            return true;
        }
        return false;
    }

    vector<PoolKeyType> getPoseKeys() const{
        vector<PoolKeyType> keys;
        keys.reserve(getPoolSize());
        for(map<PoolKeyType, MeasurementPose>::const_iterator it = poseSet->begin(); it != poseSet->end(); ++it)
            keys.push_back(it->first);
        return keys;
    }

    /** @brief get reference to all selected poses
     *
     */



	/**
	 * Returns all available poses.
	 * @return All available poses.
	 */
    boost::shared_ptr<map<PoolKeyType, MeasurementPose> > getPosePool() const;

    /**
     * Replace a pose in the pool
     * @param[in] key the Key to replace
     * @param[in] newPose the new MeasurementPose
     * @return True if pose could be replaced. False if the key does not exist in the pool
     */
    bool replacePose(PoolKeyType key, MeasurementPose newPose);

	/** Returns the kinematic calibration state.
	 * @return The kinematic calibration state.
	 */
    CalibrationState getCalibrationState() const;

    void setCalibrationState(const CalibrationState & state);


    /**
     * @brief computes the jacobian of the selected set
     * @param[out] jacobian the resulting jacobian
     */
    virtual void getJacobian(Eigen::MatrixXd& jacobian) const;
    virtual void initializePoseSet(const unsigned int& n);
    virtual void initializePoseSet(const unsigned int &n, const std::string & chainName);
    // Returns a vector of all possible successors to the current active poses by adding one (inactive) pose
    virtual vector< boost::shared_ptr<MeasurementPoseSet> > addPose() const;
        // Returns a vector of all possible successors to the current active poses by removing one active pose
    virtual vector< boost::shared_ptr<MeasurementPoseSet> > removePose() const;
    virtual unsigned int getNumberOfPoses() const;
    vector<sensor_msgs::JointState> getPoses() const; // TODO: Rename to getActivePoses or remove completely...
    vector<MeasurementPose> getActiveMeasurementPoses() const ;
    vector<MeasurementPose> getAllMeasurementPoses() const ;
	virtual vector<sensor_msgs::JointState> getUnusedPoses() const;

	/**
	 * Writes the used poses into a file.
	 * @param filename The file to be used.
	 * @return True if successful, false otherwise.
	 */
	bool writeToFile(const string& filename) const;

private:
	/**
	 * Set of all poses (active and currently unused!).
	 */
    boost::shared_ptr<map<PoolKeyType, MeasurementPose> > poseSet;

	/**
	 * Active poses (used e.g. for calculating the jacobian matrix).
	 */
    std::set<PoolKeyType> activePoses;

    KinematicChainToIDType kinematicChainToIDs;

	// TODO : Why does each MeasurementPoseSet require a KinematicCalibrationState ?
    CalibrationState state;

	// TODO Isn't this the same as poseSet->size() ?
	int numOfPoses;
};

} /* namespace kinematic_calibration */

#endif /* POSESET_H_ */
