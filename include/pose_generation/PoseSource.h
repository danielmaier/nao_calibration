/*
 * PoseSource.h
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#ifndef POSESOURCE_H_
#define POSESOURCE_H_

#include <kinematic_calibration/measurementData.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <map>
#include <string>
#include <vector>

#include <common/PoseSet.h>


namespace kinematic_calibration {

/**
 * (Abstract) base class for getting measurement poses.
 */
class PoseSource {
public:
	/**
	 * Constructor.
	 */
	PoseSource() {
		// nothing to do
	}

	/**
	 * Destructor.
	 */
	virtual ~PoseSource() {
		// nothing to do
	}

	/**
     * Adds poses for the specified kinematic chain to the vector.
	 * The number of poses to be added is unspecified,
	 * i.e. the poses added can be within the range [0,inf].
	 * @param[in] kinematicChain The kinematic chain for which new
	 * 			measurement poses should be added.
     * @param[out] poses The vector to which the new poses should be added.
	 */
	virtual void getPoses(const KinematicChain& kinematicChain,
			vector<MeasurementPose>& poses) = 0;


	/**
	 * Returns an initial pose set for the specified number of poses and chain.
	 * @param[in] kinematicChain The kinematic chain.
	 * @param[in] state The calibration state used for initialization of the set.
	 * @param[in] n Number of poses.
	 * @return The initial pose set.
	 */
    virtual boost::shared_ptr<MeasurementPoseSet> getInitialPoseSet(
			const KinematicChain& kinematicChain,
			CalibrationState& state, const int& n);

};

/*
struct JointStatePose
{
public:
    sensor_msgs::JointState jointState;
    std::string id;
};
*/

class PoseSourcePool
{
public:
    typedef std::map< std::string, sensor_msgs::JointState> PoolOfPoses;
    typedef PoolOfPoses::iterator iterator;
    typedef PoolOfPoses::const_iterator const_iterator;

public:
    // will return true if pose has been added, false if id is already in pool
    bool addPose(const std::string & id, const sensor_msgs::JointState & jsp) {
        if (pool.count(id) > 0)
            return false;
        pool[id] = jsp;
        return true;
    }
    bool getById(const std::string & id, sensor_msgs::JointState & js){
        if (pool.count(id) == 0)
            return false;
        js = pool[id];
        return false;
    }
    unsigned int size() { return pool.size();}
    iterator begin() { return pool.begin(); }
    const_iterator begin() const { return pool.begin(); }
    iterator end() { return pool.end(); }
    const_iterator end() const { return pool.end(); }

protected:
    // mapping from an id to a joint state pose
    PoolOfPoses pool;

};




} /* namespace kinematic_calibration */

#endif /* POSESOURCE_H_ */
