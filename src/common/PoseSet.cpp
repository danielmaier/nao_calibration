/*
 * PoseSet.cpp
 *
 *  Created on: 03.03.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSet.h"

#include <boost/make_shared.hpp>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <utility>


namespace kinematic_calibration {



MeasurementPoseSet::MeasurementPoseSet(const CalibrationState& newstate) :
    poseSet( boost::shared_ptr<map<PoolKeyType, MeasurementPose> >( new map<PoolKeyType, MeasurementPose>())), state(newstate) {
}

MeasurementPoseSet::MeasurementPoseSet(const CalibrationState& newstate,
        boost::shared_ptr<map<PoolKeyType, MeasurementPose> > poseSet, const std::set<PoolKeyType> & activePoses, const KinematicChainToIDType & kinChainToId) :
    poseSet(poseSet), activePoses(activePoses), kinematicChainToIDs(kinChainToId), state(newstate) {
}

MeasurementPoseSet::MeasurementPoseSet(const MeasurementPoseSet & other) :
     poseSet(other.poseSet), activePoses(other.activePoses), state(other.state){
}

MeasurementPoseSet::~MeasurementPoseSet() {
}

bool MeasurementPoseSet::addMeasurementPose(MeasurementPose pose) {

    string id = pose.getID();

    if (poseSet->count(id)) // ID already contained
            return false;

    (*this->poseSet)[id] = pose; // and place pose there
    kinematicChainToIDs[pose.getKinematicChainName()].insert(id);
    return true;
}

void MeasurementPoseSet::unsetActive(const PoolKeyType & key)
{
    activePoses.erase(key);
}

void MeasurementPoseSet::deleteMeasurementPose(const string &ID)
{
    if (poseSet->count(ID) > 0)
    {
        kinematicChainToIDs[(*poseSet)[ID].getKinematicChainName()].erase(ID);
        poseSet->erase(ID);
    }
    unsetActive(ID);

}

bool MeasurementPoseSet::addMeasurementPoses(vector<MeasurementPose> poses) {
    bool success = true;
    for (vector<MeasurementPose>::const_iterator it = poses.begin(); it != poses.end(); it++) {
        if(!addMeasurementPose(*it))
            success = false;
	}
    return success;
}

bool MeasurementPoseSet::addActiveMeasurementPoses( vector<MeasurementPose> poses)
{
    bool success = true;
    for (vector<MeasurementPose>::const_iterator it = poses.begin(); it != poses.end(); it++) {
        if(addMeasurementPose(*it))
            activePoses.insert(it->getID());
        else
            success = false;
	}
    return success;
}

void MeasurementPoseSet::getJacobian(Eigen::MatrixXd& jacobian) const {
	// calculate all derivative vectors and store all of them in a list
	int rows = 0;
	int cols = 0;
	std::vector<Eigen::MatrixXd> derivativesList;
    for (set<PoolKeyType>::const_iterator it = this->activePoses.begin(); it != this->activePoses.end(); it++) {
		Eigen::MatrixXd derivativesVector;
		// compute partialDerivative for poseSet[ poseIdx ]
		(*this->poseSet)[*it].getPartialDerivatives(state, derivativesVector);
		derivativesList.push_back(derivativesVector);
		rows += derivativesVector.rows();
		cols = derivativesVector.cols();
	}

	// grow the jacobian matrix size and fill it rowwise with the derivative vectors
	jacobian.resize(rows, cols);
	int currentRow = 0;
    for (unsigned int i = 0; i < derivativesList.size(); i++) {
		for (int row = 0; row < derivativesList[i].rows(); row++) {
			jacobian.row(currentRow) << derivativesList[i].row(row);
			currentRow++;
		}
	}

}
void MeasurementPoseSet::initializePoseSet(const unsigned int& n, const std::string & chainName){
  // take the first n poses of the pose set
    for (map<PoolKeyType, MeasurementPose>::iterator it = this->poseSet->begin(); it != this->poseSet->end(); it++) {
        if (it->second.getChainName() != chainName)
            continue;
        if (activePoses.size() >= n)
            break;
        else
            activePoses.insert(it->first);
    }
}


void MeasurementPoseSet::initializePoseSet(const unsigned int& n) {
	// take the first n poses of the pose set
    for (map<PoolKeyType, MeasurementPose>::iterator it = poseSet->begin(); it != poseSet->end(); it++) {
        if (activePoses.size() >= n)
			break;
        else
            activePoses.insert(it->first);
	}
}

bool MeasurementPoseSet::setActive(const MeasurementPose& pose) {

    string id = pose.getID();
    // check if it exists
    if (!poseSet->count(id))
        return false;
    // if yes, make it active (if it isn't already)
    activePoses.insert(id);
    return true;
}

vector<boost::shared_ptr<MeasurementPoseSet> > MeasurementPoseSet::addPose() const {
    // declare return value
	vector<boost::shared_ptr<MeasurementPoseSet> > successors;

	// iterate through all available poses
    for (map<PoolKeyType, MeasurementPose>::iterator it = poseSet->begin(); it != poseSet->end(); it++) {
		// check whether the pose is already one of the active poses
        if (activePoses.end() != std::find(activePoses.begin(), activePoses.end(), it->first)) {
            // pose was found within the active poses, so it cannot extend the current set of selected poses
			continue;
		}

		// add the new pose
        std::set<PoolKeyType> newPoses = activePoses;
        newPoses.insert(it->first);

		// create a new pose set instance
        successors.push_back(boost::shared_ptr<MeasurementPoseSet>(new MeasurementPoseSet(state, poseSet, newPoses, kinematicChainToIDs)));
	}
	return successors;
}

vector<boost::shared_ptr<MeasurementPoseSet> > MeasurementPoseSet::removePose() const {
    // declare return value
	vector<boost::shared_ptr<MeasurementPoseSet> > successors;

	// iterate through all active poses
    for (set<PoolKeyType>::const_iterator remIt = activePoses.begin(); remIt != activePoses.end(); remIt++) {
		// copy all poses except the one to be removed
        set<PoolKeyType> newPoses = activePoses;
        newPoses.erase(*remIt);

		// create a new pose set instance
        successors.push_back(boost::shared_ptr<MeasurementPoseSet>(new MeasurementPoseSet(state, poseSet, newPoses, kinematicChainToIDs)));
	}

	return successors;
}

unsigned int MeasurementPoseSet::getNumberOfPoses() const {
    return activePoses.size();
}

vector<sensor_msgs::JointState> MeasurementPoseSet::getPoses() const {
    // declare return value
	vector<sensor_msgs::JointState> poses;
    poses.reserve( poseSet->size() );
    for (set<PoolKeyType>::const_iterator it = activePoses.begin(); it != activePoses.end(); it++) {
        poses.push_back((*poseSet)[*it].getJointState());
	}
	return poses;
}

vector<MeasurementPose> MeasurementPoseSet::getAllMeasurementPoses() const {
    // declare return value
    vector<MeasurementPose> poses;
    poses.reserve( this->poseSet->size() );
    for (map<PoolKeyType, MeasurementPose>::const_iterator it = poseSet->begin(); it != poseSet->end(); it++) {
        poses.push_back(it->second);
    }
    return poses;
}

vector<MeasurementPose> MeasurementPoseSet::getActiveMeasurementPoses() const {
    // declare return value
    vector<MeasurementPose> poses;
    poses.reserve( this->activePoses.size() );
    for (set<PoolKeyType>::const_iterator it = activePoses.begin(); it != activePoses.end(); it++) {
        poses.push_back((*this->poseSet)[*it]);
    }
    return poses;
}

vector<sensor_msgs::JointState> MeasurementPoseSet::getUnusedPoses() const {
	vector<sensor_msgs::JointState> poses;
    for (map<PoolKeyType, MeasurementPose>::const_iterator it = poseSet->begin(); it != poseSet->end(); it++) {
        if (activePoses.end() == std::find(activePoses.begin(), activePoses.end(), it->first)) {
			poses.push_back(it->second.getJointState());
		}
	}
	return poses;
}

bool MeasurementPoseSet::writeToFile(
		const string& filename) const {
	stringstream strStream;
	ofstream fStream(filename.c_str());
	int i = 1;
    for (set<PoolKeyType>::const_iterator it = this->activePoses.begin(); it != this->activePoses.end(); it++) {
        (*this->poseSet)[*it].toPoseManagerString(i++, strStream);
	}
	if(!fStream.good()) {
		return false;
	}

	fStream << strStream.str();
	fStream.flush();
	fStream.close();
	return true;
}

boost::shared_ptr<map<MeasurementPoseSet::PoolKeyType, MeasurementPose> > MeasurementPoseSet::getPosePool() const {
	return this->poseSet;
}

CalibrationState MeasurementPoseSet::getCalibrationState() const {
	return this->state;
}

bool MeasurementPoseSet::replacePose(PoolKeyType key, MeasurementPose newPose)
{
    if (poseSet->count(key) < 1)
    {
        return false;
    }
    (*poseSet)[key] = newPose;
    return true;
}

void MeasurementPoseSet::setCalibrationState(const CalibrationState &newstate)
{
    ROS_INFO("Resetting all partial derivatives when setting new KinematicCalibrationState");
    for (map<PoolKeyType, MeasurementPose>::iterator it = poseSet->begin(); it != poseSet->end(); ++it )
    {
        it->second.resetDerivatives();
    }
    this->state = newstate;
}


} /* namespace kinematic_calibration */
