/*
 */

#include "../../include/pose_generation/ValidationPoseSource.h"
#include "../../include/pose_generation/PoseSelectionNode.h"

#include <sys/stat.h>
#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Core>
#include <boost/foreach.hpp>

namespace kinematic_calibration {

using namespace std;
using boost::shared_ptr;


ValidationPoseSource::ValidationPoseSource(int numOfPartitionsPerChain, const ros::NodeHandle& nhPrivate, bool use_bag) :
    MeasurementMsgPoseSource(!use_bag),
    numOfPartitionsPerChain(numOfPartitionsPerChain), validationPartition(1)
{
  nhPrivate.getParam("file_prefix", filePrefix);
  nhPrivate.getParam("directory_prefix", dirPrefix);
  cout << "got file_prefix " << filePrefix << endl;
  cout << "got dir_prefix " << dirPrefix << endl;
  cout << "Using bag file: " << use_bag << endl;

}

ValidationPoseSource::~ValidationPoseSource()
{
}

void ValidationPoseSource::addPosesFromBagFile(const std::vector<std::string> & filenames)
{
  string msg_topic = "/kinematic_calibration/measurement_data";
  //string cam_info_topic = "/nao_camera/camera_info";

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(msg_topic);
  //topics.push_back(cam_info_topic);

  for(unsigned int i =0; i < filenames.size(); ++i)
  {
    cout << "Reading from " << filenames[i] << endl;
    // Load bag
    rosbag::Bag bag;
    bag.open(filenames[i], rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == msg_topic || ("/" + m.getTopic() == msg_topic))
      {
        kinematic_calibration::measurementData::ConstPtr meas_msg = m.instantiate<kinematic_calibration::measurementData>();
        if (meas_msg != NULL)
          measurementCb(meas_msg);
      }
      /*
      if (m.getTopic() == cam_info_topic || ("/" + m.getTopic() == cam_info_topic))
      {
        sensor_msgs::CameraInfo::ConstPtr info_msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (info_msg != NULL)
          node.camerainfoCallback(info_msg);
      }
      */
    }
    bag.close();
  }
  this->splitPoseSets(this->numOfPartitionsPerChain);
}

void ValidationPoseSource::splitPoseSets(int numOfPartitionsPerChain)
{
  this->splits.clear();
  for (map<string, PoseSourcePool >::iterator it = this->poses.begin(); it != this->poses.end(); it++)
  {
    string chainName = it->first;
    unsigned int size = it->second.size();
    unsigned int partitionSize = size / numOfPartitionsPerChain; // TODO: Watch out, this is intege division
    ROS_INFO("Partition size for chain %s is %d.", it->first.c_str(), partitionSize);
    PoseSourcePool::iterator poseIt = it->second.begin();
    for (unsigned int curPartition = 1; curPartition <= this->numOfPartitionsPerChain; curPartition++)
    {
      while (this->splits[chainName][curPartition].size() < partitionSize)
      {
        this->splits[chainName][curPartition].addPose(poseIt->first, poseIt->second);
        poseIt++;
      }
      // write the ids
      stringstream ss;
      ss << chainName << "_partition_" << curPartition;
      this->writeIds(this->splits[chainName][curPartition], dirPrefix + "/" + filePrefix, ss.str());
    }
  }
}

void ValidationPoseSource::setCurrentValidationPartition(unsigned int validationPartition)
{
  if (validationPartition >= 1 && validationPartition <= this->numOfPartitionsPerChain)
  {
    this->validationPartition = validationPartition;
    ROS_INFO("Validation partition number set to %d.", validationPartition);
  }
  else
  {
    ROS_INFO("Invalid partition number requested: %d.", validationPartition);
  }
}

void ValidationPoseSource::getPoses(const KinematicChain& kinematicChain, vector<MeasurementPose>& poses)
{
  ROS_INFO("ValidationPoseSource::getPoses() with pose size %zu", this->poses.size());
  if (this->poses.size() == 0)
  {
    this->collectData();
    this->splitPoseSets(this->numOfPartitionsPerChain);
  }
  for (unsigned int i = 1; i <= this->numOfPartitionsPerChain; i++)
  {
    if (i == this->validationPartition)
    {
      continue;
    }
    for (PoseSourcePool::const_iterator it = splits[kinematicChain.getName()][i].begin();
        it != splits[kinematicChain.getName()][i].end(); it++)
    {
        poses.push_back(MeasurementPose(kinematicChain, it->second, it->first));
    }
  }

}

shared_ptr<MeasurementPoseSet> ValidationPoseSource::getInitialPoseSet(const KinematicChain& kinematicChain,
                                                                       CalibrationState& state, const int& n)
{
  stringstream paramName;
  paramName << "initial_pose_ids/" << this->validationPartition << "/" << n;
  if (nh.hasParam(paramName.str()))
  {
    ROS_INFO("Initialize the pose set from give ids...");

    // get the ids
    vector < string > initialIds;
    nh.getParam(paramName.str(), initialIds);

    // get the pose pool
    vector < MeasurementPose > posePool;
    this->getPoses(kinematicChain, posePool);

    // get the respective poses
    vector < MeasurementPose > poses;
    this->getPosesByIds(kinematicChain, initialIds, poses);

    // initialize the pose set
    shared_ptr < MeasurementPoseSet > poseSet = make_shared < MeasurementPoseSet > (state);
    poseSet->addMeasurementPoses(posePool);
    for (unsigned int i = 0; i < poses.size(); i++)
    {
      poseSet->setActive(poses[i]);
    }

    return poseSet;
  }
  else
  {
    return MeasurementMsgPoseSource::getInitialPoseSet(kinematicChain, state, n);
  }
}

void ValidationPoseSource::measurementCb(const measurementDataConstPtr &msg)
{
    MeasurementMsgPoseSource::measurementCb(msg);
    Observation obs;
    obs.marker_data = msg->marker_data;
    obs.marker_type = msg->marker_type;
    obs.marker_frame = msg->marker_frame;
    obs.camera_frame = msg->camera_frame;
    if (observations.count(msg->id) > 0)
    {
        ROS_WARN("Observation for ID %s already contained! Overwriting..", msg->id.c_str());
    }
    observations[msg->id]=obs;
}

int ValidationPoseSource::writeIds(vector<sensor_msgs::JointState> jointStates, string folderName, string fileName)
{
    vector<string> ids = this->getPoseIds(jointStates);
    if (ids.size() != jointStates.size())
    {
        throw std::runtime_error("Failed to get ids for joint states in ValidationPoseSource::writeIds!");
    }
    PoseSourcePool psp;
    for( unsigned int i =0; i < jointStates.size(); ++i)
    {
        psp.addPose( ids[i], jointStates[i]);
    }
    return writeIds(psp, folderName, fileName);
}

int ValidationPoseSource::writeIds(const PoseSourcePool & psp, string folderName, string fileName)
{
  mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	stringstream ss;
	ss << folderName << "/" << fileName;
	ofstream ofs(ss.str().c_str());
	if (!ofs.good()) {
		cout << "Could not write the file  " << ss.str() << endl;
		return -1;
	}
    PoseSourcePool::const_iterator it;
    for (it = psp.begin(); it != psp.end(); ++it) {
        ofs << "\"" << it->first << "\", ";
	}
	ofs.close();

	return 0;
}

} /* namespace kinematic_calibration */

