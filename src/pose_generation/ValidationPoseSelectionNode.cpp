/*
 * ValidationPoseSelectionNode.cpp
 *
 *  Created on: 27.06.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/ValidationPoseSelectionNode.h"
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

using namespace boost;
using namespace ros;
using namespace std;


ValidationPoseSelectionNode::ValidationPoseSelectionNode(
		int numOfPartitionsPerChain, bool use_bag) : nhPrivate("~"),
        poseSource(numOfPartitionsPerChain, nhPrivate, use_bag), numOfPartitionsPerChain(numOfPartitionsPerChain) {
  // note: ValidationPoseSource's constructor calls MeasurementPoseSource constructor, which subscribes to msgs


}

ValidationPoseSelectionNode::~ValidationPoseSelectionNode() {
}

void ValidationPoseSelectionNode::run()
{
  std::string filePrefix, dirPrefix;
  nhPrivate.getParam("file_prefix", filePrefix);
  nhPrivate.getParam("directory_prefix", dirPrefix);

  // create actual pose selection object with source
  PoseSelectionNode node(poseSource);
  bool use_bag = false;
  nhPrivate.param("use_bagfile", use_bag, use_bag);
  // fill source with messages from bagfiles
  if (use_bag)
  {
     std::vector<string> bagfile_paths;
     nhPrivate.getParam("bagfiles", bagfile_paths);
     ROS_INFO("Calling addPosesFromBagFile with %zu bag files", bagfile_paths.size());
     poseSource.addPosesFromBagFile(bagfile_paths);
  }
  else
    ROS_INFO("Not using bagfile");

  //ROS_INFO("poseSource has %d poses", poses);
  // iterate over partitions
  for (int i = 1; i <= numOfPartitionsPerChain; i++)
  {
    // set current partition as validation set
    poseSource.setCurrentValidationPartition(i);
    vector< string > validationIds;
    // get corresponding ids for all chains
    map<string, map<int, vector<sensor_msgs::JointState> > >::iterator it;
    for(it = poseSource.getSplits().begin();
        it != poseSource.getSplits().end(); ++it)
    {
      cout << "validation ids for chain " << it->first << endl;
      map<int, vector<sensor_msgs::JointState> > js = it->second;
      vector < string > ids = poseSource.getPoseIds(js[i]);
      for (int j = 0; j < ids.size(); j++)
      {
        cout << "\"" << ids[j] << "\", ";
        validationIds.push_back( ids[j] );
      }
      cout << endl;

    }
    // perform actual pose selection
    shared_ptr < MeasurementPoseSet > poses = node.getOptimalPoseSetAndValidate(validationIds, i); // this calls poseSource.getPoses() which collects data and generates the measurement (optimization) set
    vector < string > ids = poseSource.getPoseIds(poses->getPoses());
    cout << "Optimized pose ids: " << endl;
    for (int j = 0; j < ids.size(); j++)
    {
      cout << "\"" << ids[j] << "\", ";
    }
    cout << endl;
    ids = poseSource.getPoseIds(poses->getUnusedPoses());
    cout << "Unused pose ids: " << endl;
    for (int j = 0; j < ids.size(); j++)
    {
      cout << "\"" << ids[j] << "\", ";
    }
    cout << endl;
    map<int, shared_ptr<MeasurementPoseSet> > sets = node.getIntermediatePoseSets();
    for (map<int, shared_ptr<MeasurementPoseSet> >::iterator it = sets.begin(); it != sets.end(); it++)
    {
      stringstream ss;
      ss << dirPrefix << "/" << filePrefix << "/" << "all_except_" << i;
      mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      ss << "/" << filePrefix << "_" << it->second->getNumberOfPoses();
      ofstream ofs(ss.str().c_str());
      if (!ofs.good())
      {
        cout << "Could not write the file  " << ss.str() << endl;
        break;
      }
      vector < string > ids = poseSource.getPoseIds(it->second->getPoses());
      for (int j = 0; j < ids.size(); j++)
      {
        ofs << "\"" << ids[j] << "\", ";
      }
      ofs.close();
    }

    stringstream indicesFilenameStream;
    indicesFilenameStream << dirPrefix << "/" << filePrefix << "/" << "all_except_" << i;
    indicesFilenameStream << "/" << "indices_" << filePrefix << "_" << sets.size() << ".csv";
    ofstream indicesOfs(indicesFilenameStream.str().c_str());
    map<int, double> indices = node.getIntermediateIndices();
    if (!indicesOfs.good())
    {
      cout << "Could not write the file  " << indicesFilenameStream.str() << endl;
    }
    indicesOfs << "NUMPOSES\tINDEX\n";
    for (map<int, double>::iterator it = indices.begin(); it != indices.end(); it++)
    {
      stringstream ss;
      ss << it->first << "\t" << it->second << "\n";
      indicesOfs << ss.str();
    }
    indicesOfs.close();
  }

}


} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	// initialize the node
	stringstream nodeName;
	//Eigen::initParallel();
	//Eigen::internal::setNbThreads(0);
	ros::Time::init();
	nodeName << "poseSelectionNode4";
	//nodeName << ros::Time::now().nsec;
	ros::init(argc, argv, nodeName.str().c_str());

	ros::NodeHandle nh, privateNh("~");
	int numOfSplits = 3;
	privateNh.param("num_of_splits", numOfSplits, numOfSplits);
	ROS_INFO(
			"Generating poses for %i-fold cross-validation. (Parameter: num_of_splits)",
			numOfSplits);

	bool use_bag = false;
	privateNh.param("use_bagfile", use_bag, use_bag);

	ValidationPoseSelectionNode node(numOfSplits, use_bag);
	node.run();
	return 0;
}
