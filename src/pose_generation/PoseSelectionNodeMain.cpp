/*
 * PoseSelectionNodeMain.cpp
 *
 *  Created on: 27.06.2014
 *      Author: stefan
 */
 

//#include <boost/smart_ptr/make_shared.hpp>
//#include <kdl/tree.hpp>
//#include <opencv/cv.h>
//#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
//#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <tf/tf.h>
//#include <urdf/model.h>
//#include <urdf_model/joint.h>
//#include <urdf_model/pose.h>
#include <fstream>
#include <iostream>
//#include <string>

#include "../../include/common/PoseSet.h"
#include "../../include/pose_generation/PoseSampling.h"
#include "../../include/pose_generation/PoseSelectionNode.h"
#include "../../include/pose_generation/PoseSelectionStrategy.h"
 
using namespace kinematic_calibration;

int main(int argc, char** argv) {
	// initialize the node
	stringstream nodeName;
	ros::Time::init();
    nodeName << "poseSelectionNode";
    //nodeName << ros::Time::now().nsec;
	ros::init(argc, argv, nodeName.str().c_str());
	ros::NodeHandle nhPrivate("~"), nh;
	//CalibrationContext* context = new RosCalibContext();

	// initialize the pose source
	string poseSource = "sampling";
	nhPrivate.param("pose_source", poseSource, poseSource);
	ROS_INFO("The selected pose source is: %s", poseSource.c_str());

	// output filename
	string chainName;
	nh.getParam("chain_name", chainName);
	string fileName = "poses_" + chainName + "_generated.yaml";
	nhPrivate.param("pose_file", fileName, fileName);
    // read pool of poses from files
    if ("file" == poseSource) {
        std::vector<string> filepaths;
        nhPrivate.getParam("pose_files", filepaths);
        if (filepaths.empty())
        {
            ROS_ERROR("You need to specify at least one filepath for the poses");
            return 1;
        }
        FilePoseSource * poseSource = new FilePoseSource(filepaths);
        PoseSelectionNode node(*poseSource);
        boost::shared_ptr<MeasurementPoseSet> poses = node.getOptimalPoseSet();
        //vector<string> ids = poseSource->getPoseIds(poses->getPoses());
        poses->writeToFile(fileName);
    }
	if ("measurement" == poseSource) {
		MeasurementMsgPoseSource* poseSource = new MeasurementMsgPoseSource();
		PoseSelectionNode node(*poseSource);
        boost::shared_ptr<MeasurementPoseSet> poses = node.getOptimalPoseSet();
		vector<string> ids = poseSource->getPoseIds(poses->getPoses());
		cout << "Optimized pose ids: " << endl;
		for (int i = 0; i < ids.size(); i++) {
			cout << "\"" << ids[i] << "\", ";
		}
		cout << endl;
		ids = poseSource->getPoseIds(poses->getUnusedPoses());
		cout << "Unused pose ids: " << endl;
		for (int i = 0; i < ids.size(); i++) {
			cout << "\"" << ids[i] << "\", ";
		}
		cout << endl;

		// write out the intermediate sets
        map<int, boost::shared_ptr<MeasurementPoseSet> > sets = node.getIntermediatePoseSets();
        for (map<int, boost::shared_ptr<MeasurementPoseSet> >::iterator it = sets.begin();
				it != sets.end(); it++) {
			// open the file
			stringstream ss;
			ss << chainName << "_" << it->second->getNumberOfPoses();
			ofstream ofs(ss.str().c_str());
			if (!ofs.good()) {
				cout << "Could not write the file  " << ss.str() << endl;
				break;
			}
			vector<string> ids = poseSource->getPoseIds(it->second->getPoses());
			for (int i = 0; i < ids.size(); i++) {
				ofs << "\"" << ids[i] << "\", ";
			}
			ofs.close();
		}

		// write intermediate indices
		stringstream indicesFilenameStream;
		indicesFilenameStream << chainName << "_" << sets.size()
				<< "_indices.csv";
		ofstream indicesOfs(indicesFilenameStream.str().c_str());
		map<int, double> indices = node.getIntermediateIndices();
		if (!indicesOfs.good()) {
			cout << "Could not write the file  " << indicesFilenameStream.str()
					<< endl;
		}
		indicesOfs << "NUMPOSES\tINDEX\n";
		for (map<int, double>::iterator it = indices.begin();
				it != indices.end(); it++) {
			stringstream ss;
			ss << it->first << "\t" << it->second << "\n";
			indicesOfs << ss.str();
		}
		indicesOfs.close();
	} else if ("sampling" == poseSource) {
		PoseSamplingPoseSource* poseSource = new PoseSamplingPoseSource();
		PoseSelectionNode node(*poseSource);
        boost::shared_ptr<MeasurementPoseSet> poses = node.getOptimalPoseSet();
		poses->writeToFile(fileName);
	} else {
		ROS_ERROR("The specified pose source type '%s' is unknown.",
				poseSource.c_str());
	}
	return 0;
}
