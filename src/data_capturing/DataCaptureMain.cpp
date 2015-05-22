/*
 * DataCaptureMain.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>
#include <cstdlib>

#include "../../include/data_capturing/DataCapture.h"
#include "../../include/common/CalibrationContext.h"
#include "../../include/common/ContextFactory.h"

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "dataCapture");
	ros::NodeHandle nh, nhPrivate("~");

	// create the context object
	CalibrationContext* context = ContextFactory::getRosContext();

	string chainName;
	nh.getParam("chain_name", chainName);
	DataCapture* dataCapture;
	if ("larm" == chainName || "xylo_larm" == chainName) {
		dataCapture = new LeftArmDataCapture(*context);
	} else if ("rarm" == chainName || "xylo_rarm" == chainName) {
		dataCapture = new RightArmDataCapture(*context);
	} else if ("lleg" == chainName) {
		dataCapture = new LeftLegDataCapture(*context);
	} else if ("rleg" == chainName) {
		dataCapture = new RightLegDataCapture(*context);
	} else {
		ROS_FATAL("No parameter was set for the chain type!");
		exit(0);
	}

	bool manual;
	nhPrivate.param("manual_data_capturing", manual, false);

	if (manual) {
		ROS_INFO("Starting manual data capturing.");
		ROS_INFO("Every time the marker is visible and "
				"the joint state is stable, a measurement "
				"message will be published.");
		dataCapture->publishMeasurementLoop();
	} else {
        // Use all poses in a range uploaded to the pose manager
		dataCapture->playChainPoses();
	}
	delete context;
}
