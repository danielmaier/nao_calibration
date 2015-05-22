/*
 * ValidationNodeMain.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: maierd
 */


#include "../../include/optimization/ValidationNode.h"

using namespace kinematic_calibration;

int main(int argc, char** argv) {
        ros::init(argc, argv, "ValidationNode");
        CalibrationContext* context = new RosCalibContext();
        ValidationNode node(context);
        node.spin();
        delete context;
        return 0;
}
