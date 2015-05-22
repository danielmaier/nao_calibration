/*
 * TemperatureNode.h
 *
 *  Created on: 12.12.2013
 *      Author: stefan
 */

#ifndef TEMPERATURENODE_H_
#define TEMPERATURENODE_H_

#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alvalue/alvalue.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <string>
#include <vector>

using namespace AL;
using namespace ros;
using namespace std;

namespace kinematic_calibration {

/**
 * Node which checks the NAO's temperature.
 */
class TemperatureNode: public ALModule {
public:
	TemperatureNode(boost::shared_ptr<AL::ALBroker> broker,
			const std::string& name);
	virtual ~TemperatureNode();

	bool connectProxy();

	void run();

protected:
	boost::shared_ptr<AL::ALBroker> m_broker;
	boost::shared_ptr<AL::ALMemoryProxy> m_memoryProxy;
	boost::shared_ptr<AL::ALMotionProxy> m_motionProxy;
	void publishHotSensorFound(bool found, vector<string>& joints);

private:
	vector<string> dataNamesList;
	vector<string> jointList;
	string hotJointFoundTopic;

	NodeHandle nh;
	NodeHandle nhPrivate;
	Publisher pub;
};

} /* namespace kinematic_calibration */

#endif /* TEMPERATURENODE_H_ */
