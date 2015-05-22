/*
 * TemperatureNode.cpp
 *
 *  Created on: 12.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/TemperatureNode.h"

#include <alerror/alerror.h>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/publisher.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <clocale>
#include <iostream>
#include <vector>

#include <kinematic_calibration/hotJoint.h>

namespace kinematic_calibration {

TemperatureNode::TemperatureNode(boost::shared_ptr<AL::ALBroker> broker,
		const std::string& name) :
		AL::ALModule(broker, name), m_broker(broker), nhPrivate("~") {
	if (!connectProxy()) {
		ROS_ERROR("Error!");
		throw std::exception();
	}

	if (!nhPrivate.getParam("hotjoint_topic", hotJointFoundTopic)) {
		hotJointFoundTopic = "nao_temperature/hot_joint_found";
	}
	pub = nh.advertise<kinematic_calibration::hotJoint>(hotJointFoundTopic, 10);

	dataNamesList.push_back(
			"Device/SubDeviceList/Battery/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LHand/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/LWristYaw/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RHand/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value");
	dataNamesList.push_back(
			"Device/SubDeviceList/RWristYaw/Temperature/Sensor/Value");
	
	jointList.push_back("Battery");
	jointList.push_back("HeadPitch");
	jointList.push_back("HeadYaw");
	jointList.push_back("LAnklePitch");
	jointList.push_back("LAnkleRoll");
	jointList.push_back("LElbowRoll");
	jointList.push_back("LElbowYaw");
	jointList.push_back("LHand");
	jointList.push_back("LHipPitch");
	jointList.push_back("LHipRoll");
	jointList.push_back("LHipYawPitch");
	jointList.push_back("LKneePitch");
	jointList.push_back("LShoulderPitch");
	jointList.push_back("LShoulderRoll");
	jointList.push_back("LWristYaw");
	jointList.push_back("RAnklePitch");
	jointList.push_back("RAnkleRoll");
	jointList.push_back("RElbowRoll");
	jointList.push_back("RElbowYaw");
	jointList.push_back("RHand");
	jointList.push_back("RHipPitch");
	jointList.push_back("RHipRoll");
	jointList.push_back("RKneePitch");
	jointList.push_back("RShoulderPitch");
	jointList.push_back("RShoulderRoll");
	jointList.push_back("RWristYaw");
}

TemperatureNode::~TemperatureNode() {
// TODO Auto-generated destructor stub
}

void TemperatureNode::run() {
	vector<float> memData;
	ALValue memDataNames(dataNamesList);

	// TODO: parameterize
	const float criticalUpperTemperature = 75.0;

	bool hotJointFound = false;

	while (ros::ok()) {
		vector<string> hotJoints;
		memData = m_memoryProxy->getListData(memDataNames);
		hotJointFound = false;

		if (memData.size() != memDataNames.getSize()) {
			ROS_ERROR("memData length %zu does not match expected length %u",
					memData.size(), memDataNames.getSize());
			continue;
		}

		ROS_INFO("Current Temperatures:");
		for (int i = 0; i < memData.size(); i++) {
			ROS_INFO("joint: %s, temperature: %f", dataNamesList[i].c_str(),
					memData[i]);
		}

		for (int i = 0; i < memData.size(); i++) {
			if (memData[i] > criticalUpperTemperature) {
				hotJointFound = true;
				hotJoints.push_back(jointList[i]);
			}
		}

		publishHotSensorFound(hotJointFound, hotJoints);
		usleep(1000 * 1000);
	}
}

bool TemperatureNode::connectProxy() {
	if (!m_broker) {
		ROS_ERROR("Broker is not ready. Have you called connectNaoQi()?");
		return false;
	}

	try {
		m_memoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(
				new AL::ALMemoryProxy(m_broker));
	} catch (const AL::ALError& e) {
		ROS_ERROR("Could not create ALMemoryProxy.");
		return false;
	}
	ROS_INFO("Proxy to ALMemory ready.");
	return true;
}

void TemperatureNode::publishHotSensorFound(bool found,
		vector<string>& joints) {
	ROS_INFO("Publish hotSensorFoundMessage: %s", found ? "true" : "false");

	// publish whether a hot joint was found
	hotJoint msg;
	msg.hotJointFound = found;
	msg.jointNames = joints;
	pub.publish(msg);
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

bool connectNaoQi();
void parse_command_line(int argc, char ** argv);

std::string m_pip("127.0.0.01");
std::string m_ip("0.0.0.0");
int m_port = 16712;
int m_pport = 9559;
std::string m_brokerName;
boost::shared_ptr<AL::ALBroker> m_broker;

void parse_command_line(int argc, char ** argv) {
	std::string pip;
	std::string ip;
	int pport;
	int port;
	boost::program_options::options_description desc("Configuration");
	desc.add_options()("help", "show this help message")("ip",
			boost::program_options::value<std::string>(&ip)->default_value(
					m_ip), "IP/hostname of the broker")("port",
			boost::program_options::value<int>(&port)->default_value(m_port),
			"Port of the broker")("pip",
			boost::program_options::value<std::string>(&pip)->default_value(
					m_pip), "IP/hostname of parent broker")("pport",
			boost::program_options::value<int>(&pport)->default_value(m_pport),
			"port of parent broker");
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	m_port = vm["port"].as<int>();
	m_pport = vm["pport"].as<int>();
	m_pip = vm["pip"].as<std::string>();
	m_ip = vm["ip"].as<std::string>();
	cout << "pip is " << m_pip << endl;
	cout << "ip is " << m_ip << endl;
	cout << "port is " << m_port << endl;
	cout << "pport is " << m_pport << endl;

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return;
	}
}

bool connectNaoQi() {
// Need this to for SOAP serialization of floats to work
	setlocale(LC_NUMERIC, "C");
// A broker needs a name, an IP and a port:
// listen port of the broker (here an anything)
	try {
		m_broker = ALBroker::createBroker(m_brokerName, m_ip, m_port, m_pip,
				m_pport, false);
	} catch (const ALError& e) {
		ROS_ERROR("Failed to connect broker to: %s:%d", m_pip.c_str(), m_port);

		return false;
	}
	cout << "broker ready." << endl;
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "TemperatureNode");
	m_brokerName = "TemperatureNodeBroker";
	parse_command_line(argc, argv);
	connectNaoQi();
	TemperatureNode node(m_broker, m_brokerName);
	node.run();
	return 0;
}
