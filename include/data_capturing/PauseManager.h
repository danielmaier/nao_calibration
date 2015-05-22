/*
 * PauseManager.h
 *
 *  Created on: 13.12.2013
 *      Author: stefan
 */

#ifndef PAUSEMANAGER_H_
#define PAUSEMANAGER_H_

#include <kinematic_calibration/CmdPauseService.h>
#include <kinematic_calibration/hotJoint.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <set>
#include <string>
#include <vector>

using namespace ros;
using namespace std;

namespace kinematic_calibration {

/**
 * Class for determining whether it should be paused.
 */
class PauseManager {
public:
	/**
	 * Constructor.
	 */
    PauseManager(bool retrieveJointNamesAutomatically=true);

	/**
	 * Deconstructor.
	 */
	virtual ~PauseManager();

	/**
	 * If there is at least one request for a pause,
	 * sleep and check periodically for new pause/resume requests.
	 */
	void pauseIfRequested();

	/**
	 * Checks whether pause is requested
	 * @return true if pause is requested, false otherwise
	 */
	bool pauseRequested();

    void setJointNames(const std::vector<std::string> & inJointNames);

protected:
	bool pauseCb(CmdPauseService::Request& req, CmdPauseService::Response& res);
	bool resumeCb(CmdPauseService::Request& req, CmdPauseService::Response& res);
	void temperatureCb(kinematic_calibration::hotJointPtr msg);

private:
	NodeHandle nh;
	NodeHandle nhPrivate;
	CallbackQueue callbackQueue;
	ServiceServer pauseService;
	ServiceServer resumeService;
	Subscriber temperatureSubscriber;
	string hotJointTopic;
	set<string> pauseReasons;
	vector<string> jointNames;
};

} /* namespace kinematic_calibration */

#endif /* PAUSEMANAGER_H_ */
