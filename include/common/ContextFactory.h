/*
 * ContextFactory.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CONTEXTFACTORY_H_
#define CONTEXTFACTORY_H_

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>

#include "../../include/common/CalibrationContext.h"

namespace kinematic_calibration {

using namespace ros;
using namespace std;

/*
 *
 */
class ContextFactory {
public:
	static CalibrationContext* getRosContext();
	static CalibrationContext* getContext(string contextType);

private:
	ContextFactory();
	ContextFactory(const ContextFactory&);
	virtual ~ContextFactory();
};

} /* namespace kinematic_calibration */

#endif /* CONTEXTFACTORY_H_ */
