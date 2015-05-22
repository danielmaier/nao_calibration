/*
 * ContextFactory.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/common/ContextFactory.h"

namespace kinematic_calibration {

ContextFactory::ContextFactory() {

}

CalibrationContext* ContextFactory::getRosContext() {
	return new RosCalibContext();
}

CalibrationContext* ContextFactory::getContext(string contextType) {
	if("ros" == contextType) {
		return new RosCalibContext();
	} else {
		// TODO: return default context ?!
		return NULL;
	}

}

ContextFactory::ContextFactory(const ContextFactory& other) {
}



ContextFactory::~ContextFactory() {
}

} /* namespace kinematic_calibration */
