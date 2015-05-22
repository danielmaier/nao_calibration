/*
 * CameraTransformUpdate.h
 *
 *  Created on: 17.12.2013
 *      Author: stefan
 */

#ifndef CAMERATRANSFORMUPDATE_H_
#define CAMERATRANSFORMUPDATE_H_

#include <tf/tf.h>
#include <urdf/model.h>
#include <string>
#include <urdf_model/model.h>

using namespace urdf;
using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class CameraTransformUpdate {
public:
	CameraTransformUpdate(Model model);
	virtual ~CameraTransformUpdate();

	void writeCalibrationData(const tf::Transform& headToCameraDelta,
			const string& filename);

	void getUpdatedCameraToHead(const tf::Transform& headToCameraDelta,
			tf::Transform& newCameraToHeadPitch);

private:
	Model model;
	tf::Transform headToCameraDelta;
};

} /* namespace kinematic_calibration */

#endif /* CAMERATRANSFORMUPDATE_H_ */
