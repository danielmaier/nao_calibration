/*
 * FrameImageConverter.h
 *
 *  Created on: 28.10.2013
 *      Author: stefan
 */

#ifndef FRAMEIMAGECONVERTER_H_
#define FRAMEIMAGECONVERTER_H_

#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>

namespace kinematic_calibration {

/**
 * Class that encapsulates the conversion from 3D coordinates into 2D image coordinates.
 */
class FrameImageConverter {
public:
	/**
	 * Constructor.
	 * @param cameraModel Camera model to be used.
	 */
	FrameImageConverter(image_geometry::PinholeCameraModel cameraModel);

	/**
	 * Deconstructor.
	 */
	virtual ~FrameImageConverter();

	/**
	 * Projects the origin of the transformation into the 2D image coordinates.
	 * @param[in] transform Transformation to be used.
	 * @param[out] x Projected x position.
	 * @param[out] y Projected y position.
	 */
	void project(const tf::Transform& transform, double& x, double& y);

	/**
	 * Return the camera model.
	 * @return Camera model to be used.
	 */
	image_geometry::PinholeCameraModel& getCameraModel() {
		return cameraModel;
	}

	/**
	 * Set the camera model.
	 * @param cameraModel Camera model to be used.
	 */
	void setCameraModel(const image_geometry::PinholeCameraModel& cameraModel) {
		this->cameraModel = cameraModel;
	}

private:
	/**
	 * Camera model to be used.
	 */
	image_geometry::PinholeCameraModel cameraModel;
};

} /* namespace kinematic_calibration */
#endif /* FRAMEIMAGECONVERTER_H_ */
