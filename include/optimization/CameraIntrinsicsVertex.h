/*
 * CameraIntrinsicsVertex.h
 *
 *  Created on: 06.12.2013
 *      Author: stefan
 */

#ifndef CAMERAINTRINSICSVERTEX_H_
#define CAMERAINTRINSICSVERTEX_H_

#include <boost/array.hpp>
#include <g2o/core/base_vertex.h>
#include <sensor_msgs/CameraInfo.h>

using namespace g2o;

#define K_FX_IDX 0
#define K_FY_IDX 4
#define K_CX_IDX 2
#define K_CY_IDX 5

#define P_FX_IDX 0
#define P_FY_IDX 5
#define P_CX_IDX 2
#define P_CY_IDX 6

namespace kinematic_calibration {

typedef boost::array<double, 5> CameraIntrinsicsType;

/**
 * Vertex representing the camera parameters.
 */
class CameraIntrinsicsVertex: public BaseVertex<5, sensor_msgs::CameraInfo> {
public:
	CameraIntrinsicsVertex(sensor_msgs::CameraInfo cameraInfo);
	virtual ~CameraIntrinsicsVertex();

	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}
	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();

protected:
	sensor_msgs::CameraInfo initial;

};

} /* namespace kinematic_calibration */

#endif /* CAMERAINTRINSICSVERTEX_H_ */
