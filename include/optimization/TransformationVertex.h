/*
 * TransformationVertex.h
 *
 *  Created on: 27.02.2014
 *      Author: stefan
 */

#ifndef TRANSFORMATIONVERTEX_H_
#define TRANSFORMATIONVERTEX_H_

#include <tf/tf.h>
#include <g2o/core/base_vertex.h>

namespace kinematic_calibration {

using namespace g2o;

/*
 * Vertex that represents a 3D transformation.
 */
class TransformationVertex: public BaseVertex<6, tf::Transform> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	TransformationVertex();
	virtual ~TransformationVertex();

	virtual bool read(std::istream& in);
	virtual bool write(std::ostream& out) const;
	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();
};

class TranslationVertex: public TransformationVertex {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	TranslationVertex() {
	}
	virtual ~TranslationVertex() {
	}

	void oplusImpl(const double* delta) {
		std::vector<double> deltaVec;
		double x, y, z;
		x = this->_estimate.getOrigin().getX();
		y = this->_estimate.getOrigin().getY();
		z = this->_estimate.getOrigin().getZ();
		for (int i = 0; i < 3; i++) {
			this->_estimate.setOrigin(
					tf::Vector3(x + delta[0], y + delta[1], z + delta[2]));
		}

	}
};

typedef TransformationVertex MarkerTransformationVertex;

} /* namespace kinematic_calibration */

#endif /* TRANSFORMATIONVERTEX_H_ */
