/*
 * MeasurementEdge.cpp
 *
 *  Created on: 30.12.2013
 *      Author: stefan
 */

#include <g2o/core/base_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <image_geometry/pinhole_camera_model.h>
#include <kinematic_calibration/measurementData.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../../include/optimization/CameraIntrinsicsVertex.h"
//#include "../../include/optimization/MeasurementEdge.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/optimization/TransformationVertex.h"

namespace kinematic_calibration {

template<int D, class Derived>
MeasurementEdge<D, Derived>::MeasurementEdge(measurementData measurement,
		FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) :
		measurement(measurement), frameImageConverter(frameImageConverter), kinematicChain(
				kinematicChain), debug(false), jointOptimizationType(
				JOINT_OFFSETS) {
	BaseMultiEdge<D, measurementData>::resize(4);
	for (int i = 0; i < measurement.jointState.name.size(); i++) {
		jointPositions.insert(
            #if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
                    make_pair(measurement.jointState.name[i], measurement.jointState.position[i])
            #else
                    make_pair<string, double>(measurement.jointState.name[i], measurement.jointState.position[i]);
            #endif
                );

	}

	Derived& derivedObj = (Derived&) *this;
	derivedObj.setMeasurement(measurement);

	// set information matrix
	BaseMultiEdge<D, measurementData>::setInformation(
			Eigen::Matrix<double, D, D>::Identity());
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::computeError() {
	// check if components are initialized
	if (NULL == kinematicChain || NULL == frameImageConverter) {
		ROS_FATAL("Uninitialized components!");
		return;
	}

	// get the pointers to the vertices
	TransformationVertex* markerTransformationVertex =
			static_cast<TransformationVertex*>(this->_vertices[0]);
	JointOffsetVertex* jointOffsetVertex =
			static_cast<JointOffsetVertex*>(this->_vertices[1]);
	TransformationVertex* cameraToHeadTransformationVertex =
			static_cast<TransformationVertex*>(this->_vertices[2]);
	CameraIntrinsicsVertex* cameraIntrinsicsVertex =
			static_cast<CameraIntrinsicsVertex*>(this->_vertices[3]);

	// get transformation from end effector to camera
	tf::Transform headToEndEffector; // root = head, tip = end effector, e.g. wrist
	map<string, double> jointOffsets = jointOffsetVertex->estimate();
	if (JOINT_6D == jointOptimizationType) {
		map<string, KDL::Frame> jointFrames = getJointFrames();
		KinematicChain kc = kinematicChain->withFrames(jointFrames);
		kc.getRootToTip(jointPositions, jointOffsets, headToEndEffector);
	} else if (JOINT_OFFSETS == jointOptimizationType) {
		kinematicChain->getRootToTip(jointPositions, jointOffsets,
				headToEndEffector);
	} else if (NONE == jointOptimizationType) {
		kinematicChain->getRootToTip(jointPositions, headToEndEffector);
	}

	// get transformation from marker to end effector
	tf::Transform endEffectorToMarker = markerTransformationVertex->estimate();

	// get transformation from camera to head
	tf::Transform cameraToHead = cameraToHeadTransformationVertex->estimate();

	// get estimated camera intrinsics
	sensor_msgs::CameraInfo cameraInfo = cameraIntrinsicsVertex->estimate();
	this->frameImageConverter->getCameraModel().fromCameraInfo(cameraInfo);

	// calculate estimated x and y
	//endEffectorToMarker.setRotation(tf::Quaternion::getIdentity());
	tf::Transform cameraToMarker = endEffectorToMarker * headToEndEffector
			* cameraToHead;

	// set error
	Derived& derivedObj = (Derived&) *this;
	derivedObj.setError(cameraToMarker);
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setJointFrameVertex(const string& jointName,
		g2o::HyperGraph::Vertex* v) {
	int curSize = this->vertices().size();
	BaseMultiEdge<D, measurementData>::resize(curSize + 1);
	BaseMultiEdge<D, measurementData>::vertices()[curSize] = v;
	this->jointFrameVerticesIndexes[jointName] = curSize;
}

template<int D, class Derived>
map<string, KDL::Frame> MeasurementEdge<D, Derived>::getJointFrames() {
	map<string, KDL::Frame> jointFrames;
	for (map<string, int>::iterator it =
			this->jointFrameVerticesIndexes.begin();
			it != this->jointFrameVerticesIndexes.end(); it++) {
		tf::Transform tfTransform = dynamic_cast<TransformationVertex*>(BaseMultiEdge<D,
				measurementData>::vertices()[it->second])->estimate();
		KDL::Frame frame;
		tf::transformTFToKDL(tfTransform, frame);
		jointFrames[it->first] = frame;
	}
	return jointFrames;
}

template<int D, class Derived>
const FrameImageConverter* MeasurementEdge<D, Derived>::getFrameImageConverter() const {
	return frameImageConverter;
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setFrameImageConverter(
		FrameImageConverter* frameImageConverter) {
	this->frameImageConverter = frameImageConverter;
}

template<int D, class Derived>
const KinematicChain* MeasurementEdge<D, Derived>::getKinematicChain() const {
	return kinematicChain;
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setKinematicChain(
		KinematicChain* kinematicChain) {
	this->kinematicChain = kinematicChain;
}

template<int D, class Derived>
bool MeasurementEdge<D, Derived>::isDebug() const {
	return debug;
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setDebug(bool debug) {
	this->debug = debug;
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setJointOptimizationType(
		JointOptimizationType jointOptimizationType) {
	this->jointOptimizationType = jointOptimizationType;
}


/*
template<int D, class Derived>
void MeasurementEdge<D, Derived>::getLinearization()
{
    //std::vector<JacobianType, aligned_allocator<JacobianType> > _jacobianOplus; ///< jacobians of the edge (w.r.t. oplus)
    cout << this->_jacobianOplus[0] << endl;
}
*/

} /* namespace kinematic_calibration */

