/*
 * JointOffsetVertex.h
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSETVERTEX_H_
#define JOINTOFFSETVERTEX_H_

#include <g2o/core/base_vertex.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace kinematic_calibration {
class KinematicChain;
} /* namespace kinematic_calibration */

using namespace std;

namespace kinematic_calibration {

/**
 * Class that represents the vertex for the joint offsets.
 */
class JointOffsetVertex: public g2o::BaseVertex<30, map<string, double> > {
public:
	/**
	 * Default constructor.
	 */
	JointOffsetVertex() :
			jointNames(vector<string>()) {
		this->_dimension = jointNames.size();
		this->setToOrigin();
	}

	/**
	 * Constructor.
	 * @param jointNames Names of the joints which offsets should be optimized.
	 */
	JointOffsetVertex(const vector<string>& jointNames) :
			jointNames(jointNames) {
		this->_dimension = jointNames.size();
		this->setToOrigin();
	}

    JointOffsetVertex(const map<string, double>& jointOffsets) ;

	/**
	 * Deconstructor.
	 */
	virtual ~JointOffsetVertex() {
	}

	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();
	virtual int estimateDimension() const;
	virtual int minimalEstimateDimension() const;
	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}
	virtual void setJointNames(const vector<string>& jointNames) {
		this->jointNames = jointNames;
		this->_dimension = jointNames.size();
	}
	/**
	 * Sets the joint mimic for one pair of joints.
	 * @param jointName The name of major joint.
	 * @param mimicJointName The name of the joint that mimics the "major" joint.
	 */
	virtual void setMimicJoint(const string& jointName,
			const string& mimicJointName);

protected:
	/**
	 * Sets the value of the mimic joint to the same value as the "major" joint,
	 * if the pair is contained within the mimicJoints map.
	 * @param majorJoint The name of the "major" joint.
	 */
	void updateMimicJoint(const string& majorJoint);

    map<string, double> initial;
    vector<string> jointNames;
	map<string, string> mimicJoints;
};

class JointOffsetListener {
public:
	JointOffsetListener(){}
	virtual ~JointOffsetListener(){}
	virtual void updateJointOffset(const string& jointName, double value) = 0;
};

class SingleJointOffsetVertex: public g2o::BaseVertex<1, double> {
public:
	SingleJointOffsetVertex(const string& jointName, JointOffsetListener* listener = NULL);
	virtual ~SingleJointOffsetVertex();

	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();
	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}

protected:
	string jointName;
	JointOffsetListener* listener;
};

class JointOffsetVertexContainer: public virtual JointOffsetListener {
public:
	/// Constructor.
	JointOffsetVertexContainer();

	/// Destructor.
	virtual ~JointOffsetVertexContainer();

	/**
	 * Adds a vertex for the given joint.
	 * @param[in] jointName The name of the joint.
	 * @param[in] fixed Set the joint to "fixed", i.e. the value is not optimized by g2o.
	 */
	void addVertexForJoint(string jointName, bool fixed = false);

	/**
	 * Add vertices for all joints of the given chain.
	 * @param[in] chain The chain containing the joints for which nodes should be added.
	 * @param[in] fixFirst Should the first joint be "fixed"
	 * @param[in] fixLast Should the last joint be "fixed"
	 */
	void addVerticesForChain(const KinematicChain& chain, bool fixFirst = false,
			bool fixLast = false);

	/**
	 * Set a vertex to "fixed".
	 * @param[in] jointName The name of the joint.
	 */
	void fixVertex(string jointName);

	/**
	 * Returns the map of the current joint offsets.
	 * @return The map of the current joint offsets.
	 */
	const map<string, double> getJointOffsets() const;

	/**
	 * Updates the joint offset of a specific joint.
	 * Should be called from within the joint offsets vertices themselves.
	 * @param[in] jointName The joint which should be updated.
	 * @param[in] value The new offset value.
	 */
	void updateJointOffset(const string& jointName, double value);

	/**
	 * Returns the list of vertices belonging to the specified kinematic chain.
	 * @param[in] chain The kinematic chain for which the vertices are requested.
	 * @return The map of vertices belonging to the specified kinematic chain.
	 */
	map<string, SingleJointOffsetVertex*> getVerticesForChain(
			const KinematicChain& chain);

	/**
	 * Returns all vertices.
	 * @return All vertices.
	 */
	map<string, SingleJointOffsetVertex*> getAllVertices() const;

	/**
	 * Deletes all vertex instances.
	 * Attention: Usually, g2o takes the ownership!
	 * Call only if you know that you do, e.g. for unit testing.
	 */
	void deleteVertices();

	/**
	 * Sets the joint mimic for one pair of joints.
	 * @param jointName The name of major joint.
	 * @param mimicJointName The name of the joint that mimics the "major" joint.
	 */
	virtual void setMimicJoint(const string& jointName,
			const string& mimicJointName);

protected:
	/**
	 * Sets the value of the mimic joint to the same value as the "major" joint,
	 * if the pair is contained within the mimicJoints map.
	 * @param majorJoint The name of the "major" joint.
	 */
	void updateMimicJoint(const string& majorJoint);

	/// map of all vertices
	map<string, SingleJointOffsetVertex*> vertexMap;

	/// map of all joint offsets
	map<string, double> jointOffsets;

	/// map of mimic joints (major -> minor)
	map<string, string> mimicMajorJoints;

	/// map of mimic joints (minor -> major)
	map<string, string> mimicMinorJoints;
};

} /* namespace kinematic_calibration */

#endif /* JOINTOFFSETVERTEX_H_ */
