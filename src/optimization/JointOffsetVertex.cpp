/*
 * JointOffsetVertex.cpp
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#include "../../include/optimization/JointOffsetVertex.h"

#include "../../include/common/KinematicChain.h"

namespace kinematic_calibration {


JointOffsetVertex::JointOffsetVertex(const map<string, double>& jointOffsets): initial(jointOffsets)
{
    for (map<string, double>::const_iterator it = initial.begin(); it != initial.end(); ++it)
    {
        jointNames.push_back(it->first);
    }
    this->_dimension = jointNames.size();
    this->setToOrigin();
}


void JointOffsetVertex::oplusImpl(const double* delta) {
    for (unsigned int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] += delta[i];
		updateMimicJoint(curName);
	}

}

void JointOffsetVertex::updateMimicJoint(const string& majorJoint) {
	for (map<string, string>::iterator it = this->mimicJoints.begin();
			it != this->mimicJoints.end(); ++it) {
		if (it->second == majorJoint) {
			this->_estimate[it->second] = this->_estimate[majorJoint];
			return;
		}
	}
}

void JointOffsetVertex::setMimicJoint(const string& jointName,
		const string& mimicJointName) {
	this->mimicJoints[jointName] = mimicJointName;
}

void JointOffsetVertex::setToOriginImpl() {
    // set initial offsets to 0
	for (int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] = 0;
	}
    // set offsets to intitial values (if available)
    //cout << "Reset all joint offsets in this vertex"<<endl;
    for (map<string, double>::const_iterator it = initial.begin(); it != initial.end(); ++it)
    {
        // cout << "Reset: " << it->first << " " << it->second << endl;
        this->_estimate[it->first] = it->second;
    }

}

int JointOffsetVertex::estimateDimension() const {
	return this->jointNames.size();
}

int JointOffsetVertex::minimalEstimateDimension() const {
	//return -1; // not supported
	return this->_estimate.size();
}

SingleJointOffsetVertex::SingleJointOffsetVertex(const string& jointName,
		JointOffsetListener* listener) :
		jointName(jointName), listener(listener) {

}

SingleJointOffsetVertex::~SingleJointOffsetVertex() {
	// nothing to do
}

void SingleJointOffsetVertex::oplusImpl(const double* delta) {
	this->_estimate += delta[0];
	if (listener) {
		listener->updateJointOffset(this->jointName, this->_estimate);
	}
}

void SingleJointOffsetVertex::setToOriginImpl() {
	this->_estimate = 0;
}

JointOffsetVertexContainer::JointOffsetVertexContainer() {
	// nothing to do
}

JointOffsetVertexContainer::~JointOffsetVertexContainer() {
	// nothing to do
}

void JointOffsetVertexContainer::addVertexForJoint(string jointName,
		bool fixed) {
	if (this->vertexMap.count(jointName) > 0) {
		// joint already exists, nothing to do
		return;
	}

	// handle the case that this joint is a "major mimic joint"
	if(mimicMajorJoints.count(jointName) > 0) {
		// check whether minor joint is already contained
		if(vertexMap.count(mimicMajorJoints[jointName]) > 0) {
			// minor joint is already contained -> set minor to fixed
			vertexMap[mimicMajorJoints[jointName]]->setFixed(true);
		}
	}

	// handle the case that this joint is a "minor mimic joint"
	if(mimicMinorJoints.count(jointName) > 0) {
		// check whether major joint is already contained
		if(vertexMap.count(mimicMinorJoints[jointName]) > 0) {
			// major joint is already contained -> set minor to fixed
			fixed = true;
		}
	}

	SingleJointOffsetVertex* vertex = new SingleJointOffsetVertex(jointName,
			this);
	vertex->setFixed(fixed);
	this->vertexMap[jointName] = vertex;
}

void JointOffsetVertexContainer::addVerticesForChain(
		const KinematicChain& chain, bool fixFirst, bool fixLast) {
	// get joint names
	vector < string > jointNames;
	chain.getJointNames(jointNames);

	// add joints
	for (int i = 0; i < jointNames.size(); i++) {
		string jointName = jointNames[i];

		// check whether the vertex should be fixed
		bool fixVertex = false;
		if (i == 0 && fixFirst) {
			fixVertex = true;
		} else if (i == jointNames.size() - 1 && fixLast) {
			fixVertex = true;
		}

		// delegate
		addVertexForJoint(jointNames[i], fixVertex);
	}
}

void JointOffsetVertexContainer::fixVertex(string jointName) {
	if (this->vertexMap.count(jointName) > 0) {
		this->vertexMap[jointName]->setFixed(true);
	}
}

const map<string, double> JointOffsetVertexContainer::getJointOffsets() const {
	return jointOffsets;
}

void JointOffsetVertexContainer::updateJointOffset(const string& jointName,
		double value) {
	if (this->vertexMap.count(jointName) > 0) {
		this->jointOffsets[jointName] = value;
	}
}

map<string, SingleJointOffsetVertex*> JointOffsetVertexContainer::getVerticesForChain(
		const KinematicChain& chain) {
	map<string, SingleJointOffsetVertex*> vertices;
	vector<string> jointNames;

	chain.getJointNames(jointNames);

	for(int i = 0; i < jointNames.size(); i++) {
		string jointName = jointNames[i];
		if(this->vertexMap.count(jointName)) {
			vertices[jointName] = this->vertexMap[jointName];
		}
	}

	return vertices;
}

map<string, SingleJointOffsetVertex*> JointOffsetVertexContainer::getAllVertices() const {
	return vertexMap;
}

void JointOffsetVertexContainer::deleteVertices() {
	// TODO
}

void JointOffsetVertexContainer::setMimicJoint(const string& jointName,
		const string& mimicJointName) {
	this->mimicMajorJoints[jointName] = mimicJointName;
	this->mimicMinorJoints[mimicJointName] = jointName;
}

void JointOffsetVertexContainer::updateMimicJoint(const string& majorJoint) {
	if (this->vertexMap.count(majorJoint) > 0 && this->mimicMajorJoints.count(majorJoint) > 0) {
		this->jointOffsets[mimicMajorJoints[majorJoint]] = this->jointOffsets[majorJoint];
	}
}

} /* namespace kinematic_calibration */

