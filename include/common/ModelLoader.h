/*
 * ModelLoader.h
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#ifndef MODELLOADER_H_
#define MODELLOADER_H_

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <string>
#include <urdf/model.h>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

using namespace std;

namespace kinematic_calibration {

/**
 * Helper class to load the model and convert it.
 */
class ModelLoader {
public:
	/**
	 * Constructor.
	 */
	ModelLoader();

	/**
	 * Deconstructor.
	 */
	virtual ~ModelLoader();


	/**
	 * Initializes the model by loading the xml string
	 * from the ROS parameter server.
	 */
	bool initializeFromRos();

	/**
	 * Initializes the model by using the passed xml string.
	 * @param[in] urdfXml xml string containing the URDF model.
	 * @return true if successful, otherwise false
	 */
	bool initializeFromUrdf(string urdfXml);

	/**
	 * Returns the model as KDL tree.
	 * @param[out] kdlTree the model as KDL tree.
	 */
	void getKdlTree(KDL::Tree& kdlTree);

	/**
	 * Returns the model as URDF model.
	 * @param[out] model the model as URDF model.
	 */
	void getUrdfModel(urdf::Model& model);

	/**
	 * Returns the urdf xml string.
	 * @return The urdf xml string.
	 */
	const string& getUrdfXml() const;

protected:
	bool loadUrdfFromRos();
	bool loadKdlFromUrdf();
	bool urdfStringToModel();

private:
	string urdfXml;
	urdf::Model urdfModel;
	KDL::Tree kdlTree;
	bool initialized;
};

} /* namespace kinematic_calibration */
#endif /* MODELLOADER_H_ */
