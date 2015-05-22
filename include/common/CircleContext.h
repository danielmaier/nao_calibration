/*
 * CircleContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CIRCLECONTEXT_H_
#define CIRCLECONTEXT_H_

#include <g2o/core/optimizable_graph.h>
#include <kinematic_calibration/measurementData.h>

#include "../data_capturing/CircleDetection.h"
#include "../data_capturing/MarkerDetection.h"
#include "../optimization/CircleMeasurementEdge.h"
#include "MarkerContext.h"

namespace kinematic_calibration {

/*
 *
 */
class CircleContext: public MarkerContext {
public:
	CircleContext();
	virtual ~CircleContext();

    virtual boost::shared_ptr<MarkerDetection> getMarkerDetectionInstance() {
        return boost::shared_ptr<MarkerDetection>(new RosCircleDetection(CircleDetection::Dummy));
		//return new RosCircleDetection(CircleDetection::HoughTransformAdaptive);
		//return new AveragingCircleDetection();
	}

	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(
			const measurementData& m, FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) {
		double radius;
		string parameterName = kinematicChain->getName() + "_marker_radius";
		radius = 0.011;
		if (nh.hasParam(parameterName))
			nh.getParam(parameterName, radius);
		else
			ROS_WARN("No parameter set for %s. Using default value %f.",
					parameterName.c_str(), radius);
		/*return new CircleMeasurementEdge(m, frameImageConverter, kinematicChain,
		 radius);*/
		LikelihoodCircleMeasurementEdge* edge =
				new LikelihoodCircleMeasurementEdge(m, frameImageConverter,
						kinematicChain, radius);
		bool debug;
		nh.param("debug_optimization", debug, false);
		edge->setDebug(debug);
		return edge;
	}
};

} /* namespace kinematic_calibration */

#endif /* CIRCLECONTEXT_H_ */
