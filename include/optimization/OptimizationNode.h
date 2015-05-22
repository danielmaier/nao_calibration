/*
 * OptimizationNode.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef OPTIMIZATIONNODE_H_
#define OPTIMIZATIONNODE_H_

#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <kinematic_calibration/measurementData.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>

#include <common/KinematicChain.h>
#include <common/ModelLoader.h>
#include <common/CalibrationContext.h>
#include <optimization/CalibrationState.h>
#include <optimization/G2oJointOffsetOptimization.h>
#include <optimization/OptimizationInterface.h>
using namespace ros;
using namespace std;

namespace kinematic_calibration {

/**
 * Node for collecting data and executing the optimization.
 */
class OptimizationNode : public G2oOptimizationInterface{
public:
	/**
	 * Constructor.
	 */
	OptimizationNode(CalibrationContext* context);

	/**
	 * Deconstructor.
	 */
	virtual ~OptimizationNode();

	/**
	 * Starts listening for measurement data
	 * and executing the optimization process.
	 */
	void startLoop();

protected:
	void collectData();
	void printResult();
    void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

	void printPoints();
	void plotPointErrorHistogram();

	void publishResults();

	bool putToImage(const string& id, const double& x, const double& y);
	bool putToImage(const measurementData& data, const double& x,
			const double& y);


    //void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	bool startOptizationCallback(std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);
    void measurementCb(const measurementDataConstPtr& msg);

	void removeIgnoredMeasurements();

private:
	NodeHandle nh;
	NodeHandle nhPrivate;
	ServiceServer optimizationService;
	Publisher resultPublisher;

	vector<measurementData> measurements;


    //Plot2D plotterDiff;

	bool collectingData;

};


/*
class Plot2D {
public:
    enum OutputType {
        WINDOW, SVG
    };

    Plot2D(string title = "", OutputType type = WINDOW) :
            title(title), type(type) {
        gnuplotPipe = popen("gnuplot ", "w");
        setTerm();
        fprintf(gnuplotPipe,
                "set autoscale;\n set xzeroaxis\n set yzeroaxis\n");
        fprintf(gnuplotPipe,
                "plot '-' with points pt 1 lc rgb 'green' title '%s' \n",
                title.c_str());
    }

    virtual ~Plot2D() {
        closePlot();
    }

    void addPoint(double x, double y) {
        fprintf(gnuplotPipe, "%f %f \n", x, y);
    }

    void showPlot() {
        fprintf(gnuplotPipe, "e ,\n ");
        fflush(gnuplotPipe);
    }

    void closePlot() {
        //fprintf(gnuplotPipe, "exit");
        fclose(gnuplotPipe);
    }

    void setTerm() {
        // TODO: use strategy pattern!
        switch (type) {
        case WINDOW:
            // do nothing (default)
            break;
        case SVG:
            fprintf(gnuplotPipe,
                    "set terminal svg size 350,262 fname 'Verdana' fsize 10\n");
            fprintf(gnuplotPipe, "set output '%s.svg'\n", title.c_str());
            break;
        default:
            // do nothing
            break;
        }
    }

private:
    FILE * gnuplotPipe;
    OutputType type;
    string title;
};
*/


} /* namespace kinematic_calibration */
#endif /* OPTIMIZATIONNODE_H_ */
