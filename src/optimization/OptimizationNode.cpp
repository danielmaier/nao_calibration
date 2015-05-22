/*
 * OptimizationNode.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include <optimization/OptimizationNode.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <geometry_msgs/Transform.h>
#include <kinematic_calibration/calibrationResult.h>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <string>
#include <fstream>
#include <urdf_model/model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "camera_calibration_parsers/parse.h"

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"
#include "../../include/common/CalibrationContext.h"
#include "../../include/optimization/LocOptJointOffsetOptimization.h"
#include "../../include/common/MeasurementPose.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std;

namespace kinematic_calibration {





OptimizationNode::OptimizationNode(CalibrationContext* context) : G2oOptimizationInterface(context),
         nhPrivate("~"), collectingData(false) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&OptimizationNode::measurementCb, this);

    /*
	bool cam_from_file = false;
	nhPrivate.param("camera_from_file",cam_from_file, cam_from_file);
	if (cam_from_file)
	  initializeCameraFromFile();
	else
	  cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&OptimizationNode::camerainfoCallback, this);
            */

	resultPublisher = nh.advertise<kinematic_calibration::calibrationResult>(
			"/kinematic_calibration/calibration_result", 1);

	optimizationService = nh.advertiseService(
			"/kinematic_calibration/start_optimization",
			&OptimizationNode::startOptizationCallback, this);




    ros::NodeHandle nh;
    bool cam_from_file = false;
    m_nhPrivate.param("camera_from_file",cam_from_file, cam_from_file);
    if (cam_from_file && !initializeCameraFromFile())
    {
      ROS_ERROR("use_camera_file set but could not initialize from file");
      throw std::runtime_error("Bad camera initialization");
    }
    else
    {
      cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
            &OptimizationNode::camerainfoCallback, this);
    }



    bool use_bag = false;
    m_nhPrivate.param("use_bagfile", use_bag, use_bag);
    if (use_bag)
    {
      ROS_INFO("OptimizationNode will read data from bag files");
      this->collectingData = false;
      measurementSubsriber.shutdown();
      std::vector<string> bagfile_paths;
      m_nhPrivate.getParam("bagfiles", bagfile_paths);
      ROS_WARN("Adding poses from bagfiles is currently disabled in OptimizationNode");
      /*
      ROS_INFO("Calling addPosesFromBagFile with %zu bag files", bagfile_paths.size());
      if (!addPosesFromBagFile(bagfile_paths))
        throw std::runtime_error("Bad bagfile");
        */

    }
    else
    {
      measurementSubsriber = nh.subscribe("/kinematic_calibration/measurement_data", 3000, &OptimizationNode::measurementCb, this);
      ROS_INFO("Waiting for data...");
      collectData();
    }

}


OptimizationNode::~OptimizationNode() {
	// nothing to do
}

void OptimizationNode::startLoop() {
	ROS_INFO("Waiting for data...");
	collectData();
	ROS_INFO("Starting optimization...");
	optimize();
	ROS_INFO("Publishing results...");
	printPoints();
	printResult();
    //plotPointErrorHistogram();
	publishResults();
}

void OptimizationNode::collectData() {
	collectingData = true;
    ros::Rate r(10.0);
	while (collectingData && ros::ok()) {
		ros::spinOnce();
        r.sleep();
	}
	removeIgnoredMeasurements();
}


void OptimizationNode::printResult() {
	cout << "Optimized joint offsets:\n";
	typedef std::map<string, double>::iterator it_type;
	for (it_type iterator = result.jointOffsets.begin();
			iterator != result.jointOffsets.end(); iterator++) {
		cout << iterator->first << " : " << iterator->second << "\n";
	}

	for (std::map<string, tf::Transform>::iterator iterator =
			result.markerTransformations.begin();
			iterator != result.markerTransformations.end(); iterator++) {
		tf::Transform transform = iterator->second.inverse(); //TODO
		string name = iterator->first;
		double r, p, y;
		tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
		cout << "Optimized transform form marker to end effector for chain "
				<< name << ":\n";
		cout << "(x, y, z) " << transform.getOrigin().x() << " "
				<< transform.getOrigin().y() << " " << transform.getOrigin().z()
				<< " ";
		cout << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	cout << "Optimized transform form camera to head:\n";
    cout << "(x, y, z) " << result.cameraTransformation.getOrigin().x()
            << " " << result.cameraTransformation.getOrigin().y() << " "
            << result.cameraTransformation.getOrigin().z() << " ";
	cout << "(q0, q1, q2, q3) "
            << result.cameraTransformation.getRotation().x() << " "
            << result.cameraTransformation.getRotation().y() << " "
            << result.cameraTransformation.getRotation().z() << " "
            << result.cameraTransformation.getRotation().w() << "\n";

	for (map<string, tf::Transform>::iterator it =
			result.jointTransformations.begin();
			it != result.jointTransformations.end(); it++) {
		cout << "Optimized transform for joint " << it->first << ":\n";
		cout << "(x, y, z) " << it->second.getOrigin().x() << " "
				<< it->second.getOrigin().y() << " "
				<< it->second.getOrigin().z() << " ";
		double r, p, y;
		tf::Matrix3x3(it->second.getRotation()).getRPY(r, p, y);
		cout << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	cout << "Optimized camera intrinsics:\n";
	cout << "(fx,fy) " << result.cameraInfo.K[K_FX_IDX] << " "
			<< result.cameraInfo.K[K_FY_IDX] << " ";
	cout << "(cx,cy) " << result.cameraInfo.K[K_CX_IDX] << " "
			<< result.cameraInfo.K[K_CY_IDX] << "\n";
	cout << "D: " << result.cameraInfo.D[0] << ", " << result.cameraInfo.D[1]
			<< ", " << result.cameraInfo.D[2] << ", " << result.cameraInfo.D[3]
			<< ", " << result.cameraInfo.D[4] << "\n";
}

void OptimizationNode::printPoints() {
	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

    //Plot3D plotEndp;
	ofstream csvFile("optimization_points.csv");
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file!");
	}

	// generate the csv header
	csvFile << "ID;XMEASURED;YMEASURED;XOPTIMIZED;YOPTIMIZED;ERROR\n";

	// update kinematic chains
    for (unsigned int i = 0; i < this->kinematicChains.size(); i++) {
		this->kinematicChains[i] = this->kinematicChains[i].withTransformations(
				result.jointTransformations);
	}

	// print out the measured position and the transformed position
    for (unsigned int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];
		cout << current.id;
		cout << " measured(x,y): " << current.marker_data[0] << "  "
				<< current.marker_data[1];

		// get transformation from end effector to camera
		map<string, double> jointPositions;
        for (unsigned int j = 0; j < current.jointState.name.size(); j++) {
			jointPositions.insert(
            #if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
                    make_pair(current.jointState.name[i],
                            current.jointState.position[i])
#else
                        make_pair<string, double>(current.jointState.name[j],
                                current.jointState.position[j])
            #endif
                        );
		}

		tf::Transform cameraToEndEffector; // root = camera, tip = end effector, e.g. wrist
		map<string, double> jointOffsets = result.jointOffsets;
        for (unsigned int j = 0; j < this->kinematicChains.size(); j++) {
			if (kinematicChains[j].getName() == current.chain_name) {
				jointOffsets[this->kinematicChains[j].getTip()] = 0;
				this->kinematicChains[j].getRootToTip(jointPositions,
						jointOffsets, cameraToEndEffector);
			}
		}

		// get transformation from marker to end effector
		tf::Transform endEffectorToMarker =
				result.markerTransformations[current.chain_name];

		// get transformation from camera to head
        tf::Transform cameraToHead = result.cameraTransformation;

		// get estimated camera intrinsics
		sensor_msgs::CameraInfo cameraInfo = result.cameraInfo;
		frameImageConverter.getCameraModel().fromCameraInfo(cameraInfo);

		// calculate estimated x and y
		//endEffectorToMarker.setRotation(tf::Quaternion::getIdentity());
		tf::Transform cameraToMarker = endEffectorToMarker * cameraToEndEffector
				* cameraToHead;
		double x, y;
		frameImageConverter.project(cameraToMarker.inverse(), x, y);

		// write into image (if available)
		putToImage(current, x, y);

		// calculate distance between camera and marker
		tf::Vector3 origin = cameraToMarker.getOrigin();
		double dist = origin.length();

		double currentX = current.marker_data[0];
		double currentY = current.marker_data[1];

		double error = (fabs(currentX - x) * fabs(currentX - x)
				+ fabs(currentY - y) * fabs(currentY - y));

		cout << "\toptimized(x,y): " << x << " " << y;
		cout << "\tdifference(x,y): " << (currentX - x) << " "
				<< (currentY - y);
		cout << "\terror: " << error;
		cout << "\tdist: " << dist;
		cout << "\n";

		csvFile << current.id << ";" << currentX << ";" << currentY << ";" << x
				<< ";" << y << ";" << error << "\n";

		// add difference to plot
        //plotterDiff.addPoint((currentX - x), (currentY - y));
		//plotEndp.addPoint(origin.x(), origin.y(), origin.z());
	}

	// show difference plot
    //plotterDiff.showPlot();

	// write the csv file
	csvFile.flush();
	csvFile.close();
}

void OptimizationNode::plotPointErrorHistogram() {
	// find the maximum error
	double max = 0;
    for (unsigned int i = 0; i < measurements.size(); i++) {
		double error, x, y;
		measurementData md = measurements[i];
        for (unsigned int j = 0; j < this->kinematicChains.size(); j++) {
			if (kinematicChains[j].getName() == md.chain_name) {
                MeasurementPose mp(kinematicChains[j], md.jointState, "");
				mp.predictImageCoordinates(this->result, x, y);
				error = (md.marker_data[0] - x) * (md.marker_data[0] - x)
						+ (md.marker_data[1] - y) * (md.marker_data[1] - y);
				max = (error > max) ? error : max;
				break;
			}
		}
	}

	// prepare the gnulpot pipe
	FILE * gnuplotPipe;
	gnuplotPipe = popen("gnuplot -p", "w");
	fprintf(gnuplotPipe, "n=100;\n");
	fprintf(gnuplotPipe, "max=%f;\n", max);
	fprintf(gnuplotPipe, "min=%f;\n", 0.0);
	fprintf(gnuplotPipe, "width=(max-min)/n;\n");
	fprintf(gnuplotPipe, "hist(x,width)=width*floor(x/width)+width/2.0;\n");
	fprintf(gnuplotPipe,
			"set terminal svg size 600,400 fname 'Verdana' fsize 10\n");
	fprintf(gnuplotPipe, "set output \"optimization_error_histogram.svg\";\n");
	fprintf(gnuplotPipe, "set xrange [min:max];\n");
	fprintf(gnuplotPipe, "set yrange [0:];\n");
	fprintf(gnuplotPipe, "set offset graph 0.05,0.05,0.05,0.0;\n");
	fprintf(gnuplotPipe, "set xtics min,(max-min)/5,max;\n");
	fprintf(gnuplotPipe, "set boxwidth width*0.9;\n");
	fprintf(gnuplotPipe, "set style fill solid 0.5;\n");
	fprintf(gnuplotPipe, "set tics out nomirror;\n");
	fprintf(gnuplotPipe, "set xlabel \"Error\";\n");
	fprintf(gnuplotPipe, "set ylabel \"Frequency\";\n");
	fprintf(gnuplotPipe,
			"plot '-'  u (hist($1,width)):(1.0) smooth freq w boxes lc rgb\"green\" notitle\n");

	// add data to the histogram
    for (unsigned int i = 0; i < measurements.size(); i++) {
		double error, x, y;
		measurementData md = measurements[i];
        for (unsigned int j = 0; j < this->kinematicChains.size(); j++) {
			if (kinematicChains[j].getName() == md.chain_name) {
                MeasurementPose mp(kinematicChains[j], md.jointState, "");
				mp.predictImageCoordinates(this->result, x, y);
				error = (md.marker_data[0] - x) * (md.marker_data[0] - x)
						+ (md.marker_data[1] - y) * (md.marker_data[1] - y);
				fprintf(gnuplotPipe, "%f \n", error);
				break;
			}
		}

	}

	// plot it
	fprintf(gnuplotPipe, "e ,\n ");
	fflush(gnuplotPipe);
	fclose(gnuplotPipe);
}

bool OptimizationNode::putToImage(const string& id, const double& x,
		const double& y) {
	const string extension = "jpg"; // TODO: parameterize!

	// build filename
	stringstream name;
	name << "/tmp/" << id << "." << extension;

	// find image
	FILE *file = NULL;
	if (NULL != (file = fopen(name.str().c_str(), "r"))) {
		fclose(file);
	} else {
		// file not found
		ROS_INFO("File %s not found!", name.str().c_str());
		return false;
	}

	// read the image with OpenCV
	cv::Mat image = cv::imread(name.str());

	// put a cross to the optimized/calibrated position
	cv::line(image, cv::Point(x - 3, y - 3), cv::Point(x + 3, y + 3),
			CV_RGB(0, 255, 0));
	cv::line(image, cv::Point(x - 3, y + 3), cv::Point(x + 3, y - 3),
			CV_RGB(0, 255, 0));

	// write the image into a new file
	stringstream newname;
	newname << "/tmp/" << id << "_calibrated." << extension;
	return cv::imwrite(newname.str(), image);
}

bool OptimizationNode::putToImage(const measurementData& data, const double& x,
		const double& y) {
	// Convert from ROS message to OpenCV image.
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(data.image,
			sensor_msgs::image_encodings::BGR8);
	cv::Mat image = cv_ptr->image;

	// put a cross and a circle to the measured position
	std::vector<cv::Point2f> corners;
	corners.push_back(cv::Point2f(data.marker_data[0], data.marker_data[1]));
	cv::drawChessboardCorners(image, cv::Size(1, 1), corners, true);

	// put a cross to the optimized/calibrated position
	cv::line(image, cv::Point(x - 3, y - 3), cv::Point(x + 3, y + 3),
			CV_RGB(0, 255, 0));
	cv::line(image, cv::Point(x - 3, y + 3), cv::Point(x + 3, y - 3),
			CV_RGB(0, 255, 0));

	// write the image into a new file
	stringstream newname;
	newname << "/tmp/" << data.id << "_calibrated.jpg";
	return cv::imwrite(newname.str(), image);
}

void OptimizationNode::publishResults() {
	kinematic_calibration::calibrationResult msg;

	// joint offsets
	for (map<string, double>::iterator it = result.jointOffsets.begin();
			it != result.jointOffsets.end(); it++) {
		msg.jointNames.push_back(it->first);
		msg.jointOffsets.push_back(it->second);
	}

	// chain names and marker transformations
	for (map<string, tf::Transform>::iterator it =
			result.markerTransformations.begin();
			it != result.markerTransformations.end(); it++) {
		msg.chainNames.push_back(it->first);
		geometry_msgs::Transform transform;
		tf::transformTFToMsg(it->second, transform);
		msg.endeffectorToMarker.push_back(transform);
	}

	// camera intrinsics
	msg.cameraInfo = result.cameraInfo;

	// camera transform
	geometry_msgs::Transform cameraTransform;
    tf::transformTFToMsg(result.cameraTransformation.inverse(),
			cameraTransform);
	msg.cameraTransform = cameraTransform;

	// publish result
	resultPublisher.publish(msg);
}

void OptimizationNode::measurementCb(const measurementDataConstPtr& msg) {
     processMeasurementDataMsg(msg);
}
void OptimizationNode::camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    setCameraModelFromMsg(msg);
}


/*
void OptimizationNode::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	}

	else
		ROS_FATAL("Camera model could not be set!");
	cameraInfoSubscriber.shutdown();
}
*/

bool OptimizationNode::startOptizationCallback(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	this->collectingData = false;
	return true;
}



void OptimizationNode::removeIgnoredMeasurements() {
	// get list of IDs to be ignored
	XmlRpc::XmlRpcValue idList;
	if (!nh.hasParam("ignore_measurements"))
		return;
	nh.getParam("ignore_measurements", idList);
	ROS_ASSERT(idList.getType() == XmlRpc::XmlRpcValue::TypeArray);

	// create new list for measurement data
	vector<measurementData> filteredList;

	// check which measurements should be ignored
	for (vector<measurementData>::iterator it = measurements.begin();
			it != measurements.end(); it++) {
		string id = it->id;
		bool remove = false;
		for (int32_t i = 0; i < idList.size(); ++i) {
			ROS_ASSERT(idList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			string cid = static_cast<string>(idList[i]);
			if (id == cid) {
				remove = true;
			}
		}
		if (!remove)
			filteredList.push_back(*it);
	}

	// reassign the measurements list
	this->measurements = filteredList;
}


/*
class Plot3D {
public:
    Plot3D(string title = "") {
        gnuplotPipe = popen("gnuplot -persistent", "w");
        fprintf(gnuplotPipe,
                "set autoscale;\n set xzeroaxis\n set yzeroaxis\n set zzeroaxis\n");
        fprintf(gnuplotPipe,
                "splot '-' with points pt 1 lc rgb 'red' title '%s' \n",
                title.c_str());
    }

    void addPoint(double x, double y, double z) {
        fprintf(gnuplotPipe, "%f %f %f \n", x, y, z);
    }

    void showPlot() {
        fprintf(gnuplotPipe, "e ,\n ");
        fclose(gnuplotPipe);
    }

private:
    FILE * gnuplotPipe;
};
*/



} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "OptimizationNode");
	RosCalibContext context;
	OptimizationNode node(&context);
	node.startLoop();
	while (ros::ok())
		;
	return 0;
}
