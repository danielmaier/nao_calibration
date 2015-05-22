/*
 * ValidationNode.cpp
 *
 *  Created on: 25.05.2014
 *      Author: stefan
 */

#include <optimization/ValidationNode.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf_conversions/tf_kdl.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <utility>
#include <gsl/gsl_statistics_double.h>
#include <gsl/gsl_sort_double.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include "camera_calibration_parsers/parse.h"
#include <errno.h>


#include "../../include/common/CalibrationContext.h"
#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/MeasurementPose.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/pose_generation/ObservabilityIndex.h"
#include "../../include/common/PoseSet.h"
#include <exception>
#include <kinematic_calibration/calibrationResult.h>

#include <camera_calibration_parsers/parse_yml.h>
#include "../../include/result_publishing/CameraTransformUpdate.h"
#include "../../include/result_publishing/JointUpdate.h"
#include "../../include/result_publishing/UrdfUpdate.h"

namespace kinematic_calibration {

ValidationNode::ValidationNode(CalibrationContext* context) : G2oOptimizationInterface(context),
    folderName("validation"), nhPrivate("~"), collectingData(false), m_modifyData(false) {


	resultPublisher = nh.advertise<kinematic_calibration::calibrationResult>(
	                        "/kinematic_calibration/calibration_result", 1);

	validationService = nh.advertiseService(
	    "kinematic_calibration/start_optimization",
	    &ValidationNode::startValidationCallback, this);

    ros::NodeHandle nh;
    bool cam_from_file = true;
    //m_nhPrivate.param("camera_from_file",cam_from_file, cam_from_file);
    if (cam_from_file && !initializeCameraFromFile())
    {
      ROS_ERROR("Failure trying to initializing camera from file ");
      throw std::runtime_error("Bad camera initialization");
    }

    bool use_bag = true;
    //m_nhPrivate.param("use_bagfile", use_bag, use_bag);
    if (use_bag)
    {
      ROS_INFO("Will evaluate data from bag files");
      this->collectingData = false;
      measurementSubsriber.shutdown();
      std::vector<string> bagfile_paths;
      m_nhPrivate.getParam("bagfiles", bagfile_paths);
      ROS_WARN("addPosesFromBagFile is disabled in ValidationNode");
      /*
      ROS_INFO("Calling addPosesFromBagFile with %zu bag files", bagfile_paths.size());
      if (!addPosesFromBagFile(bagfile_paths))
        throw std::runtime_error("Bad bagfile");
        */

    }
    //ROS_WARN("Spinning temporarily disabled. THIS IS A HACK! ");
	ROS_INFO("ValidationNode Ready");


}


/*
void ValidationNode::camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
}
*/


ValidationNode::~ValidationNode() {
	// nothing to do
}

// TODO: Is this function still needed? Who calls it?
/*
void ValidationNode::initialize() {
	nh.getParam("optimization_ids", optimizationDataIds);
	ROS_INFO("From param server: Got %zu optimization_ids", optimizationDataIds.size());
	nh.param("folder_name", folderName, folderName);
	//mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	// select the validation strategy
	string valDataStrategyType = "all";
	nh.param("validation_data_strategy", valDataStrategyType,
			valDataStrategyType);
	if ("all" == valDataStrategyType)
		this->valDataStrategy = boost::make_shared<ValidateOnAllStrategy>();
	else if ("others" == valDataStrategyType)
		this->valDataStrategy = boost::make_shared<ValidateOnOthersStrategy>();
	else if ("split" == valDataStrategyType)
		this->valDataStrategy = boost::make_shared<SplitStrategy>();
	else
		this->valDataStrategy = boost::make_shared<ValidateOnOthersStrategy>();

	// get the calibration options
	this->calibrationOptions = context->getCalibrationOptions();

    ROS_INFO("Validation initalized");
}
*/

void ValidationNode::spin() {
	while (ros::ok()) {
		ros::spinOnce();
	}
}
void ValidationNode::startNoiseLoop() {
    ROS_INFO("This is the noise loop. ");

    //
    ROS_INFO("(Re-)initialize...");
    //initialize();
    splitData();
    // make backup of optimizationData
    vector<measurementData> optimizationDataBackup = optimizationData;
    MeasurementsRandomizer randomizer;
    vector<bool> robust_kernel;
    robust_kernel.push_back(true);
    robust_kernel.push_back(false);
    string folderNameBackup = folderName;
    ROS_INFO("Base foldername is %s", folderName.c_str());
    vector< vector<measurementData> > noisyData;
    ROS_INFO("Adding noise from 0 to %zu poses", optimizationDataBackup.size());
    int numPoses = optimizationDataBackup.size();
    noisyData.push_back(optimizationData); // add data without noise (=idx 0)
    for (int i =0; i <= numPoses; ++i)
    {
        // modify optimizationData
        randomizer.wbRandomizeMeasurements(i, optimizationData, 640, 480);
        // store result in vector
        noisyData.push_back(optimizationData);
        // restore original optimizationData for next iteration
        optimizationData = optimizationDataBackup;
    }

    ROS_INFO("Starting optimization from noisy data");
    for (unsigned int j =0; j < robust_kernel.size(); ++j)
    {
        nh.setParam("use_robust_kernel", robust_kernel[j]);
        for (unsigned int i =0; i < noisyData.size(); ++i)
        {
            optimizationData = noisyData[i];
            // TODO
            folderName = folderName + "_" + boost::lexical_cast<string>(i) + "_" + boost::lexical_cast<string>(int(robust_kernel[j]));
            ROS_INFO("Writing to %s",folderName.c_str());
            if(mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
                throw std::runtime_error("Failed to create directory " + folderName);
            // TODO: Set filename
            ROS_INFO("Starting optimization...");
            this->repeatCounter = 0;
            G2oOptimizationInterface::optimize();
            printResult();
            ROS_INFO("Starting validation...");
            validate(); // TODO: Check if this uses some previous results
            writeCalibrationResults();
            folderName = folderNameBackup;
        }
        // restore data

    }
    ROS_INFO("Done!");
    ROS_INFO("Publishing results...");
    publishResults();
}

void ValidationNode::startLoop() {
	ROS_INFO("(Re-)initialize...");
    //initialize();
	splitData();
	modifyData();
	ROS_INFO("Starting optimization...");
    this->repeatCounter = 0;
    G2oOptimizationInterface::optimize();
	printResult();
	ROS_INFO("Starting validation...");
	validate();
	writeCalibrationResults();
	ROS_INFO("Done!");
	ROS_INFO("Publishing results...");
	publishResults();
}

void ValidationNode::collectData() {
	collectingData = true;
    ros::Rate r(20.0);
	while (collectingData && ros::ok()) {
		ros::spinOnce();
        r.sleep();
	}
}

void ValidationNode::splitData() {
	this->optimizationData.clear();
	this->validataionData.clear();
	cout << "measurementPool consists of";
	for(vector<measurementData>::iterator it = measurementsPool.begin(); it != measurementsPool.end(); ++it)
	{
	  //cout << it->id << ",";
	}
    //this->valDataStrategy->addMeasurements(optimizationData, validataionData,
    //		measurementsPool);
	ROS_INFO("OptimizationData: %ld; ValidationData: %ld",
			optimizationData.size(), validataionData.size());
}

void ValidationNode::modifyData() {
	// should some measurements be randomized?
    bool randomizeMeasurements = false;
    /*
    ROS_INFO("modify data is checking in namespace %s", nh.getNamespace().c_str());
    if (!nh.getParam("randomize_measurements_per_chain", randomizeMeasurements))
    {
        ROS_ERROR("Could not retrive randomize_measurements_per_chain");
        randomizeMeasurements = false;
    }
    */

    ROS_INFO("randomizeMeasurements is %d", randomizeMeasurements);
    if (randomizeMeasurements) {
        ROS_INFO("CALLING RANDOMIZER");
		MeasurementsRandomizer randomizer;
        randomizer.wbRandomizeMeasurements(5,
                optimizationData, 640, 480);
	}
    ROS_INFO("Done randomizer");

	// should (all) measurements be "noisified"?
	int addNoise = 0;
	nh.param("add_noise_to_measurements", addNoise, addNoise);
	if (addNoise > 0) {
		ROS_INFO("Add noise to measurements (%d pixel).", addNoise);
		MeasurementsNoiseAdder noisifier;
		noisifier.noisifyMeasurements(addNoise, optimizationData);
	}
}


bool ValidationNode::optimize(const CalibrationState & state) {
    /*
    if (! cameraModel.initialized())
    {
        ROS_ERROR("Camera Model is not initialized! Quitting.");
        throw std::runtime_error("Camera model not initialized in ValidatioNode::optimize()");
    }

	// instantiate the frame image converter
    FrameImageConverter frameImageConverter(cameraModel);

    // new
    frameImageConverter.getCameraModel().fromCameraInfo(state.cameraInfo);

    // initial state
    KinematicCalibrationState initialState = state;



    cout << "initial State is\n" << initialState << endl;

    ROS_INFO("Will optimize %zu poses", optimizationData.size());


    CalibrationOptions opt = context->getCalibrationOptions();
    OptimizationOptions optOptimization = context->getOptimizationOptions();

    cout << "optimization options are \n";
    cout << "maxIterations: " << optOptimization.maxIterations << endl;
    cout << opt.calibrateCameraIntrinsics << endl;
    cout << opt.markerOptimizationType << endl;
    cout << opt.calibrateJointOffsets << endl;


	// optimization instance
	G2oJointOffsetOptimization optimization(*context, optimizationData,
			kinematicChains, frameImageConverter, initialState);
	optimization.setSaveIntermediateStates(true);
	ROS_INFO("Call g2o...");

    int cjIterations = optimization.optimize(result);
    cout << "calibrated State is\n" << result << endl;

    BatchStatisticsContainer statistics = optimization.statistics;

//    cout << "Computed statistics. Size is " << statistics.size() << endl;
//    for (int i =0; i < statistics.size(); ++i){
//        cout << "--" << i << endl;
//        G2OBatchStatistics bat = statistics[i];
//        cout << bat.iteration << ", " << bat.levenbergIterations << "," << bat.chi2 << ", " << bat.iterationsLinearSolver << endl;
//    }

    if (cjIterations <= 8 && optimizationData.size() > 8 && optOptimization.maxIterations > 8 )
    {
        repeatCounter++;
        if (repeatCounter > 10)
            ROS_WARN("Too many retries. Giving up on restarting optimization..");
        else
        {
            ROS_WARN("It seems as if the optimization failed %d times! Restarting...",repeatCounter);
            return optimize(initialState);
        }
    }
    */

    if(m_modifyData)
        modifyData();


    bool success =   G2oOptimizationInterface::optimize(state);

	// get intermdiate states
    ROS_INFO("Getting intermediate States");
    m_optimization->getIntermediateStates(intermediateStates);

    // TODO: Compute covariance somehow and return result for covariance in sweet spot
    /*
    for (int s = 0; s < intermediateStates.size(); ++s)
    {
        KinematicCalibrationState kcs = intermediateStates[s];
        MeasurementPoseSet poseSet(kcs);
        vector<MeasurementPose> activePoses;
        for(int i =0; i < optimizationData.size(); ++i)
        {
            measurementData d = optimizationData[i];

            KinematicChain chain;
            for (int j = 0; j < this->chainNames.size(); ++j)
            {
                if (chainNames[j] == d.chain_name)
                    chain =this->kinematicChains[j];
            }
            sensor_msgs::JointState js = d.jointState;
            MeasurementPose m (chain, js, opt);
            activePoses.push_back(m);
        }
        poseSet.addActiveMeasurementPoses(activePoses);
        Eigen::MatrixXd jac;
        poseSet.getJacobian(jac);
        cout << "Covariance for intermediate state " << s << "is" << endl;
        //cout << jac << endl;
        Eigen::MatrixXd cov = (jac.transpose() * jac).inverse();
        //cout << "covariance is\n " << cov << endl;
        double prod = 1.0;
        double maxi = 0.0;
        for(int j = 0; j < cov.rows(); ++ j)
        {
            //cout << cov(j,j) << ",";
            prod *= cov(j,j);
            maxi = max(maxi, cov(j,j));
        }
        cout << " prod: " << prod << " maxi " << maxi ;
        cout << endl;
    }
    */
    return success;
}


double ValidationNode::validate() {
    ROS_INFO("Validating on %zu poses", validataionData.size());
    if(mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0 && errno != EEXIST)
       throw std::runtime_error("Failed to create directory " + folderName);
	printErrorPerIteration();
	printOptimizationError();
    double errorsum = printValidationError();
	printIntermediateResults();
    printPartlyError();
    return errorsum;
}

void ValidationNode::writeCalibrationResults(){

  ModelLoader cleanmodelLoader;
  cleanmodelLoader.initializeFromRos();

  urdf::Model cleanmodel;
  cleanmodelLoader.getUrdfModel(cleanmodel);

  string jointOffsetsFilename;
  string cameraTransformFilename;
  string urdfFilename;
  string markerTransformsFilename;
  string cameraIntrnsicsFilename;

  // get the filenames
  nh.param(string("joint_offsets_filename_suffix"), jointOffsetsFilename,
           string("calibration_joint_offsets.xacro"));
  nh.param(string("camera_transform_filename_suffix"),
           cameraTransformFilename,
           string("calibration_camera_transform.xacro"));
  nh.param(string("urdf_filename_suffix"), urdfFilename,
           string("robot_model_calibrated.xml"));
  nh.param(string("marker_transforms_filename_suffix"),
           markerTransformsFilename,
           string("calibration_marker_transformations.xacro"));
  nh.param(string("camera_intrnsics_filename"), cameraIntrnsicsFilename,
           string("nao_bottom_640x480.yaml"));

  // get the robot name and prepend to the filenames
  //stringstream ss;
  //ss << folderName << "/" << "comparison_error_per_iteration.csv";
  string filePrefix = folderName + "/" + cleanmodel.getName();
  jointOffsetsFilename = filePrefix + "_" + jointOffsetsFilename;
  cameraTransformFilename = filePrefix + "_" + cameraTransformFilename;
  urdfFilename = filePrefix + "_" + urdfFilename;
  markerTransformsFilename = filePrefix + "_" + markerTransformsFilename;
  cameraIntrnsicsFilename = filePrefix + "_" + cameraIntrnsicsFilename;

  JointUpdate ju(cleanmodel);
  ju.writeCalibrationData(result.jointOffsets, jointOffsetsFilename);

  // write the parameter file for the camera transformation
  //tf::Transform transform;
  //tf::transformMsgToTF(msg->cameraTransform, transform);

  CameraTransformUpdate ctu(cleanmodel);
  ctu.writeCalibrationData(result.cameraTransformation.inverse(), cameraTransformFilename);

  // write the new urdf file
  /*
  UrdfUpdate urdfUpdate;
  urdfUpdate.readFromRos("/robot_description");
  urdfUpdate.updateJointOffsets(result.jointOffsets);
  urdfUpdate.updateCameraDeltaTransform(result.cameraToHeadTransformation.inverse());
  urdfUpdate.writeToFile(urdfFilename);
  */

  // print the marker transformation on the screen and as parameters in a file
  // TODO: move into own class?
  // TODO: also update urdf...
  ofstream file;
  file.open(markerTransformsFilename.c_str());
  file << "<?xml version=\"1.0\"?>\n";
  file << "<robot>\n";

  for (map<string, tf::Transform>::iterator it =
                         result.markerTransformations.begin();
                         it != result.markerTransformations.end(); it++) {
         //        msg.chainNames.push_back(it->first);
          //       geometry_msgs::Transform transform;
           //      tf::transformTFToMsg(it->second, transform);
            //     msg.endeffectorToMarker.push_back(transform);

    double rr, rp, ry, tx, ty, tz;
    tf::Transform current = it->second;
    //tf::transformMsgToTF(msg->endeffectorToMarker[i], current);
    current = current.inverse();
    tf::Matrix3x3(current.getRotation()).getRPY(rr, rp, ry);
    tx = current.getOrigin().getX();
    ty = current.getOrigin().getY();
    tz = current.getOrigin().getZ();
    //cout << "chain: " << it->first << "\n";
    //cout << "translation: " << tx << " " << ty << " " << tz << "\n";
    //cout << "rotation: " << rr << " " << rp << " " << ry << "\n";
    file << "\t<property name=\"" << it->first << "_marker_tx"
        << "\" value=\"" << tx << "\" />\n";
    file << "\t<property name=\"" << it->first << "_marker_ty"
        << "\" value=\"" << ty << "\" />\n";
    file << "\t<property name=\"" << it->first << "_marker_tz"
        << "\" value=\"" << tz << "\" />\n";
    file << "\t<property name=\"" << it->first << "_marker_rr"
        << "\" value=\"" << rr << "\" />\n";
    file << "\t<property name=\"" << it->first << "_marker_rp"
        << "\" value=\"" << rp << "\" />\n";
    file << "\t<property name=\"" << it->first << "_marker_ry"
        << "\" value=\"" << ry << "\" />\n";
  }
  file << "</robot>\n";
  file.close();

  // write camera intrinsics
  const string cameraName = "nao_camera";
  camera_calibration_parsers::writeCalibrationYml(cameraIntrnsicsFilename,
                                                  cameraName,result.cameraInfo);
}

void ValidationNode::publishResults() {
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


bool ValidationNode::startValidationCallback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	this->collectingData = false;
	measurementSubsriber.shutdown();
    bool noise_mode;
    if (!nh.getParam("randomize_measurements_per_chain", noise_mode))
    {
        ROS_ERROR("Could not retrieve randomize_measurements_per_chain");
        noise_mode = false;
    }
    if (noise_mode)
        this->startNoiseLoop();
    else
        this->startLoop();
	return true;
}

void ValidationNode::printIntermediateResults() {
	IntermediateResultsCsvWriter writer;
	stringstream filename;
	filename << this->folderName << "/optimization_intermediate_results.csv";
	writer.writeToCsv(this->intermediateStates, filename.str());
}

void ValidationNode::printErrorPerIteration() {
	// init csv file
	stringstream ss;
	ss << folderName << "/" << "comparison_error_per_iteration.csv";
	ofstream csvFile(ss.str().c_str());
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file %s!", ss.str().c_str());
		return;
	}
	csvFile << "ITERATION\tOPTIMIZATIONERROR\tVALIDATIONERROR\t";
	csvFile << "OPTMEAN\tOPTVAR\tOPTMIN\tOPTMAX\tOPTLOQTIL\tOPTHIQTIL\t";
	csvFile << "VALMEAN\tVALVAR\tVALMIN\tVALMAX\tVALLOQTIL\tVALHIQTIL";

	if(kinematicChains.size() == 1) {
		csvFile << "\tOBSERVINDEX";
	}
	csvFile << "\n";

	NoiseAmplificationIndex indexCalculator;

	// init poses
	vector<MeasurementPose> optimizationPoses, validationPoses;

	map<string, KinematicChain> kinematicChainsMap;
    for (unsigned int i = 0; i < this->kinematicChains.size(); i++) {
		kinematicChainsMap[kinematicChains[i].getName()] = kinematicChains[i];
	}

    for (unsigned int i = 0; i < this->optimizationData.size(); i++) {
		optimizationPoses.push_back(
				MeasurementPose(
						kinematicChainsMap[this->optimizationData[i].chain_name],
						this->optimizationData[i].jointState,
                    this->optimizationData[i].id,
						calibrationOptions));
	}

    for (unsigned int i = 0; i < this->validataionData.size(); i++) {
		validationPoses.push_back(
				MeasurementPose(
						kinematicChainsMap[this->validataionData[i].chain_name],
						this->validataionData[i].jointState,
                    this->validataionData[i].id,
						calibrationOptions));
	}

	// iterate through all intermediate states
    for (unsigned int iteration = 0; iteration < this->intermediateStates.size();
			iteration++) {
		double optimizationError = 0.0, validationError = 0.0;
		vector<double> optErrorVec, valErrorVec;

		// calculate the optimized error
        for (unsigned int poseNum = 0; poseNum < this->optimizationData.size();
				poseNum++) {
			double x, y;
			optimizationPoses[poseNum].predictImageCoordinates(
					intermediateStates[iteration], x, y);
			double currentX = optimizationData[poseNum].marker_data[0];
			double currentY = optimizationData[poseNum].marker_data[1];
			double optError = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			optimizationError += optError;
			optErrorVec.push_back(optError);
		}

		// calculate the validation error
        for (unsigned int poseNum = 0; poseNum < this->validataionData.size();
				poseNum++) {
			double x, y;
			validationPoses[poseNum].predictImageCoordinates(
					intermediateStates[iteration], x, y);
			double currentX = validataionData[poseNum].marker_data[0];
			double currentY = validataionData[poseNum].marker_data[1];
			double valError = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			validationError += valError;
			valErrorVec.push_back(valError);
		}

		// prevent bad things
		if (optErrorVec.empty())
			optErrorVec.push_back(0.0);
		if (valErrorVec.empty())
			valErrorVec.push_back(0.0);

		// write errors
		csvFile << iteration << "\t" << optimizationError << "\t"
				<< validationError << "\t";

		gsl_sort(optErrorVec.data(), 1, optErrorVec.size());
		gsl_sort(valErrorVec.data(), 1, valErrorVec.size());

		// write some statistics
		csvFile << gsl_stats_mean(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_variance(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_min(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_max(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(optErrorVec.data(), 1,
						optErrorVec.size(), 0.25) << "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(optErrorVec.data(), 1,
						optErrorVec.size(), 0.75) << "\t";

		csvFile << gsl_stats_mean(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_variance(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_min(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_max(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(valErrorVec.data(), 1,
						valErrorVec.size(), 0.25) << "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(valErrorVec.data(), 1,
						valErrorVec.size(), 0.75);

		// observability index
		// TODO: This could be replaced, as index can be calculated for arbitrary chains now
		if(kinematicChains.size() == 1) {
			double obsIndex;
			MeasurementPoseSet poseSet(intermediateStates[iteration]);
			poseSet.addActiveMeasurementPoses(optimizationPoses);
			indexCalculator.calculateIndex(poseSet,	obsIndex);
			csvFile << "\t" << obsIndex;
		}

		// finish the line
		csvFile << "\n";
	}

	// close file
	csvFile.flush();
	csvFile.close();
}

void ValidationNode::printPartlyError() {
	// init csv file
	stringstream ss;
	ss << folderName << "/" << "partly_error.csv";
	ofstream csvFile(ss.str().c_str());
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file %s!", ss.str().c_str());
		return;
	}
	csvFile << "INITIAL\t";
	csvFile << "OPTIMIZATIONERROR\tVALIDATIONERROR\t";
	csvFile << "OPTMEAN\tOPTVAR\tOPTMIN\tOPTMAX\tOPTLOQTIL\tOPTHIQTIL\t";
	csvFile << "VALMEAN\tVALVAR\tVALMIN\tVALMAX\tVALLOQTIL\tVALHIQTIL";
	csvFile << "\n";

	// init part states
	vector<CalibrationState> partlyResults;
	vector<string> resetComponent;

	CalibrationState resetCameraIntrinsics = result;
	resetCameraIntrinsics.cameraInfo = this->initialState.cameraInfo;
	partlyResults.push_back(resetCameraIntrinsics);
	resetComponent.push_back("CameraIntrinsics");

	CalibrationState resetCameraExtrinsics = result;
    resetCameraExtrinsics.cameraTransformation =
            this->initialState.cameraTransformation;
	partlyResults.push_back(resetCameraExtrinsics);
	resetComponent.push_back("CameraExtrinsics");

	CalibrationState resetJointOffsets = result;
	resetJointOffsets.jointOffsets = this->initialState.jointOffsets;
	partlyResults.push_back(resetJointOffsets);
	resetComponent.push_back("JointOffsets");

	CalibrationState resetMarker = result;
	resetMarker.markerTransformations =
			this->initialState.markerTransformations;
	partlyResults.push_back(resetMarker);
	resetComponent.push_back("MarkerTransformations");

	// init poses
	vector<MeasurementPose> optimizationPoses, validationPoses;

	map<string, KinematicChain> kinematicChainsMap;
    for (unsigned int i = 0; i < this->kinematicChains.size(); i++) {
		kinematicChainsMap[kinematicChains[i].getName()] = kinematicChains[i];
	}

    for (unsigned int i = 0; i < this->optimizationData.size(); i++) {
		optimizationPoses.push_back(
				MeasurementPose(
						kinematicChainsMap[this->optimizationData[i].chain_name],
						this->optimizationData[i].jointState,
                    this->optimizationData[i].id,
						calibrationOptions));
	}

    for (unsigned int i = 0; i < this->validataionData.size(); i++) {
		validationPoses.push_back(
				MeasurementPose(
						kinematicChainsMap[this->validataionData[i].chain_name],
						this->validataionData[i].jointState,
                    this->validataionData[i].id,
						calibrationOptions));
	}

	// iterate through all intermediate states
    for (unsigned int num = 0; num < partlyResults.size(); num++) {
		double optimizationError = 0.0, validationError = 0.0;
		vector<double> optErrorVec, valErrorVec;

		// calculate the optimized error
        for (unsigned int poseNum = 0; poseNum < this->optimizationData.size();
				poseNum++) {
			double x, y;
			optimizationPoses[poseNum].predictImageCoordinates(
					partlyResults[num], x, y);
			double currentX = optimizationData[poseNum].marker_data[0];
			double currentY = optimizationData[poseNum].marker_data[1];
			double optError = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			optimizationError += optError;
			optErrorVec.push_back(optError);
		}

		// calculate the validation error
        for (unsigned int poseNum = 0; poseNum < this->validataionData.size();
				poseNum++) {
			double x, y;
			validationPoses[poseNum].predictImageCoordinates(partlyResults[num],
					x, y);
			double currentX = validataionData[poseNum].marker_data[0];
			double currentY = validataionData[poseNum].marker_data[1];
			double valError = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			validationError += valError;
			valErrorVec.push_back(valError);
		}

		// prevent bad things
		if (optErrorVec.empty())
			optErrorVec.push_back(0.0);
		if (valErrorVec.empty())
			valErrorVec.push_back(0.0);

		// write errors
		csvFile << resetComponent[num] << "\t" << optimizationError << "\t"
				<< validationError << "\t";

		gsl_sort(optErrorVec.data(), 1, optErrorVec.size());
		gsl_sort(valErrorVec.data(), 1, valErrorVec.size());

		// write some statistics
		csvFile << gsl_stats_mean(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_variance(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_min(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_max(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(optErrorVec.data(), 1,
						optErrorVec.size(), 0.25) << "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(optErrorVec.data(), 1,
						optErrorVec.size(), 0.75) << "\t";

		csvFile << gsl_stats_mean(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_variance(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_min(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_max(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(valErrorVec.data(), 1,
						valErrorVec.size(), 0.25) << "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(valErrorVec.data(), 1,
						valErrorVec.size(), 0.75);

		// finish the line
		csvFile << "\n";
	}

	// close file
	csvFile.flush();
	csvFile.close();
}

double  ValidationNode::printOptimizationError() {
    return printError(optimizationData, "optimization_error.csv");

}

double ValidationNode::printValidationError() {
    return printError(validataionData, "validation_error.csv");
}

double ValidationNode::printError(vector<measurementData>& measurements,
		string filename) {
	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

	stringstream ss;
	ss << folderName << "/" << filename;
	ofstream csvFile(ss.str().c_str());
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file %s!", ss.str().c_str());
        return -1;
	}

	// generate the csv header
	csvFile
			<< "ID\tXMEASURED\tYMEASURED\tXOPTIMIZED\tYOPTIMIZED\tXDIFF\tYDIFF\tERROR\n";

	// update kinematic chains
    for (unsigned int i = 0; i < this->kinematicChains.size(); i++) {
		this->kinematicChains[i] = this->kinematicChains[i].withTransformations(
				result.jointTransformations);
	}
    double errorsum = 0.0;

	// print out the measured position and the transformed position
    for (unsigned int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];

		// get transformation from end effector to camera
		map<string, double> jointPositions;
        for (unsigned int i = 0; i < current.jointState.name.size(); i++) {
			jointPositions.insert(
            #if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
                    make_pair(current.jointState.name[i], current.jointState.position[i])
            #else
                        make_pair<string, double>(current.jointState.name[i], current.jointState.position[i])
            #endif
                        );
		}

		tf::Transform cameraToEndEffector; // root = camera, tip = end effector, e.g. wrist
		map<string, double> jointOffsets = result.jointOffsets;
        for (unsigned int j = 0; j < this->kinematicChains.size(); j++) {
			if (kinematicChains[j].getName() == current.chain_name) {
				jointOffsets[this->kinematicChains[j].getTip()] = 0; // TODO: Is this necessary? TipJoint should not be estimated anyway...
				if (calibrationOptions.calibrateJoint6D) {
					// optimization of joint 6D offsets
					KinematicChain kc =
							this->kinematicChains[j].withTransformations(
									result.jointTransformations);
					kc.getRootToTip(jointPositions, jointOffsets,
							cameraToEndEffector);
				} else {
					// joint angle offsets only
					this->kinematicChains[j].getRootToTip(jointPositions,
							jointOffsets, cameraToEndEffector);
				}
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

//		// calculate distance between camera and marker
//		tf::Vector3 origin = cameraToMarker.getOrigin();
//		double dist = origin.length();

		double currentX = current.marker_data[0];
		double currentY = current.marker_data[1];

		double error = (fabs(currentX - x) * fabs(currentX - x)
				+ fabs(currentY - y) * fabs(currentY - y));

		csvFile << current.id << "\t" << currentX << "\t" << currentY << "\t"
				<< x << "\t" << y << "\t" << (currentX - x) << "\t"
				<< (currentY - y) << "\t" << error << "\n";
            errorsum += error;
	}

	// write the csv file
	csvFile.flush();
    csvFile.close();
    return errorsum;
}

map<string, KDL::Frame> ValidationNode::getJointFrames() {
	map<string, tf::Transform> jointTransforms =
			this->result.jointTransformations;
	map<string, KDL::Frame> jointFrames;
	for (map<string, tf::Transform>::iterator it = jointTransforms.begin();
			jointTransforms.end() != it; ++it) {
		KDL::Frame frame;
		tf::transformTFToKDL(it->second, frame);
		jointFrames[it->first] = frame;
	}
	return jointFrames;
}


void ValidationNode::printResult() {
	stringstream ss;
	ss << "Optimized joint offsets:\n";
	typedef std::map<string, double>::iterator it_type;
	for (it_type iterator = result.jointOffsets.begin();
			iterator != result.jointOffsets.end(); iterator++) {
		ss << iterator->first << " : " << iterator->second << "\n";
	}

	for (std::map<string, tf::Transform>::iterator iterator =
			result.markerTransformations.begin();
			iterator != result.markerTransformations.end(); iterator++) {
		tf::Transform transform = iterator->second.inverse(); //TODO
		string name = iterator->first;
		double r, p, y;
		tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
		ss << "Optimized transform form marker to end effector for chain "
				<< name << ":\n";
		ss << "(x, y, z) " << transform.getOrigin().x() << " "
				<< transform.getOrigin().y() << " " << transform.getOrigin().z()
				<< " ";
		ss << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	ss << "Optimized transform form camera to head:\n";
    ss << "(x, y, z) " << result.cameraTransformation.getOrigin().x()
            << " " << result.cameraTransformation.getOrigin().y() << " "
            << result.cameraTransformation.getOrigin().z() << " ";
	ss << "(q0, q1, q2, q3) "
            << result.cameraTransformation.getRotation().x() << " "
            << result.cameraTransformation.getRotation().y() << " "
            << result.cameraTransformation.getRotation().z() << " "
            << result.cameraTransformation.getRotation().w() << "\n";

	for (map<string, tf::Transform>::iterator it =
			result.jointTransformations.begin();
			it != result.jointTransformations.end(); it++) {
		ss << "Optimized transform for joint " << it->first << ":\n";
		ss << "(x, y, z) " << it->second.getOrigin().x() << " "
				<< it->second.getOrigin().y() << " "
				<< it->second.getOrigin().z() << " ";
		double r, p, y;
		tf::Matrix3x3(it->second.getRotation()).getRPY(r, p, y);
		ss << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	ss << "Optimized camera intrinsics:\n";
	ss << "(fx,fy) " << result.cameraInfo.K[K_FX_IDX] << " "
			<< result.cameraInfo.K[K_FY_IDX] << " ";
	ss << "(cx,cy) " << result.cameraInfo.K[K_CX_IDX] << " "
			<< result.cameraInfo.K[K_CY_IDX] << "\n";
	ss << "D: " << result.cameraInfo.D[0] << ", " << result.cameraInfo.D[1]
			<< ", " << result.cameraInfo.D[2] << ", " << result.cameraInfo.D[3]
			<< ", " << result.cameraInfo.D[4] << "\n";
	cout << ss.str() << endl;
	stringstream filename;
	filename << this->folderName << "/optimization_result.txt";
	ofstream ofs(filename.str().c_str());
	if (!ofs.good()) {
		cout << "Could not write results to " << filename.str() << endl;
	} else {
		ofs << ss.str();
		ofs.close();
	}
}

IntermediateResultsCsvWriter::IntermediateResultsCsvWriter(string delimiter) :
		delimiter(delimiter) {
}

IntermediateResultsCsvWriter::~IntermediateResultsCsvWriter() {
}

bool IntermediateResultsCsvWriter::writeToCsv(
		vector<CalibrationState>& intermediateStates,
		string filename) const {
	if (intermediateStates.size() < 2)
		return false;

	ofstream ofs(filename.c_str());
	if (!ofs.good())
		return false;

	if (!writeHeader(intermediateStates[1], ofs))
		return false;

	if (!writeContent(intermediateStates, ofs))
		return false;

	return true;
}

bool IntermediateResultsCsvWriter::writeHeader(
		CalibrationState& result, ostream& stream) const {
	stream << "iteration" << delimiter;

	typedef std::map<string, double>::iterator it_type;
	for (it_type iterator = result.jointOffsets.begin();
			iterator != result.jointOffsets.end(); iterator++) {
		stream << iterator->first << "_offset" << delimiter;
	}

	for (std::map<string, tf::Transform>::iterator iterator =
			result.markerTransformations.begin();
			iterator != result.markerTransformations.end(); iterator++) {
		tf::Transform transform = iterator->second.inverse(); //TODO
		string name = iterator->first;
		double r, p, y;
		tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
		stream << "marker_transform_" << name << "_tx" << delimiter;
		stream << "marker_transform_" << name << "_ty" << delimiter;
		stream << "marker_transform_" << name << "_tz" << delimiter;
		stream << "marker_transform_" << name << "_rr" << delimiter;
		stream << "marker_transform_" << name << "_rp" << delimiter;
		stream << "marker_transform_" << name << "_ry" << delimiter;
	}

	stream << "camera_transform_" << "tx" << delimiter;
	stream << "camera_transform_" << "ty" << delimiter;
	stream << "camera_transform_" << "tz" << delimiter;
	stream << "camera_transform_" << "rr" << delimiter;
	stream << "camera_transform_" << "rp" << delimiter;
	stream << "camera_transform_" << "ry" << delimiter;

	for (map<string, tf::Transform>::iterator it =
			result.jointTransformations.begin();
			it != result.jointTransformations.end(); it++) {
		stream << it->first << "_transform_" << "tx" << delimiter;
		stream << it->first << "_transform_" << "ty" << delimiter;
		stream << it->first << "_transform_" << "tz" << delimiter;
		stream << it->first << "_transform_" << "rr" << delimiter;
		stream << it->first << "_transform_" << "rp" << delimiter;
		stream << it->first << "_transform_" << "ry" << delimiter;
	}

	stream << "camera_fx" << delimiter;
	stream << "camera_fy" << delimiter;
	stream << "camera_cx" << delimiter;
	stream << "camera_cy" << delimiter;
	stream << "camera_D0" << delimiter;
	stream << "camera_D1" << delimiter;
	stream << "camera_D2" << delimiter;
	stream << "camera_D3" << delimiter;
	stream << "camera_D4" << delimiter;
	stream << "\n";

	return true;
}

bool IntermediateResultsCsvWriter::writeContent(
		vector<CalibrationState>& intermediateStates,
		ostream& stream) const {
	int iteration = 0;
	for (vector<CalibrationState>::iterator statesIt =
			intermediateStates.begin(); statesIt != intermediateStates.end();
			statesIt++) {
		stream << iteration << delimiter;

		typedef std::map<string, double>::iterator it_type;
		for (it_type iterator = statesIt->jointOffsets.begin();
				iterator != statesIt->jointOffsets.end(); iterator++) {
			stream << iterator->second << delimiter;
		}

		for (std::map<string, tf::Transform>::iterator iterator =
				statesIt->markerTransformations.begin();
				iterator != statesIt->markerTransformations.end(); iterator++) {
			tf::Transform transform = iterator->second.inverse(); //TODO
			string name = iterator->first;
			double r, p, y;
			tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
			stream << transform.getOrigin().x() << delimiter
					<< transform.getOrigin().y() << delimiter
					<< transform.getOrigin().z() << delimiter;
			stream << r << delimiter << p << delimiter << y << delimiter;
		}

		double r, p, y;
        tf::Matrix3x3(statesIt->cameraTransformation.getRotation()).getRPY(
				r, p, y);
        stream << statesIt->cameraTransformation.getOrigin().x()
				<< delimiter
                << statesIt->cameraTransformation.getOrigin().y()
				<< delimiter
                << statesIt->cameraTransformation.getOrigin().z()
				<< delimiter;
		stream << r << delimiter << p << delimiter << y << delimiter;

		for (map<string, tf::Transform>::iterator it =
				statesIt->jointTransformations.begin();
				it != statesIt->jointTransformations.end(); it++) {
			stream << it->second.getOrigin().x() << delimiter
					<< it->second.getOrigin().y() << delimiter
					<< it->second.getOrigin().z() << delimiter;
			double r, p, y;
			tf::Matrix3x3(it->second.getRotation()).getRPY(r, p, y);
			stream << r << delimiter << p << delimiter << y << delimiter;
		}

		stream << statesIt->cameraInfo.K[K_FX_IDX] << delimiter
				<< statesIt->cameraInfo.K[K_FY_IDX] << delimiter;
		stream << statesIt->cameraInfo.K[K_CX_IDX] << delimiter
				<< statesIt->cameraInfo.K[K_CY_IDX] << delimiter;
		stream << statesIt->cameraInfo.D[0] << delimiter
				<< statesIt->cameraInfo.D[1] << delimiter
				<< statesIt->cameraInfo.D[2] << delimiter
				<< statesIt->cameraInfo.D[3] << delimiter
				<< statesIt->cameraInfo.D[4] << "\n";

		iteration++;
	}
	return false;
}

ValidationDataStrategy::ValidationDataStrategy() {
	nh.getParam("optimization_ids", optimizationDataIds);
}

ValidationDataStrategy::~ValidationDataStrategy() {
// nothing to do
}

ValidateOnAllStrategy::ValidateOnAllStrategy() {
// nothing to do
}

ValidateOnAllStrategy::~ValidateOnAllStrategy() {
// nothing to do
}

void ValidateOnAllStrategy::addMeasurement(
		vector<measurementData>& optimizationData,
		vector<measurementData>& validataionData, measurementData data) {
	if (std::find(optimizationDataIds.begin(), optimizationDataIds.end(),
			data.id) != optimizationDataIds.end()) {
		optimizationData.push_back(measurementData(data));
	}
	validataionData.push_back(measurementData(data));

}

void ValidateOnAllStrategy::addMeasurements(
		vector<measurementData>& optimizationData,
		vector<measurementData>& validataionData,
		vector<measurementData>& data) {
	for (vector<measurementData>::iterator it = data.begin(); it != data.end();
			it++) {
		this->addMeasurement(optimizationData, validataionData, *it);
	}
}

ValidateOnOthersStrategy::ValidateOnOthersStrategy() {
// nothing to do
}

ValidateOnOthersStrategy::~ValidateOnOthersStrategy() {
// nothing to do
}

void ValidateOnOthersStrategy::addMeasurement(
		vector<measurementData>& optimizationData,
		vector<measurementData>& validataionData, measurementData data) {
	if (std::find(optimizationDataIds.begin(), optimizationDataIds.end(),
			data.id) != optimizationDataIds.end()) {
		optimizationData.push_back(measurementData(data));
	} else {
		validataionData.push_back(measurementData(data));
	}
}

void ValidateOnOthersStrategy::addMeasurements(
		vector<measurementData>& optimizationData,
		vector<measurementData>& validataionData,
		vector<measurementData>& data) {
	for (vector<measurementData>::iterator it = data.begin(); it != data.end();
			it++) {
		this->addMeasurement(optimizationData, validataionData, *it);
	}
}

void SplitStrategy::addMeasurement(vector<measurementData>& optimizationData,
		vector<measurementData>& validataionData, measurementData data) {
	if (std::find(optimizationDataIds.begin(), optimizationDataIds.end(),
			data.id) != optimizationDataIds.end()) {
		optimizationData.push_back(measurementData(data));
	} else if (std::find(validationDataIds.begin(), validationDataIds.end(),
			data.id) != validationDataIds.end()) {
		validataionData.push_back(measurementData(data));
	}
}

void SplitStrategy::addMeasurements(vector<measurementData>& optimizationData,
		vector<measurementData>& validataionData,
		vector<measurementData>& data) {
	nh.getParam("optimization_ids", optimizationDataIds);
	nh.getParam("validation_ids", validationDataIds);
	for (vector<measurementData>::iterator it = data.begin(); it != data.end();
			it++) {
		this->addMeasurement(optimizationData, validataionData, *it);
	}
}

MeasurementsRandomizer::MeasurementsRandomizer() {
	// nothing to do
    srand(time(NULL));
}

MeasurementsRandomizer::~MeasurementsRandomizer() {
	// nothing to do
}

void MeasurementsRandomizer::randomizeMeasurements(int n,
		vector<measurementData>& data, vector<string>& chains, int xMax,
		int yMax) {
	// initialize the random function


	// initialize the counter
	map<string, int> counter;
	for (vector<string>::iterator it = chains.begin(); it != chains.end();
			it++) {
		counter[*it] = 0;
	}
    for (unsigned int i = 0; i < data.size(); i++) {
		if (counter[data[i].chain_name] < n) {
			data[i].marker_data[0] = static_cast<double>(rand())
					/ (static_cast<double>(RAND_MAX / xMax));
			data[i].marker_data[1] = static_cast<double>(rand())
					/ (static_cast<double>(RAND_MAX / yMax));
			counter[data[i].chain_name]++;
		}
	}

}

template<typename OutputIterator>
 void MeasurementsRandomizer::sample_without_replacement(OutputIterator out, int n, int min, int max)
{
  if (n < 0)
    throw std::runtime_error("negative sample size");
  if (max < min)
    throw std::runtime_error("invalid range");
  if (n > max-min+1)
    throw std::runtime_error("sample size larger than range");

  while (n>0)
  {
    double r = std::rand()/(RAND_MAX+1.0);
    if (r*(max-min+1) < n)
    {
      *out++ = min;
      --n;
    }
    ++min;
  }
}




void MeasurementsRandomizer::wbRandomizeMeasurements(int n, vector<measurementData>& data, int xMax, int yMax) {
    // initialize the random function

    // randomly sample n affected poses
    vector<int> ids(n);
    sample_without_replacement( ids.begin(), n, 0, data.size()-1 );
    ROS_INFO("sampled %zu indices", ids.size());
    // randomize marker coordinates for these poses
    for (unsigned int i = 0; i < ids.size(); i++) {
        cout << ids[i] << ",";
        data[ids[i]].marker_data[0] = static_cast<double>(rand())
                / (static_cast<double>(RAND_MAX / xMax));
        data[ids[i]].marker_data[1] = static_cast<double>(rand())
                / (static_cast<double>(RAND_MAX / yMax));
    }
    cout << endl;

    ROS_INFO("end wbRandomizeMeasurements()");

}

MeasurementsNoiseAdder::MeasurementsNoiseAdder() {
	// nothing to do
}

MeasurementsNoiseAdder::~MeasurementsNoiseAdder() {
	// nothing to do
}

void MeasurementsNoiseAdder::noisifyMeasurements(int addNoise,
		vector<measurementData>& data) {
	if (addNoise == 0)
		return;

	// initialize the random function
	srand(time(NULL));

	// add noise
	double min = -addNoise;
	double max = addNoise;
    for (unsigned int i = 0; i < data.size(); i++) {
		double randValueX = min
				+ static_cast<double>(rand())
						/ (static_cast<double>(RAND_MAX / (max - min)));
		double randValueY = min
				+ static_cast<double>(rand())
						/ (static_cast<double>(RAND_MAX / (max - min)));
		data[i].marker_data[0] += randValueX;
		data[i].marker_data[1] += randValueY;
	}
}

} /* namespace kinematic_calibration */

