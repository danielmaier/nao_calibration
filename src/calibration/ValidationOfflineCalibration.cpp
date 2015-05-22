/*
 *  Kinematic Calibration: A calibration framework for the humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *      Stefan Wrobel 2014, University of Freiburg
 */


#include <calibration/ValidationOfflineCalibration.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <fstream>

namespace kinematic_calibration
{
using namespace boost;
using namespace ros;
using namespace std;

ValidationOfflineCalibration::ValidationOfflineCalibration() : m_splitID(1)
{
    m_nhPrivate.param("num_of_splits", m_num_of_splits, m_num_of_splits);
    this->initialize();
    ROS_INFO("New ValidationOfflineCalibration created");


}

void ValidationOfflineCalibration::initPoseSource()
{
    std::vector<string> filepaths;
    m_nhPrivate.getParam("pose_files", filepaths);
    if (filepaths.empty())
    {
        ROS_ERROR("You need to specify at least one filepath for the poses");
    }
    m_poseSource.reset(new ValidationPoseSource(m_num_of_splits, m_nhPrivate, true));
    m_poseSource->addPosesFromBagFile(filepaths);
}



ValidationOfflineCalibration::~ValidationOfflineCalibration()
{

}

void ValidationOfflineCalibration::validate()
{

    std::string filePrefix, dirPrefix;
    m_nhPrivate.getParam("file_prefix", filePrefix);
    m_nhPrivate.getParam("directory_prefix", dirPrefix);


    for (int i = 1; i <= m_num_of_splits; i++)
      {
        // reset the result pose set, state and intermediate results for each split
        initPoseSet();
        m_optimalPoses.clear();
        m_intermediateIndices.clear();
        m_optimizationInterface->clearValidationData();

        // set current partition as validation set
        m_poseSource->setCurrentValidationPartition(i);
        m_splitID = i;


        // "observe" all poses
        //vector<MeasurementPose> poses = m_resultSet->getAllMeasurementPoses();

        //observe(poses); // to fill m_observationData

        // get corresponding ids for all chains
        map<string, map<int, PoseSourcePool > >::iterator it;
        for(it = m_poseSource->getSplits().begin();
            it != m_poseSource->getSplits().end(); ++it)
        {
          cout << "validation ids for chain " << it->first << endl;
          map<int, PoseSourcePool > allSplits = it->second;
          PoseSourcePool psp = allSplits[i];
          KinematicChainType curChain = m_kinematicChains[it->first];
          //vector<sensor_msgs::JointState> js = psp.getJointStates();
          //vector < string > ids;
          vector<MeasurementPose> poses ;
          for (PoseSourcePool::const_iterator poolIt = psp.begin(); poolIt != psp.end(); ++poolIt)
          {
           //   ids.push_back(poolIt->first);
              poses.push_back(MeasurementPose(*curChain, poolIt->second, poolIt->first));
          }
          // observe the validation poses
          observe(poses);

          for (vector<MeasurementPose>::iterator poseIt = poses.begin(); poseIt != poses.end(); ++poseIt)
          {
              measurementDataPtr m(new measurementData());
              if(!createMeasurementDataFromMeasurementPose(*poseIt, *m))
              {
                  ROS_ERROR("Could not create a measurementData for id %s. Optimization will fail", poseIt->getID().c_str());
                  throw std::runtime_error("Could not create measurementData");
              }
              cout << "\"" << poseIt->getID() << "\", ";
              m_optimizationInterface->addValidationData(m);
          }
          cout << endl;

        }

        /*
        // set the validation data for the validationNode (i.e. optimizer)
        for(vector<measurementData>::iterator it = m_optimizationNode->getMeasurementPool().begin(); it != m_optimizationNode->getMeasurementPool().end(); ++it)
        {
          if (std::find(validationIDs.begin(), validationIDs.end(),
                               it->id) != validationIDs.end()) {
            m_optimizationNode->getValidationData().push_back(measurementData(*it));
          }
        } // tmp
        */

        // ////////////////////////////////////////
        // perform pose selection and optimization
        calibrate(); // this fills m_optimialPoses and m_intermediateIndices
        // ///////////////////////////////////////

        if (m_optimalPoses.count(m_maximumNumberOfPoses) == 0)
        {
            ROS_WARN("Could not find a pose set for final number of poses!");
            throw std::runtime_error("Could not find a pose set for final number of poses");
        }
        boost::shared_ptr < MeasurementPoseSet > poseSet = m_optimalPoses[m_maximumNumberOfPoses];
                //m_optimizationNode.getOptimalPoseSetAndValidate(validationIds, i); // this calls m_poseSource.getPoses() which collects data and generates the measurement (optimization) set
        vector < string > ids = m_poseSource->getPoseIds(poseSet->getPoses());
        cout << "Optimized pose ids: " << endl;
        for (unsigned int j = 0; j < ids.size(); j++)
        {
          cout << "\"" << ids[j] << "\", ";
        }
        cout << endl;
        ids = m_poseSource->getPoseIds(poseSet->getUnusedPoses());
        cout << "Unused pose ids: " << endl;
        for (unsigned int j = 0; j < ids.size(); j++)
        {
          cout << "\"" << ids[j] << "\", ";
        }
        cout << endl;
        //map<int, shared_ptr<MeasurementPoseSet> > sets = node.getIntermediatePoseSets();
        cout << "writing to " << dirPrefix << "/" << filePrefix << endl;
        for (map<int, boost::shared_ptr<MeasurementPoseSet> >::iterator it = m_optimalPoses.begin(); it != m_optimalPoses.end(); it++)
        {
          stringstream ss;
          ss << dirPrefix << "/" << filePrefix << "/" << "all_except_" << i;
          mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
          ss << "/" << filePrefix << "_" << it->second->getNumberOfPoses();
          ofstream ofs(ss.str().c_str());
          if (!ofs.good())
          {
            cout << "Could not write the file  " << ss.str() << endl;
            break;
          }
          vector < string > ids = m_poseSource->getPoseIds(it->second->getPoses());
          for (unsigned int j = 0; j < ids.size(); j++)
          {
            ofs << "\"" << ids[j] << "\", ";
          }
          ofs.close();
        }

        stringstream indicesFilenameStream;
        indicesFilenameStream << dirPrefix << "/" << filePrefix << "/" << "all_except_" << i;
        indicesFilenameStream << "/" << "indices_" << filePrefix << "_" << m_optimalPoses.size() << ".csv";
        ofstream indicesOfs(indicesFilenameStream.str().c_str());
        //map<int, double> indices = node.getIntermediateIndices();
        if (!indicesOfs.good())
        {
          cout << "Could not write the file  " << indicesFilenameStream.str() << endl;
        }
        indicesOfs << "NUMPOSES\tINDEX\n";
        for (map<int, double>::iterator it = m_intermediateIndices.begin(); it != m_intermediateIndices.end(); it++)
        {
          stringstream ss;
          ss << it->first << "\t" << it->second << "\n";
          indicesOfs << ss.str();
        }
        indicesOfs.close();
    }
}


int ValidationOfflineCalibration::observe(const vector<string> & poseIds)
{
    ROS_INFO("observing %zu poses", poseIds.size());
    int numNewObservations = 0;
    for( vector<string>::const_iterator it = poseIds.begin(); it != poseIds.end(); ++it)
    {
        string id = *it;
        if ( m_poseIDToObservation.count(id) > 0)
            ROS_WARN("Pose %s already observed! Overwriting... ", id.c_str());

        Observation obs;
        m_poseSource->getObservationForId(id, obs);
        m_poseIDToObservation[id] = obs;
        numNewObservations++;
    }
    return numNewObservations;
}

int ValidationOfflineCalibration::observe(const vector<MeasurementPose> & poses)
{

    vector<string> ids;
    ids.reserve(poses.size());
    for( vector<MeasurementPose>::const_iterator it = poses.begin(); it != poses.end(); ++it)
    {
        string id = it->getID();
        ids.push_back(id);
    }
    return observe(ids);

}

int ValidationOfflineCalibration::observe()
{
     vector<MeasurementPose> poses = m_resultSet->getActiveMeasurementPoses();
     return observe(poses);

}

void ValidationOfflineCalibration::selectAndOptimize(boost::shared_ptr<PoseSelectionStrategy> strategy)
{
    ros::Time t0 = ros::Time::now();
    double indexValue;
    m_resultSet = strategy->getOptimalPoseSet(m_resultSet, m_observabilityIndex, indexValue);
    ROS_INFO("Pose selection took %f s", (ros::Time::now() - t0).toSec());

    // observe the selected pose set
    ROS_INFO("Observing %d poses ", m_resultSet->getNumberOfPoses());

    // observe/load poses from the source
    observe();

    // optimize from current result set
    t0 = ros::Time::now();
    if(!optimize())
        ROS_ERROR("Could not optimize the selected poses");
    ROS_INFO("Optimization took %f s for %d poses", (ros::Time::now() - t0).toSec(), m_resultSet->getNumberOfPoses());


    int numActivePoses = m_resultSet->getNumberOfPoses();
    this->m_optimalPoses[numActivePoses] = m_resultSet;
    this->m_intermediateIndices[numActivePoses] = indexValue;
}



void ValidationOfflineCalibration::initOptimization()
{
    ROS_INFO("ValidationOfflineCalibration::initOptimization()");
    CalibrationContext* context = new RosCalibContext(m_nhPrivate);
    m_optimizationInterface.reset(new ValidationNode(context));
    //m_optimizationNode->initialize();
}

bool ValidationOfflineCalibration::optimize()
{
    // get ValidationNode pointer
    boost::shared_ptr<ValidationNode> validationNode = boost::static_pointer_cast<ValidationNode>(m_optimizationInterface); // TODO: This is a bit ugly

    // prepare file system for output
    std::string file_prefix;
    m_nhPrivate.getParam("file_prefix", file_prefix);
    string splitstr = boost::lexical_cast<string>(m_splitID);

    //validationNode->folderName = "/home/maierd/kin_calib_experiments/results/" + file_prefix + "/all_except_" + splitstr + "/" + file_prefix + "_" + boost::lexical_cast<string>(m_optimizationNode.getOptimizationData().size());

    validationNode->folderName = "/home/maierd/kin_calib_experiments/results/" + file_prefix + "/all_except_" + splitstr + "/" + file_prefix + "_" + boost::lexical_cast<string>(m_resultSet->getNumberOfPoses()) ;
    cout << "\nWriting to " << validationNode->folderName << endl;
    boost::filesystem::path dirpath(validationNode->folderName);
    try
    {
        boost::filesystem::create_directories( dirpath );
    }
    catch(const boost::filesystem::filesystem_error& e)
    {
        cerr << e.what() << endl;
        cerr << e.code() << endl;
        throw std::runtime_error("Unable to create destination directory " + validationNode->folderName );
    }
    // add noise to optimization data
    ROS_INFO("Setting Modifying data");
    validationNode->setModifyData(true);
    //ROS_INFO("Starting optimize...");

    // actual call to OptimizationInterface is in here
    ros::Time t0=ros::Time::now();
    //CalibrationState optimizedState = CalibrationFramework::optimize();
    if(!CalibrationFramework::optimize())
        return false;
    ROS_INFO("Optimization took %f s", (ros::Time::now() - t0).toSec());


    // after optimization, verify
    ROS_INFO("Not Printing results");
    validationNode->printResult();
    ROS_INFO("Starting validation...");
    double errorsum = validationNode->validate();
    ROS_INFO("Errorsum  is %f", errorsum);
    ROS_INFO("Writing calibration results");
    validationNode->writeCalibrationResults();

    return true;

}

}

int main(int argc, char** argv) {
    // initialize the node

    //Eigen::initParallel();
    //Eigen::internal::setNbThreads(0);
    //ros::Time::init();
    //nodeName << ros::Time::now().nsec;
    ros::init(argc, argv, "validationOfflineCalibration" );

    kinematic_calibration::ValidationOfflineCalibration node;
    node.validate();

    ROS_INFO("Validation finished");
    return 0;
}


