/*
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */


#include <pose_generation/DetmaxPoseSelectionStrategy.h>
#include <exception>
#include <stdexcept>
#include <cstdlib>
#include <time.h>
#include <omp.h>
#include <cmath>
#include <iterator>


namespace kinematic_calibration {



template<typename OutputIterator>
 void sample(OutputIterator out, int n, int min, int max)
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


DetmaxPoseSelectionStrategy::DetmaxPoseSelectionStrategy(unsigned int numOfPoses)
    : numOfPoses(numOfPoses) {
  srand(time(NULL));
  ros::NodeHandle nhPrivate("~"); // private node handle
  // TODO: Automatic extraction from KDL tree (watch out with mimic joints)
  nhPrivate.getParam("dofs", dofs);
  nhPrivate.param("detmax_iterations", numIterations, 10);
  ROS_INFO("Using %d detmax iterations to sample %d poses", numIterations, numOfPoses);
  //srand(1337);
}


vector<vector<MeasurementPoseSet::PoolKeyType> > DetmaxPoseSelectionStrategy::getRandomIndices(int n, const CHAINTOPOSEID_TYPE & chainNameToPoseID){
    int ndof = 0;
    ROS_INFO("Sampling poses using DOF distribution");
    vector<string> chainNames;
    for(CHAINTOPOSEID_TYPE::const_iterator it = chainNameToPoseID.begin(); it != chainNameToPoseID.end(); ++it)
    {
        //cout << "chain " << it->first << " has " << it->second.size() << " poses"<<endl;
        chainNames.push_back(it->first);
        if (dofs.count(it->first) < 1)
            throw std::runtime_error("DOF " + it->first + " not found in DOFS map");
        ndof += dofs[it->first]; // count total dofs
        ROS_INFO("Chain %s has %d optimizable dofs", it->first.c_str(), dofs[it->first]);
    }

    //unsigned int poolSize = initialPoseSet->getPoolSize();
    vector< vector<MeasurementPoseSet::PoolKeyType> > indices(n);
    for (int i =0; i < n; ++i ) // prepare n vectors with numOfPoses samples
    {
        indices[i].reserve(numOfPoses);
        for(CHAINTOPOSEID_TYPE::const_iterator it = chainNameToPoseID.begin(); it != chainNameToPoseID.end(); ++it)
        {
            double ratio = double(dofs[it->first])/ndof;
            int num_poses = std::floor(ratio*numOfPoses); // sample num_poses poses from the current chain
            vector<int> sampledNumbers(num_poses, 0);
            sample(sampledNumbers.begin(), num_poses, 0, it->second.size() - 1);
            for (int j =0; j < num_poses; ++j)
                indices[i].push_back( it->second[ sampledNumbers[j] ] );
        }
        // because of the floor operator in num_poses, it might happen that we did not sample numOfPoses samples but fewer, so we add more here
        while( indices[i].size() < numOfPoses)
        {
            // sample a chain
             int r = std::rand() % chainNames.size();
             string chainName = chainNames[r];
             // sample a pose from this chain
             int rr = std::rand() % chainNameToPoseID.at(chainName).size();
             string sampledPoseID = chainNameToPoseID.at(chainName).at(rr);
             // push to the indices
             indices[i].push_back(sampledPoseID);
        }

    }
    return indices;
}

boost::shared_ptr<MeasurementPoseSet> DetmaxPoseSelectionStrategy::getOptimalPoseSet(
                boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
                boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
      ros::WallTime start = ros::WallTime::now();


      // precompute all partial derivatives (for OMP)
      Eigen::MatrixXd mat;
      for ( map<MeasurementPoseSet::PoolKeyType, MeasurementPose>::iterator it = initialPoseSet->getPosePool()->begin(); it != initialPoseSet->getPosePool()->end(); ++it)
      {
        it->second.getPartialDerivatives( initialPoseSet->getCalibrationState(), mat);
      }
      // sort poses by chainName
      map<string, vector<string> > chainNameToPoseID;
      vector<MeasurementPose> allPoses = initialPoseSet->getAllMeasurementPoses();
      for (vector<MeasurementPose>::const_iterator it = allPoses.begin(); it != allPoses.end(); ++it)
      {
          string chainName = it->getKinematicChainName();
          string id = it->getID();
          chainNameToPoseID[chainName].push_back(id);
      }
      // randomly sample pose ids according to the DOF distribution
      vector<vector<MeasurementPoseSet::PoolKeyType> > indices = getRandomIndices(numIterations, chainNameToPoseID);

      // now starts the actual improvement and evaluation of the randomly sampled pose ids
      ExchangePoseSelectionStrategy exStrategy;
      vector<MeasurementPoseSet::PoolKeyType> bestIndices;
      double bestIndexValue = -1;
#pragma omp parallel num_threads(1)
      {
        vector<MeasurementPoseSet::PoolKeyType> priv_bestIndices;
        double priv_bestIndexValue = -1;
        boost::shared_ptr<MeasurementPoseSet> priv_poseSet(new MeasurementPoseSet(*initialPoseSet));
#pragma omp for
        for(unsigned int j =0; j < indices.size(); ++j){
          cout << "random vector [";
          //sample(indices.begin(), numOfPoses, 0, pool->size()-1);
          for (unsigned int i =0; i < indices[j].size(); ++i)
            cout << indices[j][i]<<",";
          cout << "] " ;
          priv_poseSet->setActivePoses(indices[j]);
          double curIndexValue, curIndexValue2;

          priv_poseSet = exStrategy.getOptimalPoseSet(priv_poseSet, observabilityIndex,
                                                        curIndexValue);

          observabilityIndex->calculateIndex(*priv_poseSet, curIndexValue2);
          cout << "has index: " << curIndexValue << endl;
          if (fabs(curIndexValue - curIndexValue2) > 1e-6)
                      ROS_ERROR("Something is wrong: %f vs %f", curIndexValue, curIndexValue2);
          if (curIndexValue > priv_bestIndexValue){
            priv_bestIndexValue = curIndexValue;
            priv_bestIndices = priv_poseSet->getActivePoses();
          }
        }
#pragma omp flush(bestIndices, bestIndexValue)
          if (priv_bestIndexValue > bestIndexValue) {
#pragma omp critical
            if (priv_bestIndexValue > bestIndexValue){
              bestIndexValue = priv_bestIndexValue;
              bestIndices  = priv_bestIndices;
              /*
              observabilityIndex->calculateIndex(*priv_poseSet, curIndexValue2);
              ROS_INFO( "Found a new best set with index %f (%f) for poses:",curIndexValue, curIndexValue2);
              for (int i =0; i < bestIndices.size(); ++i)
                cout << bestIndices[i]<<",";
              cout << "] " ;
              */
            }
          }

          // if( (ros::WallTime::now() - start).toSec() > 60*1)
          //   break;
          //if (ros::WallTime::now() - start).toSec() >
      }

      boost::shared_ptr<MeasurementPoseSet> resultSet(new MeasurementPoseSet(*initialPoseSet));
      resultSet->setActivePoses(bestIndices);
      double curIndexValue;
      observabilityIndex->calculateIndex(*resultSet, curIndexValue);
      ROS_INFO("SamplingPoseSelectionStrategy finished with best index %f (%f) for a set of size %zu", bestIndexValue, curIndexValue, bestIndices.size());
      if (fabs(curIndexValue - bestIndexValue) > 1e-6)
      {
         throw std::runtime_error("Difference between curIndexValue and bestIndexValue!");
      }

      ROS_INFO("Computation took %f s", (ros::WallTime::now()-start).toSec());

      index = bestIndexValue;
      return resultSet;
}

}
