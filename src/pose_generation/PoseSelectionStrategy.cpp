/*
 * PoseSelectionStrategy.cpp
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/PoseSelectionStrategy.h"
#include <exception>
#include <stdexcept>
#include <cstdlib>
#include <time.h>
#include <omp.h>
#include <cmath>
#include <iterator>


namespace kinematic_calibration {





 /*

 boost::shared_ptr<MeasurementPoseSet>  DetmaxIncreasePoseSelectionStrategy::getInitialPoseSet(boost::shared_ptr<MeasurementPoseSet> poseSet, boost::shared_ptr<ObservabilityIndex> observabilityIndex)
 {
     ros::WallTime start = ros::WallTime::now();

     // precompute all partial derivatives (for OMP)
     Eigen::MatrixXd mat;
     for ( map<int, MeasurementPose>::iterator it = poseSet->getPosePool()->begin(); it != poseSet->getPosePool()->end(); ++it)
     {
       it->second.getPartialDerivatives( poseSet->getKinematicCalibrationState(), mat);
     }
     vector<vector<int> > indices = getRandomIndices(numIterations, poseSet);
     ExchangePoseSelectionStrategy exStrategy;
     //vector<int> indices(numOfPoses),
     vector<int> bestIndices;
     double bestIndexValue = -1;
     //ros::WallTime start = ros::WallTime::now();
     int count = 0;
#pragma omp parallel num_threads(1)
     {
       vector<int> priv_bestIndices;
       double priv_bestIndexValue = -1;
       boost::shared_ptr<MeasurementPoseSet> priv_poseSet(new MeasurementPoseSet(*poseSet));
#pragma omp for
       for(int j =0; j < indices.size(); ++j){
         cout << "random vector [";
         //sample(indices.begin(), numOfPoses, 0, pool->size()-1);
         for (int i =0; i < indices[j].size(); ++i)
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
             //observabilityIndex->calculateIndex(*priv_poseSet, curIndexValue2);
             //ROS_INFO( "Found a new best set with index %f (%f) for poses:",curIndexValue, curIndexValue2);
             //for (int i =0; i < bestIndices.size(); ++i)
             //  cout << bestIndices[i]<<",";
             //cout << "] " ;

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
       ROS_ERROR("This does not look right!");
     }

     ROS_INFO("Computation took %f s", (ros::WallTime::now()-start).toSec());

     index = bestIndexValue;
     return resultSet;


     //poseSet->setActivePoses(bestIndices);
 }

 boost::shared_ptr<MeasurementPoseSet>  DetmaxIncreasePoseSelectionStrategy::getImprovedPoseSet(boost::shared_ptr<MeasurementPoseSet> poseSet, boost::shared_ptr<ObservabilityIndex> observabilityIndex, double &indexValue)
 {
     // best successor
     boost::shared_ptr<MeasurementPoseSet> bestSuccessor = poseSet;
     double bestIndexValue = -1;

     // calculate the best n poses
     for (int i = 0; i < numOfPoses; i++) {
         bestIndexValue = -1;
         vector<boost::shared_ptr<MeasurementPoseSet> > successors = bestSuccessor->addPose();
         for (vector<boost::shared_ptr<MeasurementPoseSet> >::iterator it = successors.begin();
                 it != successors.end(); it++) {
             double curIndexValue;
             observabilityIndex->calculateIndex(**it, curIndexValue);
             if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
                 bestIndexValue = curIndexValue;
                 bestSuccessor = *it;
             }
         }
         ROS_INFO("Iteration: %d index value: %.20f size: %d", i, bestIndexValue,
                 bestSuccessor->getNumberOfPoses());
     }

     indexValue = bestIndexValue;
     return bestSuccessor;

 }

 bool DetmaxIncreasePoseSelectionStrategy::canBeImproved(boost::shared_ptr<MeasurementPoseSet> poseSet)
 {
     return poseSet->getNumberOfPoses() < m_num_final_poses;
 }

 vector<vector<int> > DetmaxIncreasePoseSelectionStrategy::getRandomIndices(int n, boost::shared_ptr<MeasurementPoseSet> poseSet)
 {
     boost::shared_ptr<map<int, MeasurementPose> >  pool = poseSet->getPosePool();
     vector< vector<int> > indices(n);
     if (poolIndex.size() > 1)
     {
       ROS_INFO("PoolIndex > 1. Using special random sampling in DetmaxPoseSelectionStrategy");
       int ndof = 0;
       for (map<string, int>::const_iterator it = poolIndex.begin(); it != poolIndex.end(); ++it)
       {
         ndof+=dofs[it->first];
       }
       cout << "There are a total of " << ndof << " degrees of freedom ";

       for (int i =0; i < n; ++i ) // prepare samples
       {
         indices[i].resize(numOfPoses, 0);
         vector<int>::iterator indit = indices[i].begin();
         int startidx = 0;
         int distance = 0;
         for (map<string, int>::const_iterator it = poolIndex.begin(); it != poolIndex.end(); ++it)
         {
           double ratio = double(dofs[it->first])/ndof;
           int num_poses = std::floor(ratio*numOfPoses);
           ROS_INFO("Sampling %d poses for %s from %d to %d",num_poses, it->first.c_str(), startidx, it->second-1);
           sample(indit, num_poses, startidx, it->second-1);
           std::advance(indit, num_poses);
           distance+=num_poses;
           startidx = it->second;
         }

         ROS_INFO("After sampling, there are %d poses in indices. Sampling %d more.", distance, numOfPoses-distance);
         // finally, sample remaining spots from all poses
         while ( numOfPoses-distance > 0)
         {
           int r = std::rand() % pool->size(); // between 0 and pool->size() - 1
           if (std::find(indices[i].begin(), indices[i].end(), r)!=indices[i].end())
             continue; // already contained, so try again. Otherwise add
           *indit++=r;
           distance++;
           //sample(indit, numOfPoses-distance, 0, pool->size()-1);
         }

         for(int j = 0; j < indices[i].size(); ++j)
           cout << indices[i][j] << " ";
         cout << endl;
         //throw std::runtime_error("You want it");
       }
     }
     else
     {
       for (int i =0; i < n; ++i ) // prepare samples
       {
         indices[i].resize(numOfPoses);
         sample(indices[i].begin(), numOfPoses, 0, pool->size()-1);
       }
     }
     return indices;
 }


*/





//SAPoseSelectionStrategy::SAPoseSelectionStrategy(
//                int numOfPoses) :
//                numOfPoses(numOfPoses) {
//  srand(time(NULL));
//}


//boost::shared_ptr<MeasurementPoseSet> SAPoseSelectionStrategy::getOptimalPoseSet(
//                boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
//                boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {

//  //boost::shared_ptr<MeasurementPoseSet> bestSuccessor = make_shared<MeasurementPoseSet>(
//   //   initialPoseSet->getKinematicCalibrationState());
//  boost::shared_ptr<map<int, MeasurementPose> >  pool = initialPoseSet->getPosePool();
//  vector<int> indices(numOfPoses), bestIndices;
//  //ros::WallTime start = ros::WallTime::now();
//  int count = 0;
//  // sample initial config
//  sample(indices.begin(), numOfPoses, 0, pool->size()-1); // sample Xk
//  initialPoseSet->setActivePoses(indices); // active Xk
//  bestIndices = indices; // use Xk as best config
//  double bestIndexValue; // compute initial energy = F(Xk)
//  observabilityIndex->calculateIndex(*initialPoseSet, bestIndexValue);
//  cout << "Initial vector [";
//  for (int i =0; i < indices.size(); ++i)
//    cout << indices[i]<<",";
//  cout << "] " << endl;
//  // compute average change from start
//  /*
//  int cnt =0;
//  double avg = 0.0;
//  for(int i =0; i < indices.size(); ++i)
//  {
//    for (int j = 0; j < pool->size()-1; ++j)
//    {
//      int prev_val = indices[i];
//      if (std::find(indices.begin(), indices.end(), j) != indices.end() ) // j is already in indices
//        continue;
//      indices[i] = j;
//      initialPoseSet->setActivePoses(indices);
//      double curIdx;
//      observabilityIndex->calculateIndex(*initialPoseSet, curIdx);
//      if (curIdx < bestIndexValue) // worse config
//      {
//        ++cnt;
//        double delta = (curIdx-bestIndexValue);
//        avg = avg* double(cnt-1)/cnt + delta/cnt;
//      }
//      indices[i] = prev_val; // set back
//    }
//  }
//  cout << "Initial vector after avg computation [";
//  for (int i =0; i < indices.size(); ++i)
//    cout << indices[i]<<",";
//  cout << "] " << endl;

//  cout << "avg is " << avg << endl;
//  double T = avg/(log(1/double(2*0.5 - 1))); //TODO
//  cout << "hence T0 is " << T << endl;
//  double gamma = T*log(2);
//  cout << "and gamma is " << gamma << endl;
//  */
//  double T = 10000.0; // init temperatur
//  int k = 0;
//  while (k < 100000){

//    int cur_set_idx = rand() % numOfPoses; // [0.. numOfPoses]
//    int pool_idx = rand() % pool->size(); // [0.. poolSize]
//    while ( std::find(indices.begin(), indices.end(), pool_idx) != indices.end()) { // make indices unique
//      pool_idx = rand() % pool->size();
//    }
//    int old_pool_idx = indices[cur_set_idx]; // store previous value to reconstruct to Xk
//    indices[cur_set_idx] = pool_idx; // Yk ~ Xk



//    initialPoseSet->setActivePoses(indices); // active Yk
//    double curIndexValue;
//    observabilityIndex->calculateIndex(*initialPoseSet, curIndexValue); // compute F(Yk)

//    if (curIndexValue > bestIndexValue){ // keep Yk if it is better
//      bestIndices = indices;
//      bestIndexValue = curIndexValue;
//      cout << "random vector [";
//      for (int i =0; i < indices.size(); ++i)
//        cout << indices[i]<<",";
//      cout << "] " ;
//      cout << "has index: " << curIndexValue << endl;
//    }
//    else{ //curIndexValue <= bestIndexValue // Yk is worse than Xk
//      double r = std::rand()/(RAND_MAX+1.0); //[0,1)
//      //T = gamma/log(k+2);
//      if (k % 50 == 0)
//        T *= 0.99;
//      if ( std::exp(curIndexValue - bestIndexValue)/T >= r ) // with some probability
//      {
//       indices [cur_set_idx] = old_pool_idx; // keep Xk, otherwise proceed with Yk
//      }
//      else
//      {
//        ROS_INFO("Jump! %f < %f", std::exp(curIndexValue - bestIndexValue)/T, r );
//      }
//    }
//    if (T<1e-3 )
//    {
//      ROS_WARN("Temperature is %f after %d iterations. Stopping", T, k);
//      break;
//    }
//    ++k;

//    //if (ros::WallTime::now() - start).toSec() >
//  }

//  ROS_INFO("SAPoseSelectionStrategy finished after %d iterations with best index %f for a set of size %zu", k, bestIndexValue, bestIndices.size());
//  initialPoseSet->setActivePoses(bestIndices);

//  index = bestIndexValue;
//  return initialPoseSet;
//}

//SamplingPoseSelectionStrategy::SamplingPoseSelectionStrategy(
//                int numOfPoses) :
//                numOfPoses(numOfPoses) {
//  srand(time(NULL));
//}

//boost::shared_ptr<MeasurementPoseSet> SamplingPoseSelectionStrategy::getOptimalPoseSet(
//                boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
//                boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {

//  //boost::shared_ptr<MeasurementPoseSet> bestSuccessor = make_shared<MeasurementPoseSet>(
//   //   initialPoseSet->getKinematicCalibrationState());
//  boost::shared_ptr<map<MeasurementPoseSet::POOLKEYTYPE, MeasurementPose> >  pool = initialPoseSet->getPosePool();
//  vector<MeasurementPoseSet::POOLKEYTYPE> indices(numOfPoses), bestIndices;
//  vector<MeasurementPoseSet::POOLKEYTYPE> keys = initialPoseSet->getPoseKeys();
//  double bestIndexValue = -1;
//  //ros::WallTime start = ros::WallTime::now();
//  int count = 0;
//  while (true){
//    cout << "random vector [";
//    sample(indices.begin(), numOfPoses, 0, pool->size()-1);
//    vector<MeasurementPoseSet::POOLKEYTYPE> selectedPoses;
//    selectedPoses.resize(numOfPoses);
//    for (int i =0; i < indices.size(); ++i)
//    {
//        selectedPoses[i] = keys[indices[i]];
//        cout << selectedPoses[i] <<",";
//    }
//    cout << "] " ;
//    initialPoseSet->setActivePoses(selectedPoses);
//    double curIndexValue;
//    observabilityIndex->calculateIndex(*initialPoseSet, curIndexValue);
//    cout << "has index: " << curIndexValue << endl;
//    if (curIndexValue > bestIndexValue){
//      bestIndices = selectedPoses;
//      bestIndexValue = curIndexValue;
//    }
//    if(count++ > 100000)
//      break;
//    //if (ros::WallTime::now() - start).toSec() >
//  }

//  ROS_INFO("SamplingPoseSelectionStrategy finished with best index %f for a set of size %zu", bestIndexValue, bestIndices.size());
//  initialPoseSet->setActivePoses(bestIndices);

//  index = bestIndexValue;
//  return initialPoseSet;
//}

IncrementalPoseSelectionStrategy::IncrementalPoseSelectionStrategy(unsigned int numIncrements) :
        numOfIncrements(numIncrements) {
}

boost::shared_ptr<MeasurementPoseSet> IncrementalPoseSelectionStrategy::getOptimalPoseSet(
		boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	// best successor
	boost::shared_ptr<MeasurementPoseSet> bestSuccessor = initialPoseSet;
	double bestIndexValue = -1;

	// calculate the best n poses
    for (unsigned int i = 0; i < numOfIncrements; i++) {
		bestIndexValue = -1;
		vector<boost::shared_ptr<MeasurementPoseSet> > successors = bestSuccessor->addPose();
		for (vector<boost::shared_ptr<MeasurementPoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}
		ROS_INFO("Iteration: %d index value: %.20f size: %d", i, bestIndexValue,
				bestSuccessor->getNumberOfPoses());
	}

	index = bestIndexValue;
	return bestSuccessor;
}


RandomSuccessorPoseSelectionStrategy::RandomSuccessorPoseSelectionStrategy(
        int numIncrements) :
    numOfIncrements(numIncrements) {
}

boost::shared_ptr<MeasurementPoseSet> RandomSuccessorPoseSelectionStrategy::getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet, boost::shared_ptr<ObservabilityIndex> observabilityIndex, double &index)
{
    boost::shared_ptr<MeasurementPoseSet> successor = initialPoseSet;
    unsigned int n = initialPoseSet->getNumberOfPoses()  + numOfIncrements; // final size
    // TODO: Isn't this inefficient? Why create all possible successors and then pick just one of them?
    // Better do it the other way around!
    while (successor->getNumberOfPoses() < n) {
            // create all possible successors by adding a not-selected (not active) pose to the poseSet
        vector<boost::shared_ptr<MeasurementPoseSet> > successors = successor->addPose();
        int randIdx = rand() % successors.size();
        successor = successors[randIdx];
    }
    // compute observability index for final pose set
    observabilityIndex->calculateIndex(*successor, index);
    return successor;

}


DecrementalPoseSelectionStrategy::DecrementalPoseSelectionStrategy(
                unsigned int numOfPoses) :
                numOfPoses(numOfPoses) {
}

boost::shared_ptr<MeasurementPoseSet> DecrementalPoseSelectionStrategy::getOptimalPoseSet(
                boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
                boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
        // best successor
        boost::shared_ptr<MeasurementPoseSet> bestSuccessor = initialPoseSet;
        double bestIndexValue = -1;

        // calculate the best n poses
        for (unsigned int i = 0; i < numOfPoses; i++) {
                bestIndexValue = -1;
                vector<boost::shared_ptr<MeasurementPoseSet> > successors = bestSuccessor->removePose();
                for (vector<boost::shared_ptr<MeasurementPoseSet> >::iterator it = successors.begin();
                                it != successors.end(); it++) {
                        double curIndexValue;
                        observabilityIndex->calculateIndex(**it, curIndexValue);
                        if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
                                bestIndexValue = curIndexValue;
                                bestSuccessor = *it;
                        }
                }
                ROS_INFO("Iteration: %d index value: %.20f size: %d", i, bestIndexValue,
                                bestSuccessor->getNumberOfPoses());
        }

        index = bestIndexValue;
        return bestSuccessor;
}

RandomPoseSelectionStrategy::RandomPoseSelectionStrategy(unsigned int numOfPoses) :
		numOfPoses(numOfPoses) {
	srand(time(NULL));
}

boost::shared_ptr<MeasurementPoseSet> RandomPoseSelectionStrategy::getOptimalPoseSet(
		boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	boost::shared_ptr<MeasurementPoseSet> successor = initialPoseSet;
	// TODO: Isn't this inefficient? Why create all possible successors and then pick just one of them?
	// Better do it the other way around!
	while (successor->getNumberOfPoses() < numOfPoses) {
	        // create all possible successors by adding a not-selected (not active) pose to the poseSet
		vector<boost::shared_ptr<MeasurementPoseSet> > successors = successor->addPose();
		int randIdx = rand() % successors.size();
		successor = successors[randIdx];
		// compute observability index for new pose set
		observabilityIndex->calculateIndex(*successor, index);
	}
	return successor;
}


boost::shared_ptr<MeasurementPoseSet> ExchangePoseSelectionStrategy::getOptimalPoseSetParallel(
                boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
                boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {

        // precompute all partial derivatives (needed for OMP, as different threads otherwise might try to compute them)
        Eigen::MatrixXd mat;
        for ( map<MeasurementPoseSet::PoolKeyType, MeasurementPose>::iterator it = initialPoseSet->getPosePool()->begin(); it != initialPoseSet->getPosePool()->end(); ++it)
        {
          // this is critical if called in parallel
          it->second.getPartialDerivatives( initialPoseSet->getCalibrationState(), mat);
        }
        /*
        Eigen::MatrixXd jac;
        initialPoseSet->getJacobian(jac);
        cout << "Initial Jacobian is \n" <<  jac << endl;
        */

        // best successor
        boost::shared_ptr<MeasurementPoseSet> bestSuccessor = initialPoseSet;
        double bestIndexValue = -1, oldIndexValue = -2;

        vector<boost::shared_ptr<MeasurementPoseSet> > successors;
        bool converged = false;

        while (!converged) {

                oldIndexValue = bestIndexValue;

                // (n) -> (n+1)
                bestIndexValue = -1;
                successors = bestSuccessor->addPose();
                /*
                cout << "By adding one pose, there are " << successors.size() << " potential successors" << endl;
                for (int i =0; i < successors.size(); ++i)
                {
                  Eigen::MatrixXd jac;
                  successors[i]->getJacobian(jac);
                  cout << "Jacobian of suc " << i << " is \n" <<  jac << endl;
                }
                */

#pragma omp parallel
                {
                  double priv_bestIndexValue = bestIndexValue;
                  boost::shared_ptr<MeasurementPoseSet> priv_bestSuccessor = bestSuccessor;
#pragma omp for
                  for (unsigned int i =0; i < successors.size(); ++i){
                    //sample(indices.begin(), numOfPoses, 0, pool->size()-1);
                    /*
                    vector<int> indices = successors[i]->getActivePoses();
                    cout << "current successor (" << indices.size() << ") [";
                    for (int j =0; j < indices.size(); ++j)
                      cout << indices[j]<<",";
                    cout << "] " << endl;
                    */
                    double curIndexValue;
                    observabilityIndex->calculateIndex(*successors[i], curIndexValue);

                    if ( curIndexValue > priv_bestIndexValue || priv_bestIndexValue == -1) {
                      priv_bestIndexValue = curIndexValue;
                      priv_bestSuccessor = successors[i];
                    }
#pragma omp flush(bestIndexValue, bestSuccessor)
                    if (priv_bestIndexValue > bestIndexValue){
#pragma omp critical
                      {
                        if (priv_bestIndexValue > bestIndexValue){
                          bestIndexValue = priv_bestIndexValue;
                          bestSuccessor = priv_bestSuccessor;
                        }
                      }
                    }
                  }
                }

                //ROS_INFO("Index n+1: %.20f", bestIndexValue);

                // (n+1) -> (n)'
                bestIndexValue = -1;
                successors = bestSuccessor->removePose();
#pragma omp parallel
                {
                  double priv_bestIndexValue = bestIndexValue;
                  boost::shared_ptr<MeasurementPoseSet> priv_bestSuccessor = bestSuccessor;
                //cout << "By removing one pose, there are " << successors.size() << " potential successors" << endl;
#pragma omp for
                  for (unsigned int i =0; i < successors.size(); ++i) {
                    double curIndexValue;
                    observabilityIndex->calculateIndex(*successors[i], curIndexValue);
                    if (curIndexValue > priv_bestIndexValue || priv_bestIndexValue == -1 ) {
                      priv_bestIndexValue = curIndexValue;
                      priv_bestSuccessor = successors[i];
                    }
#pragma omp flush(bestIndexValue, bestSuccessor)
                    if (priv_bestIndexValue > bestIndexValue){
#pragma omp critical
                      if (priv_bestIndexValue > bestIndexValue)
                      {
                        bestIndexValue = priv_bestIndexValue;
                        bestSuccessor = priv_bestSuccessor;
                      }
                    }
                  }
                }

                //ROS_INFO("Index n': %.20f", bestIndexValue);

                if (fabs(oldIndexValue - bestIndexValue) < 1e-30) {
                        converged = true;
                }

        }

        index = bestIndexValue;
        return bestSuccessor;
}


boost::shared_ptr<MeasurementPoseSet> ExchangePoseSelectionStrategy::getOptimalPoseSet(
		boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
  // use parallel version
  return getOptimalPoseSetParallel(initialPoseSet, observabilityIndex, index);
  // the following part is ignored
	// best successor
	boost::shared_ptr<MeasurementPoseSet> bestSuccessor = initialPoseSet;
	double bestIndexValue = -1, oldIndexValue = -2;

	vector<boost::shared_ptr<MeasurementPoseSet> > successors;
	bool converged = false;

	while (!converged) {

		oldIndexValue = bestIndexValue;

		// (n) -> (n+1)
		bestIndexValue = -1;
		successors = bestSuccessor->addPose();
		//cout << "By adding one pose, there are " << successors.size() << " potential successors" << endl;
		for (vector<boost::shared_ptr<MeasurementPoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}

		//ROS_INFO("Index n+1: %.20f", bestIndexValue);

		// (n+1) -> (n)'
		bestIndexValue = -1;
		successors = bestSuccessor->removePose();
		//cout << "By removing one pose, there are " << successors.size() << " potential successors" << endl;
		for (vector<boost::shared_ptr<MeasurementPoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}

		//ROS_INFO("Index n': %.20f", bestIndexValue);

		if (fabs(oldIndexValue - bestIndexValue) < 1e-30) {
			converged = true;
		}

	}

	index = bestIndexValue;
	return bestSuccessor;
}

ExchangeAddExchangePoseSelectionStrategy::ExchangeAddExchangePoseSelectionStrategy(
		int initialSize, int finalSize) :
		initialSize(initialSize), finalSize(finalSize) {
}

boost::shared_ptr<MeasurementPoseSet> ExchangeAddExchangePoseSelectionStrategy::getOptimalPoseSet(
		boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	boost::shared_ptr<MeasurementPoseSet> resultSet = initialPoseSet;

	ROS_INFO("Initializing pose set with random poses...");
	RandomPoseSelectionStrategy intializingStrategy(this->initialSize);
	resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);

	// optimize by adding and exchanging
	IncrementalPoseSelectionStrategy incrementalStrategy(1);
	ExchangePoseSelectionStrategy exchangeStrategy;
	ROS_INFO("Optimize pose set...");
	resultSet = exchangeStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);
	for (int i = this->initialSize; i < this->finalSize; i++) {
        ROS_INFO("Increment pose set size...");
		resultSet = incrementalStrategy.getOptimalPoseSet(resultSet,
				observabilityIndex, index);

	}
	ROS_INFO("Optimize pose set...");
	resultSet = exchangeStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);

	return resultSet;
}

RandomExchangePoseSelectionStrategy::RandomExchangePoseSelectionStrategy(
		int numOfPoses) :
		numOfPoses(numOfPoses) {
	// nothing to do!
}

boost::shared_ptr<MeasurementPoseSet> RandomExchangePoseSelectionStrategy::getOptimalPoseSet(
		boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	// get n random poses
	ROS_INFO("Get %d random pose(s)...", numOfPoses);
	RandomPoseSelectionStrategy randomStrategy(numOfPoses);
	boost::shared_ptr<MeasurementPoseSet> randomSet = randomStrategy.getOptimalPoseSet(
			initialPoseSet, observabilityIndex, index);
	ROS_INFO("Observability index is %.20f.", index);

	// improve the pose set
	ROS_INFO("Improve by exchange...");
	ExchangePoseSelectionStrategy exchangeStrategy;
	boost::shared_ptr<MeasurementPoseSet> improvedSet = exchangeStrategy.getOptimalPoseSet(
			randomSet, observabilityIndex, index);
	ROS_INFO("Improved observability index is %.20f.", index);

    return improvedSet;
}




} /* namespace kinematic_calibration */
