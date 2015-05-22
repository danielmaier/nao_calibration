/*
 * PoseSelectionStrategy.h
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#ifndef POSESELECTIONSTRATEGY_H_
#define POSESELECTIONSTRATEGY_H_

#include <common/PoseSet.h>
#include "ObservabilityIndex.h"

//using namespace boost;
//using namespace std;

namespace kinematic_calibration {

template<typename OutputIterator>
 void sample(OutputIterator out, int n, int min, int max);



class PoseSelectionStrategy {
public:
	PoseSelectionStrategy() {
	}
	virtual ~PoseSelectionStrategy() {
	}


	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
    virtual boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(
            boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
            boost::shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index) = 0;
};

//class SAPoseSelectionStrategy: public PoseSelectionStrategy{
//public:
//  SAPoseSelectionStrategy(int numOfPoses);
//  virtual ~SAPoseSelectionStrategy() {
//  }
//  boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
//                          boost::shared_ptr<ObservabilityIndex> observabilityIndex,
//                          double& index);
//protected:
//  int numOfPoses;
//};

//class SamplingPoseSelectionStrategy: public PoseSelectionStrategy{
//public:
//  SamplingPoseSelectionStrategy(int numOfPoses);
//  virtual ~SamplingPoseSelectionStrategy() {
//  }
//  boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
//                          boost::shared_ptr<ObservabilityIndex> observabilityIndex,
//                          double& index);
//protected:
//  int numOfPoses;
//};


class DecrementalPoseSelectionStrategy: public PoseSelectionStrategy {
public:
        DecrementalPoseSelectionStrategy(unsigned int numOfPoses);
        virtual ~DecrementalPoseSelectionStrategy() {
        }

        /**
         * Determines the optimal pose set and returns it.
         * @return The optimal pose set.
         */
        boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
                        boost::shared_ptr<ObservabilityIndex> observabilityIndex,
                        double& index);

private:
        unsigned int numOfPoses;
};

class IncrementalPoseSelectionStrategy: public PoseSelectionStrategy {
public:
    IncrementalPoseSelectionStrategy(unsigned int numOfIncrements);
	virtual ~IncrementalPoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
    boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
            boost::shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

private:
    unsigned int numOfIncrements;
};

class RandomSuccessorPoseSelectionStrategy: public PoseSelectionStrategy {
public:
    RandomSuccessorPoseSelectionStrategy(int numOfIncrements);
    virtual ~RandomSuccessorPoseSelectionStrategy() {
    }

    /**
     * Determines the optimal pose set and returns it.
     * @return The optimal pose set.
     */
    boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
            boost::shared_ptr<ObservabilityIndex> observabilityIndex,
            double& index);

private:
    int numOfIncrements;
};


class ExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	ExchangePoseSelectionStrategy() {
	}
	virtual ~ExchangePoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
    boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
            boost::shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

    boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSetParallel(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
	                       boost::shared_ptr<ObservabilityIndex> observabilityIndex,
	                        double& index);
};

class RandomPoseSelectionStrategy: public PoseSelectionStrategy {
public:
    RandomPoseSelectionStrategy(unsigned int numOfPoses);
	virtual ~RandomPoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

private:
    unsigned int numOfPoses;
};

class ExchangeAddExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	ExchangeAddExchangePoseSelectionStrategy(int initialSize,
			int finalSize);
	virtual ~ExchangeAddExchangePoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

private:
	int initialSize;
	int finalSize;
};

class RandomExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	RandomExchangePoseSelectionStrategy(int numOfPoses);
	virtual ~RandomExchangePoseSelectionStrategy() {
	}

	/**
	 * Draws n poses randomly and improves the random set by exchanging single poses.
	 * @return The optimal pose set according to the strategy.
	 */
boost::shared_ptr<MeasurementPoseSet> getOptimalPoseSet(boost::shared_ptr<MeasurementPoseSet> initialPoseSet,
		boost::shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);
private:
	int numOfPoses;
};

} /* namespace kinematic_calibration */

#endif /* POSESELECTIONSTRATEGY_H_ */
