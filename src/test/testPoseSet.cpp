#include <iostream>
#include <common/PoseSet.h>



using namespace boost;
using namespace std;
using namespace kinematic_calibration;

/*
vector<string> getActivePoseIDs(const MeasurementPoseSet & set)
{
    vector<string> poseIds;
    vector<int> activePoses = set.getActivePoses();
    boost::shared_ptr< map<int, MeasurementPose> > pool = set.getPosePool(); // dangerous
    for (vector<int>::iterator it = activePoses.begin(); it != activePoses.end(); ++it)
    {
       if (pool->count(*it) < 1)
            throw std::runtime_error("This should not be possible: activePose index not found in pool");
        MeasurementPose & pose = (*pool)[*it];
        //cout << pose.getID() << endl;
        poseIds.push_back(pose.getID());
    }
    return poseIds;

}



vector<string> getPoseIDs(const MeasurementPoseSet & set)
{
    vector<string> poseIds;
    vector<int> activePoses = set.getActivePoses();
    map<int, MeasurementPose>::iterator  poolIt = set.getPosePool()->begin(); // dangerous
    for (; poolIt != set.getPosePool()->end(); ++poolIt)
    {
        bool activePose = false;
        if ( activePoses.end() != std::find(activePoses.begin(), activePoses.end(), poolIt->first ))
        {
            activePose = true;
        }

        MeasurementPose & pose = poolIt->second;
        //cout << pose.getID() << endl;
        if (!activePose)
            poseIds.push_back("["+pose.getID()+"]");
        else
            poseIds.push_back(pose.getID());
    }
    return poseIds;

}

bool compareVectorOrString(const vector<string> & vec1, const vector<string> & vec2)
{
    bool is_equal = false;
    if(vec1.size() == vec2.size())
      is_equal = std::equal(vec1.begin(), vec1.end(), vec2.begin());
    return is_equal;

}

bool testPoseSet()
{
    CalibrationState state; // empty state..
    MeasurementPoseSet set(state);
    KinematicChain kinematicChain; // dummy Chain
    sensor_msgs::JointState js1,js2,js3,js4,js5,js6,js7; // dummy JS;
    js1.header.stamp.fromSec(17);
    js2.header.stamp.fromSec(8823);
    js3.header.stamp.fromSec(9.12);
    js4.header.stamp.fromSec(19);
    js5.header.stamp.fromSec(15.0);
    js6.header.stamp.fromSec(153.7);
    js7.header.stamp.fromSec(1232.3);

    MeasurementPose pose1(kinematicChain,js1,"pose1");
    MeasurementPose pose2(kinematicChain,js2,"pose2");
    MeasurementPose pose3(kinematicChain,js3,"pose3");
    MeasurementPose pose4(kinematicChain,js4,"pose4");
    MeasurementPose pose5(kinematicChain,js5,"pose5");
    MeasurementPose pose6(kinematicChain,js6,"pose6");
    MeasurementPose pose7(kinematicChain,js7,"pose7");


    set.addMeasurementPose(pose1);
    vector<MeasurementPose> poses;
    poses.push_back(pose2);
    poses.push_back(pose3);
    poses.push_back(pose4);
    set.addActiveMeasurementPoses(poses);

    // initially [pose1], pose2, pose3, pose4
    if(!compareVectorOrString({ "[pose1]","pose2", "pose3", "pose4" }, getPoseIDs(set) ))
        return false;

    // delete pose3
    set.deleteMeasurementPose("pose3");
    if(!compareVectorOrString({ "[pose1]","pose2", "pose4" }, getPoseIDs(set) ))
        return false;

    set.addMeasurementPose(pose5);
    if(!compareVectorOrString({ "[pose1]","pose2", "pose4", "[pose5]"}, getPoseIDs(set) ))
        return false;
    poses.clear();
    poses.push_back(pose6);
    set.addActiveMeasurementPoses(poses);
    if(!compareVectorOrString({ "[pose1]", "pose2", "pose4", "[pose5]", "pose6" }, getPoseIDs(set) ))
        return false;
    set.deleteMeasurementPose("pose2");
    if(!compareVectorOrString({ "[pose1]", "pose4", "[pose5]", "pose6" }, getPoseIDs(set) ))
        return false;
    set.deleteMeasurementPose("pose1");
    poses.clear();
    poses.push_back(pose1);
    set.addActiveMeasurementPoses(poses);
    if(!compareVectorOrString({  "pose4", "[pose5]", "pose6", "pose1" }, getPoseIDs(set) ))
        return false;
    set.addMeasurementPose(pose7);
    vector<MeasurementPose> allPoses = set.getAllMeasurementPoses();
    for(int i =0; i < allPoses.size(); ++i)
        set.setActive(allPoses[i]);

    // we should have pose4, pose5, pose6, pose1
    if(!compareVectorOrString({ "pose4", "pose5", "pose6", "pose1", "pose7" }, getPoseIDs(set) ))
        return false;
    set.deleteMeasurementPose("pose6");
    if(!compareVectorOrString({ "pose4", "pose5", "pose1", "pose7" }, getPoseIDs(set) ))
        return false;




    return true;


}

int main(int argc, char ** argv)
{
    cout << "Testing PoseSet" << endl;
    if (!testPoseSet())
    {
        cerr << "PoseSet test failed" << endl;
        return 1;
    }

    cout << "PoseSet [ OK ] "<<endl;
    return 0;

}
*/
