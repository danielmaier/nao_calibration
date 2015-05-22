#ifndef MEASUREMENTMSGPOSESOURCE_H
#define MEASUREMENTMSGPOSESOURCE_H

#include <pose_generation/PoseSource.h>

namespace kinematic_calibration
{
/**
 * Subscribes to the measurement topic and collects the poses.
 */
class MeasurementMsgPoseSource: public PoseSource {
public:
    /**
     * Constructor.
     */
    MeasurementMsgPoseSource(bool auto_subscribe=true);

    /**
     * Desctructor.
     */
    virtual ~MeasurementMsgPoseSource();

    // TODO: Probably need to change something here for multiple kinematic Chains
    virtual void getPoses(const KinematicChain& kinematicChain,
            vector<MeasurementPose>& poses);

    virtual bool getPoseById(const std::string & id, MeasurementPose & pose);

    virtual void getPosesByIds(const KinematicChain& kinematicChain,
            const vector<string>& ids, vector<MeasurementPose>& poses);

    virtual void subscribe();
    /**
     * For the given joint states, returns the corresponding pose ids.
     * @param jointStates The given joint states.
     * @return The corresponding pose ids.
     */
    vector<string> getPoseIds(vector<sensor_msgs::JointState> jointStates);

protected:
    /**
     * Callback method for measurement messages.
     * @param[in] msg Incoming measurement message.
     */
    virtual void measurementCb(const measurementDataConstPtr& msg);

    /**
     * Calls all pending callbacks in the callback queue.
     */
    void collectData();

    /**
     * Stops the collecting of measurements.
     * @param request -
     * @param response -
     * @return Always true.
     */
    bool stopCollectingCallback(std_srvs::Empty::Request& request,
            std_srvs::Empty::Response& response);

    /**
     * Collected poses. Mapping from a chain name to a vector of JointStates
     */
    //map<string, vector<sensor_msgs::JointState> > poses;
    map<string, PoseSourcePool > poses;


    map<ros::Time, string> ids;

    /**
     * NodeHandle instance.
     */
    NodeHandle nh;

private:
    /**
     * Subscriber for measurement messages.
     */
    Subscriber measurementSubscriber;

    /**
     * Topic of the measurements.
     */
    string topic;

    /**
     * Class private callback queue.
     */
    CallbackQueue callbackQueue;

    /**
     * Service to stop collecting measurement messages.
     */
    ServiceServer msgService;

    /**
     * Flag that indicates whether data is being collected.
     */
    bool collectingData;
};

} // end namespace

#endif // MEASUREMENTMSGPOSESOURCE_H
