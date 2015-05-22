#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <ros/ros.h>
#include <vector>
#include <string>

class Observation{
public:
    std::vector<double> marker_data;
    std::string marker_type;
    std::string marker_frame;
    ros::Time stamp;
    std::string camera_frame;
};


#endif // OBSERVATION_H
