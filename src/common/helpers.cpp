/*
 *  Kinematic Calibration: A calibration framework for the humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */


#include <iostream>
#include <iomanip>
#include <common/helpers.h>

namespace kinematic_calibration {
std::ostream & operator<<(std::ostream & out, const tf::Transform & echo_transform)
{
    int old_precision = out.precision();
    out.precision(3);
    out.setf(std::ios::fixed,std::ios::floatfield);
    //out << "At time " << echo_transform.stamp_.toSec() << std::endl;
    double yaw, pitch, roll;
    echo_transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = echo_transform.getRotation();
    tf::Vector3 v = echo_transform.getOrigin();
    out << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    out << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
        << q.getZ() << ", " << q.getW() << "]" << std::endl
        << "            in RPY [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl;
    out.precision(old_precision);
    return out;
}


std::string withoutLeadingSlash( const std::string & frame )
{
    // if frame is empty or does not start with slash, return frame, otherwise recursively remove leading slash
    return ( frame.empty() || frame[0] != '/' ) ? frame : withoutLeadingSlash(frame.substr((1)));
}

}
