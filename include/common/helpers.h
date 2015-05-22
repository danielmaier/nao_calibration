/*
 *  Kinematic Calibration: A calibration framework for the humanoid robots
 *
 *  (c) Daniel Maier 2015, University of Freiburg
 *
 */


#ifndef HELPERS_H_
#define HELPERS_H_

#include <ostream>
#include <tf/transform_datatypes.h>

namespace kinematic_calibration {
std::ostream & operator<<(std::ostream & out, const tf::Transform & echo_transform);


std::string withoutLeadingSlash( const std::string & frame );

}

#endif /* HELPERS_H_ */
