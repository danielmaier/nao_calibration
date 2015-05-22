#include <data_capturing/DataCapture.h>
#include <common/CalibrationContext.h>

using namespace kinematic_calibration;

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"dataCapturing");
  ros::NodeHandle m_nhPrivate("~");
  RosCalibContext context(m_nhPrivate);
  GeneralDataCapture d(context);
  ros::spin();
}
