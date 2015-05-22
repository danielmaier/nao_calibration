#include "../../include/common/sample.h"
#include <exception>
#include <time.h>
#include <stdexcept>
#include <cstdlib>

namespace kinematic_calibration{



template<typename OutputIterator>
 void sample_without_replacement(OutputIterator out, int n, int min, int max)
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




}
