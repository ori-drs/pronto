#pragma once
#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>

namespace pronto {
class MedianFilter {
private:
  boost::circular_buffer<double> data;

  int len;
  bool lengthset;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MedianFilter();
  void setLength(const int &length);
  double processSample(const double &sample);
};
}
