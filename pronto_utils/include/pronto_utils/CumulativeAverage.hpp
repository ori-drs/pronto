#pragma once
#include <Eigen/Geometry>

namespace pronto {
class CumulativeAverage {
private:
  Eigen::VectorXd CA;
  int size;
  int elementcounter;

  void reset();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CumulativeAverage();

  void setSize(const int &s);
  Eigen::VectorXd processSamples(const Eigen::VectorXd &val);
  Eigen::VectorXd getCA();

};
}
