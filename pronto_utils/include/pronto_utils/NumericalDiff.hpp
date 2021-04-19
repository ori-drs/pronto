#pragma once
#include <Eigen/Geometry>

namespace pronto {
// This class should be updated to support vectored signals, rather than the present single channel
// Maybe the signals should be handled in a vector, as that will separate this module from the Eignen requirement
class NumericalDiff {
private:
        Eigen::VectorXd prev_sample;
        int size;
        unsigned long long prev_time;

        bool first_pass;
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        NumericalDiff();
        void setSize(int size);

        //Eigen::VectorXd diff(const double &time, const Eigen::VectorXd &sample);

        Eigen::VectorXd diff(const unsigned long long &ts, const Eigen::VectorXd &sample);
        void diff(const unsigned long long &ts, int count, double sample[]);

};
}
