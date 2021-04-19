#pragma once
#include <Eigen/Geometry>
#include <pronto_utils/TrapezoidalInt.hpp>
#include <pronto_utils/NumericalDiff.hpp>

namespace pronto {

class RateChange {
private:
        unsigned long long prev_u_time;
        unsigned long desired_period_us;
        int size;

        Eigen::VectorXd state;
        Eigen::VectorXd int_vals;
        TrapezoidalInt generic_integrator;
        NumericalDiff generic_diff;

        bool checkNewRateTrigger(const unsigned long long &cur_u_time);


public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RateChange();
        RateChange(const unsigned long &period_us);

        void setSize(const int &s);

        void setDesiredPeriod_us(const unsigned long long &start_u_time, const unsigned long &period_us);
        bool genericRateChange(const unsigned long long &uts, const Eigen::VectorXd &samples, Eigen::VectorXd &returnval);

};
}
