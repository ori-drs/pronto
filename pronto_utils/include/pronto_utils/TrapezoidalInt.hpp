#pragma once
#include <Eigen/Geometry>
namespace pronto {
// This is a mid-point integratioin rule, not trapezoidal
class TrapezoidalInt {
private:
        Eigen::VectorXd int_dx;
        Eigen::VectorXd prev_dx;

        unsigned long long u_stime;
        int size;

        bool first_pass;
        bool size_set;


        void Init();

public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TrapezoidalInt();
        //~TrapezoidalInt();

        void setSize(const int &s);
        void setStateTo(const Eigen::VectorXd &set_val);

        Eigen::VectorXd integrate(const unsigned long long &u_ts, const Eigen::VectorXd &dx);
        Eigen::VectorXd integrate(const unsigned long long &ts, const int &num_joints, const double samples[]);

        Eigen::VectorXd getVal();
        void setVal(const Eigen::VectorXd &val);

        void reset_in_time();

};
}
