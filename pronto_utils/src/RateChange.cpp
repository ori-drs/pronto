#include "pronto_utils/RateChange.hpp"
#include <iostream>

namespace pronto {

RateChange::RateChange() {
        std::cout << "RateChange -- assuming rate of 1Hz\n";
        setDesiredPeriod_us(0,(unsigned long)1E6);
}

RateChange::RateChange(const unsigned long &period_us) {
        setDesiredPeriod_us(0, period_us);
}

void RateChange::setDesiredPeriod_us(const unsigned long long &start_u_time, const unsigned long &period_us) {
        desired_period_us = period_us;
        prev_u_time = start_u_time;
}

bool RateChange::checkNewRateTrigger(const unsigned long long &cur_u_time) {

        //std::cout << " t: " << (cur_u_time - prev_u_time);

        if ((cur_u_time - prev_u_time) >= desired_period_us) {
                // This is a rate transition trigger event
                prev_u_time = cur_u_time;

                return true;
        }

        return false;
}

bool RateChange::genericRateChange(const unsigned long long &uts, const Eigen::VectorXd &samples, Eigen::VectorXd &returnval) {

        bool flag = false;
        int_vals = generic_integrator.integrate(uts, samples);

        if (checkNewRateTrigger(uts)) {
                state = generic_diff.diff(uts, int_vals);
                flag = true;
        }
        returnval = state;
        return flag;
}

void RateChange::setSize(const int &s) {
        size = s;

        generic_integrator.setSize(size);
        state.resize(size);
        int_vals.resize(size);
        generic_diff.setSize(size);
}



}
