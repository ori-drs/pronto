#include "pronto_utils/CumulativeAverage.hpp"

namespace pronto {
CumulativeAverage::CumulativeAverage() {
        setSize(1);
}

void CumulativeAverage::setSize(const int &s) {
        size = s;
        CA.resize(size);
        reset();
}

void CumulativeAverage::reset() {

        CA.setZero();
        elementcounter = 0;
}

Eigen::VectorXd CumulativeAverage::processSamples(const Eigen::VectorXd &val) {
        Eigen::VectorXd alias(size);

        elementcounter++;
        alias = (val + (elementcounter-1)*CA)/(elementcounter);
        CA = alias;

        return CA;
}

Eigen::VectorXd CumulativeAverage::getCA() {
        return CA;
}
}
