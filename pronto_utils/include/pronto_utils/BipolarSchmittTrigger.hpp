#pragma once
#include <pronto_utils/SchmittTrigger.hpp>

namespace pronto {
class BipolarSchmittTrigger {
private:
        double prev_value;
        SchmittTrigger* _trigger;
        //SchmittTrigger* _lowside;
public:
        BipolarSchmittTrigger(double lt, double ht, long low_delay, long high_delay);
        ~BipolarSchmittTrigger();
        void UpdateState(long present_time, double value);
        void Reset();
        float getState();
};
}
