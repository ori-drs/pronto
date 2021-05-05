#pragma once
namespace pronto {
class ExpireTimer {
private:
        unsigned long desired_uper;
        unsigned long uperiod;
        unsigned long long previous_uts;
        bool firstpass;

public:
        ExpireTimer();

        void setDesiredPeriod_us(const unsigned long &uts);
        void reset();

        bool processSample(const unsigned long long &uper);
        bool getState();
        unsigned long getRemainingTime_us();
};
}
