#pragma once
#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>
namespace pronto {
class DistributedDiff {
private:
        Eigen::VectorXd timespans;
        Eigen::VectorXd weights;
        int individual_diffs;
        int size;
        int hist_len;
        bool sizeset;
        bool hist_len_set;
        unsigned long max_hist_utime;
        unsigned long period;
        int firstpasses;

        Eigen::VectorXd cum_sum;
        Eigen::VectorXd cum_sum_buf;

        Eigen::VectorXd alldiffs;

        // This could be built into a single struct if you bored
        boost::circular_buffer<Eigen::VectorXd*> _state_hist;
        boost::circular_buffer<unsigned long long> utimes;

        void addDataToBuffer(const unsigned long long &u_ts, const Eigen::VectorXd &samples);
        Eigen::VectorXd searchHistElement(const unsigned long &delta_u_ts, unsigned long *actual_delta_u_ts);
        Eigen::VectorXd findDifferentialFromLatest(const unsigned long &desired_hist_ut);
        Eigen::VectorXd FindDifferentials();

public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DistributedDiff();
        ~DistributedDiff();

        bool ready();

        void setSize(int s);

        void InitializeTaps(int hist_length, const long &per, const Eigen::VectorXd &w, const Eigen::VectorXd &t);
        void ParameterFileInit(std::string path_to_weights);

        Eigen::VectorXd diff(const unsigned long long &u_ts, const Eigen::VectorXd &samples);

};
}
