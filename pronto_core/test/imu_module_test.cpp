#include "pronto_core/ins_module.hpp"
#include <random>
#include <chrono>

#include <gtest/gtest.h>
using namespace pronto;

TEST(InsModule, ballisticTrajectory) {

    uint64_t history_span = 1000000; // 1 sec of history
    RBIS init_state;
    RBIM init_covariance;

    InsConfig ins_config;
    ImuMeasurement imu_measurement;

    ins_config.accel_bias_initial = Eigen::Vector3d::Zero();
    ins_config.accel_bias_recalc_at_start = false;
    ins_config.accel_bias_update_online = true;

    ins_config.cov_accel = 0.01;
    ins_config.cov_accel_bias = 0.001;
    ins_config.cov_gyro = 0.01;
    ins_config.cov_gyro_bias = 0.001;

    ins_config.dt = 0.001; // integration step is 1kHz
    ins_config.gyro_bias_initial = Eigen::Vector3d::Zero();
    ins_config.gyro_bias_recalc_at_start = false;
    ins_config.gyro_bias_update_online = true;

    // setting the covariance matrix of the filter (using the same values for biases)
    double sigma_Delta_xy_init = 0; //bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_xy");
    double sigma_Delta_z_init = 0; //bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_z");
    double sigma_chi_xy_init = 0; // bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_xy"));
    double sigma_chi_z_init = 0; // bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_z"));
    double sigma_vb_init = 0;//bot_param_get_double_or_fail(param, "state_estimator.sigma0.vb");

    Eigen::Vector3d xyz_cov_diag =
        Eigen::Array3d(sigma_Delta_xy_init, sigma_Delta_xy_init, sigma_Delta_z_init).square();

    Eigen::Vector3d init_chi_cov_diag = Eigen::Array3d(sigma_chi_xy_init, sigma_chi_xy_init, sigma_chi_z_init).square();
    init_covariance.setZero();

    //set all the sub-blocks of the covariance matrix
    init_covariance.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = std::pow(sigma_vb_init,2) * Eigen::Matrix3d::Identity();
    init_covariance.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = init_chi_cov_diag.asDiagonal();
    init_covariance.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = xyz_cov_diag.asDiagonal();
    init_covariance.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Eigen::Matrix3d::Identity()
        * ins_config.cov_gyro_bias * ins_config.cov_gyro_bias;
    init_covariance.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Eigen::Matrix3d::Identity()
        * ins_config.cov_accel_bias * ins_config.cov_accel_bias;

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> accel_error(0, ins_config.cov_accel);
    std::normal_distribution<double> gyro_error(0, ins_config.cov_gyro);
    std::normal_distribution<double> accel_bias_error(0, ins_config.cov_accel_bias);
    std::normal_distribution<double> gyro_bias_error(0, ins_config.cov_gyro_bias);

    double accel_bias = 0;
    double gyro_bias = 0;

    InsModule ins_module(ins_config, Eigen::Affine3d::Identity());

    // the imu is tossed from an airplane with a constant horizontal velocity
    // of 1 m/s and zero vertical velocity

    // the imu should measure zero angular velocity
    // zero acceleration (the gravity is compensated)

    // the initial state of the filter should be the constant velocity and the
    // altitude
    init_state.velocity() << 1, 0, 0;
    // 1/2 * 9.81 * 4
    // we expect the z to be 0 after 2 seconds of free fall
    init_state.position() << 0, 0, 19.62;


    StateEstimator state_estimator(new RBISResetUpdate(init_state,
                                                          init_covariance,
                                                          pronto::RBISUpdateInterface::reset,
                                                          init_state.utime),
                                      history_span);

    Eigen::Vector3d result;
    // iterate over 2 seconds of measurements
    for(int i = 0; i < 2*1e3; i++){
        imu_measurement.utime = (i * ins_config.dt) * 1e6;
        seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator.seed(seed);
        imu_measurement.acceleration = Eigen::Vector3d::Zero();
        imu_measurement.acceleration += Eigen::Vector3d::Ones()*accel_bias;
        imu_measurement.acceleration += Eigen::Vector3d::Ones() * accel_error(generator);

        seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator.seed(seed);

        imu_measurement.omega = Eigen::Vector3d::Zero();
        imu_measurement.omega += Eigen::Vector3d::Ones()*gyro_bias;
        imu_measurement.omega += Eigen::Vector3d::Ones() * gyro_error(generator);

        seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator.seed(seed);

        accel_bias += accel_bias_error(generator);

        seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator.seed(seed);

        gyro_bias += gyro_bias_error(generator);

        RBISUpdateInterface* meas = ins_module.processMessage(&imu_measurement,&state_estimator);

        state_estimator.addUpdate(meas, true);
        RBIS head_state;
        RBIM head_covariance;
        state_estimator.getHeadState(head_state,head_covariance);
        result = head_state.position();
    }
    // we expect to land 2 meters in front of us, on the ground.
    // we don't accept an error superior to 5 cm
    // TODO maybe we should reduce it
    EXPECT_NEAR(result(0),2,0.18);
    EXPECT_NEAR(result(1),0,0.18);
    EXPECT_NEAR(result(2),0,0.18);

}

int main(int argc, char**argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}




