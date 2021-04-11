#ifndef RCG_ANYMAL_MODEL_CONSTANTS_H_
#define RCG_ANYMAL_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace pronto {
namespace anymal {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar rz_velodyne_base_link = -2.443459987640381;
const Scalar sin_rz_velodyne_base_link = ScalarTraits::sin(rz_velodyne_base_link);
const Scalar cos_rz_velodyne_base_link = ScalarTraits::cos(rz_velodyne_base_link);
const Scalar rz_velodyne = -2.443459987640381;
const Scalar sin_rz_velodyne = ScalarTraits::sin(rz_velodyne);
const Scalar cos_rz_velodyne = ScalarTraits::cos(rz_velodyne);
const Scalar rx_camera_link = -0.0063240001909434795;
const Scalar sin_rx_camera_link = ScalarTraits::sin(rx_camera_link);
const Scalar cos_rx_camera_link = ScalarTraits::cos(rx_camera_link);
const Scalar ry_camera_link = 0.20410700142383575;
const Scalar sin_ry_camera_link = ScalarTraits::sin(ry_camera_link);
const Scalar cos_ry_camera_link = ScalarTraits::cos(ry_camera_link);
const Scalar rz_camera_link = 0.03119100071489811;
const Scalar sin_rz_camera_link = ScalarTraits::sin(rz_camera_link);
const Scalar cos_rz_camera_link = ScalarTraits::cos(rz_camera_link);
const Scalar rx_realsense_parent = -0.0063240001909434795;
const Scalar sin_rx_realsense_parent = ScalarTraits::sin(rx_realsense_parent);
const Scalar cos_rx_realsense_parent = ScalarTraits::cos(rx_realsense_parent);
const Scalar ry_realsense_parent = 0.20410700142383575;
const Scalar sin_ry_realsense_parent = ScalarTraits::sin(ry_realsense_parent);
const Scalar cos_ry_realsense_parent = ScalarTraits::cos(ry_realsense_parent);
const Scalar rz_realsense_parent = 0.03119100071489811;
const Scalar sin_rz_realsense_parent = ScalarTraits::sin(rz_realsense_parent);
const Scalar cos_rz_realsense_parent = ScalarTraits::cos(rz_realsense_parent);
const Scalar rx_realsense_d435i_bottom_screw_frame = -0.0063240001909434795;
const Scalar sin_rx_realsense_d435i_bottom_screw_frame = ScalarTraits::sin(rx_realsense_d435i_bottom_screw_frame);
const Scalar cos_rx_realsense_d435i_bottom_screw_frame = ScalarTraits::cos(rx_realsense_d435i_bottom_screw_frame);
const Scalar ry_realsense_d435i_bottom_screw_frame = 0.20410700142383575;
const Scalar sin_ry_realsense_d435i_bottom_screw_frame = ScalarTraits::sin(ry_realsense_d435i_bottom_screw_frame);
const Scalar cos_ry_realsense_d435i_bottom_screw_frame = ScalarTraits::cos(ry_realsense_d435i_bottom_screw_frame);
const Scalar rz_realsense_d435i_bottom_screw_frame = 0.03119100071489811;
const Scalar sin_rz_realsense_d435i_bottom_screw_frame = ScalarTraits::sin(rz_realsense_d435i_bottom_screw_frame);
const Scalar cos_rz_realsense_d435i_bottom_screw_frame = ScalarTraits::cos(rz_realsense_d435i_bottom_screw_frame);
const Scalar rx_realsense_d435i_link = -0.0063240001909434795;
const Scalar sin_rx_realsense_d435i_link = ScalarTraits::sin(rx_realsense_d435i_link);
const Scalar cos_rx_realsense_d435i_link = ScalarTraits::cos(rx_realsense_d435i_link);
const Scalar ry_realsense_d435i_link = 0.20410700142383575;
const Scalar sin_ry_realsense_d435i_link = ScalarTraits::sin(ry_realsense_d435i_link);
const Scalar cos_ry_realsense_d435i_link = ScalarTraits::cos(ry_realsense_d435i_link);
const Scalar rz_realsense_d435i_link = 0.03119100071489811;
const Scalar sin_rz_realsense_d435i_link = ScalarTraits::sin(rz_realsense_d435i_link);
const Scalar cos_rz_realsense_d435i_link = ScalarTraits::cos(rz_realsense_d435i_link);
const Scalar tx_LF_HAA = 0.2770000100135803;
const Scalar ty_LF_HAA = 0.11599999666213989;
const Scalar ty_LF_HFE = 0.04100000113248825;
const Scalar tz_LF_HFE = 0.06350000202655792;
const Scalar tx_LF_KFE = 0.25;
const Scalar tz_LF_KFE = 0.10899999737739563;
const Scalar tx_RF_HAA = 0.2770000100135803;
const Scalar ty_RF_HAA = -0.11599999666213989;
const Scalar ty_RF_HFE = -0.04100000113248825;
const Scalar tz_RF_HFE = 0.06350000202655792;
const Scalar tx_RF_KFE = 0.25;
const Scalar tz_RF_KFE = -0.10899999737739563;
const Scalar tx_LH_HAA = -0.2770000100135803;
const Scalar ty_LH_HAA = 0.11599999666213989;
const Scalar ty_LH_HFE = 0.04100000113248825;
const Scalar tz_LH_HFE = -0.06350000202655792;
const Scalar tx_LH_KFE = 0.25;
const Scalar tz_LH_KFE = 0.10899999737739563;
const Scalar tx_RH_HAA = -0.2770000100135803;
const Scalar ty_RH_HAA = -0.11599999666213989;
const Scalar ty_RH_HFE = -0.04100000113248825;
const Scalar tz_RH_HFE = -0.06350000202655792;
const Scalar tx_RH_KFE = 0.25;
const Scalar tz_RH_KFE = -0.10899999737739563;
const Scalar tx_leica_prism = -0.03700000047683716;
const Scalar tz_leica_prism = 0.35499998927116394;
const Scalar tx_imu_mount_link = 0.05534999817609787;
const Scalar ty_imu_mount_link = 0.05635000020265579;
const Scalar tz_imu_mount_link = 0.1932000070810318;
const Scalar tx_imu_link = 0.03799999877810478;
const Scalar ty_imu_link = 0.062449999153614044;
const Scalar tz_imu_link = 0.18369999527931213;
const Scalar tx_velodyne_base_link = -0.10199999809265137;
const Scalar tz_velodyne_base_link = 0.28999999165534973;
const Scalar tx_velodyne = -0.10199999809265137;
const Scalar tz_velodyne = 0.3276999890804291;
const Scalar tx_camera_link = 0.3709999918937683;
const Scalar ty_camera_link = 0.017999999225139618;
const Scalar tz_camera_link = 0.16200000047683716;
const Scalar tx_realsense_parent = 0.35862600803375244;
const Scalar ty_realsense_parent = 1.140000022132881E-4;
const Scalar tz_realsense_parent = 0.1519089937210083;
const Scalar tx_realsense_d435i_bottom_screw_frame = 0.35862600803375244;
const Scalar ty_realsense_d435i_bottom_screw_frame = 1.140000022132881E-4;
const Scalar tz_realsense_d435i_bottom_screw_frame = 0.1519089937210083;
const Scalar tx_realsense_d435i_link = 0.3709999918937683;
const Scalar ty_realsense_d435i_link = 0.017999999225139618;
const Scalar tz_realsense_d435i_link = 0.16200000047683716;
const Scalar ty_LF_ADAPTER = -0.10000000149011612;
const Scalar tz_LF_ADAPTER = -0.019999999552965164;
const Scalar tx_LF_FOOT = 0.32124999165534973;
const Scalar ty_LF_FOOT = -0.10000000149011612;
const Scalar tz_LF_FOOT = -0.019999999552965164;
const Scalar ty_RF_ADAPTER = -0.10000000149011612;
const Scalar tz_RF_ADAPTER = 0.019999999552965164;
const Scalar tx_RF_FOOT = 0.32124999165534973;
const Scalar ty_RF_FOOT = -0.10000000149011612;
const Scalar tz_RF_FOOT = 0.019999999552965164;
const Scalar ty_LH_ADAPTER = 0.10000000149011612;
const Scalar tz_LH_ADAPTER = -0.019999999552965164;
const Scalar tx_LH_FOOT = 0.32124999165534973;
const Scalar ty_LH_FOOT = 0.10000000149011612;
const Scalar tz_LH_FOOT = -0.019999999552965164;
const Scalar ty_RH_ADAPTER = 0.10000000149011612;
const Scalar tz_RH_ADAPTER = 0.019999999552965164;
const Scalar tx_RH_FOOT = 0.32124999165534973;
const Scalar ty_RH_FOOT = 0.10000000149011612;
const Scalar tz_RH_FOOT = 0.019999999552965164;
const Scalar m_base = 18.19849967956543;
const Scalar comx_base = 0.004984000232070684;
const Scalar comy_base = -7.430000114254653E-4;
const Scalar comz_base = 0.06640399992465973;
const Scalar ix_base = 0.36886999011039734;
const Scalar ixy_base = 0.005042000208050013;
const Scalar ixz_base = 0.006616999860852957;
const Scalar iy_base = 0.8737519979476929;
const Scalar iyz_base = 0.0018449999624863267;
const Scalar iz_base = 0.7158809900283813;
const Scalar m_LF_HIP = 1.4246200323104858;
const Scalar comx_LF_HIP = 1.5199999324977398E-4;
const Scalar comy_LF_HIP = -0.0037869999650865793;
const Scalar comz_LF_HIP = 0.06451600044965744;
const Scalar ix_LF_HIP = 0.007930999621748924;
const Scalar ixy_LF_HIP = 2.5999999706982635E-5;
const Scalar ixz_LF_HIP = -9.999999974752427E-7;
const Scalar iy_LF_HIP = 0.008232000283896923;
const Scalar iyz_LF_HIP = -3.330000035930425E-4;
const Scalar iz_LF_HIP = 0.0024510000366717577;
const Scalar m_LF_THIGH = 1.6349799633026123;
const Scalar comx_LF_THIGH = 0.21458299458026886;
const Scalar comy_LF_THIGH = 0.0038980001118034124;
const Scalar comz_LF_THIGH = 0.05422699823975563;
const Scalar ix_LF_THIGH = 0.007327000144869089;
const Scalar ixy_LF_THIGH = 0.0013000000035390258;
const Scalar ixz_LF_THIGH = 0.017619000747799873;
const Scalar iy_LF_THIGH = 0.09212899953126907;
const Scalar iyz_LF_THIGH = 4.130000015720725E-4;
const Scalar iz_LF_THIGH = 0.08737300336360931;
const Scalar m_LF_SHANK = 0.3473750054836273;
const Scalar comx_LF_SHANK = 0.09806399792432785;
const Scalar comy_LF_SHANK = -0.058733001351356506;
const Scalar comz_LF_SHANK = -0.010824000462889671;
const Scalar ix_LF_SHANK = 0.002257999964058399;
const Scalar ixy_LF_SHANK = -0.0033629999961704016;
const Scalar ixz_LF_SHANK = -6.849999772384763E-4;
const Scalar iy_LF_SHANK = 0.01023900043219328;
const Scalar iyz_LF_SHANK = 2.530000056140125E-4;
const Scalar iz_LF_SHANK = 0.012242999859154224;
const Scalar m_RF_HIP = 1.4246200323104858;
const Scalar comx_RF_HIP = 1.5199999324977398E-4;
const Scalar comy_RF_HIP = 0.0037869999650865793;
const Scalar comz_RF_HIP = 0.06451600044965744;
const Scalar ix_RF_HIP = 0.007930999621748924;
const Scalar ixy_RF_HIP = -2.5999999706982635E-5;
const Scalar ixz_RF_HIP = 2.9000000722589903E-5;
const Scalar iy_RF_HIP = 0.008232000283896923;
const Scalar iyz_RF_HIP = 3.330000035930425E-4;
const Scalar iz_RF_HIP = 0.0024510000366717577;
const Scalar m_RF_THIGH = 1.6349799633026123;
const Scalar comx_RF_THIGH = 0.21458299458026886;
const Scalar comy_RF_THIGH = 0.0038980001118034124;
const Scalar comz_RF_THIGH = -0.05422699823975563;
const Scalar ix_RF_THIGH = 0.007327000144869089;
const Scalar ixy_RF_THIGH = 0.0014349999837577343;
const Scalar ixz_RF_THIGH = -0.017619000747799873;
const Scalar iy_RF_THIGH = 0.09212899953126907;
const Scalar iyz_RF_THIGH = -4.130000015720725E-4;
const Scalar iz_RF_THIGH = 0.08737300336360931;
const Scalar m_RF_SHANK = 0.3473750054836273;
const Scalar comx_RF_SHANK = 0.09806399792432785;
const Scalar comy_RF_SHANK = -0.058733001351356506;
const Scalar comz_RF_SHANK = 0.010824000462889671;
const Scalar ix_RF_SHANK = 0.002257999964058399;
const Scalar ixy_RF_SHANK = -0.0034759999252855778;
const Scalar ixz_RF_SHANK = 6.849999772384763E-4;
const Scalar iy_RF_SHANK = 0.01023900043219328;
const Scalar iyz_RF_SHANK = -2.530000056140125E-4;
const Scalar iz_RF_SHANK = 0.012242999859154224;
const Scalar m_LH_HIP = 1.4246200323104858;
const Scalar comx_LH_HIP = 1.5199999324977398E-4;
const Scalar comy_LH_HIP = -0.0037869999650865793;
const Scalar comz_LH_HIP = -0.06451600044965744;
const Scalar ix_LH_HIP = 0.007930999621748924;
const Scalar ixy_LH_HIP = 2.5999999706982635E-5;
const Scalar ixz_LH_HIP = 9.999999974752427E-7;
const Scalar iy_LH_HIP = 0.008232000283896923;
const Scalar iyz_LH_HIP = 3.330000035930425E-4;
const Scalar iz_LH_HIP = 0.0024510000366717577;
const Scalar m_LH_THIGH = 1.6349799633026123;
const Scalar comx_LH_THIGH = 0.21458299458026886;
const Scalar comy_LH_THIGH = -0.0038980001118034124;
const Scalar comz_LH_THIGH = 0.05422699823975563;
const Scalar ix_LH_THIGH = 0.007327000144869089;
const Scalar ixy_LH_THIGH = -0.0013000000035390258;
const Scalar ixz_LH_THIGH = 0.017619000747799873;
const Scalar iy_LH_THIGH = 0.09212899953126907;
const Scalar iyz_LH_THIGH = -4.130000015720725E-4;
const Scalar iz_LH_THIGH = 0.08737300336360931;
const Scalar m_LH_SHANK = 0.3473750054836273;
const Scalar comx_LH_SHANK = 0.09806399792432785;
const Scalar comy_LH_SHANK = 0.058733001351356506;
const Scalar comz_LH_SHANK = -0.010824000462889671;
const Scalar ix_LH_SHANK = 0.002257999964058399;
const Scalar ixy_LH_SHANK = 0.0033629999961704016;
const Scalar ixz_LH_SHANK = -6.849999772384763E-4;
const Scalar iy_LH_SHANK = 0.01023900043219328;
const Scalar iyz_LH_SHANK = -2.530000056140125E-4;
const Scalar iz_LH_SHANK = 0.012242999859154224;
const Scalar m_RH_HIP = 1.4246200323104858;
const Scalar comx_RH_HIP = 1.5199999324977398E-4;
const Scalar comy_RH_HIP = 0.0037869999650865793;
const Scalar comz_RH_HIP = -0.06451600044965744;
const Scalar ix_RH_HIP = 0.007930999621748924;
const Scalar ixy_RH_HIP = -2.5999999706982635E-5;
const Scalar ixz_RH_HIP = -2.9000000722589903E-5;
const Scalar iy_RH_HIP = 0.008232000283896923;
const Scalar iyz_RH_HIP = -3.330000035930425E-4;
const Scalar iz_RH_HIP = 0.0024510000366717577;
const Scalar m_RH_THIGH = 1.6349799633026123;
const Scalar comx_RH_THIGH = 0.21458299458026886;
const Scalar comy_RH_THIGH = -0.0038980001118034124;
const Scalar comz_RH_THIGH = -0.05422699823975563;
const Scalar ix_RH_THIGH = 0.007327000144869089;
const Scalar ixy_RH_THIGH = -0.0014349999837577343;
const Scalar ixz_RH_THIGH = -0.017619000747799873;
const Scalar iy_RH_THIGH = 0.09212899953126907;
const Scalar iyz_RH_THIGH = 4.130000015720725E-4;
const Scalar iz_RH_THIGH = 0.08737300336360931;
const Scalar m_RH_SHANK = 0.3473750054836273;
const Scalar comx_RH_SHANK = 0.09806399792432785;
const Scalar comy_RH_SHANK = 0.058733001351356506;
const Scalar comz_RH_SHANK = 0.010824000462889671;
const Scalar ix_RH_SHANK = 0.002257999964058399;
const Scalar ixy_RH_SHANK = 0.0034759999252855778;
const Scalar ixz_RH_SHANK = 6.849999772384763E-4;
const Scalar iy_RH_SHANK = 0.01023900043219328;
const Scalar iyz_RH_SHANK = 2.530000056140125E-4;
const Scalar iz_RH_SHANK = 0.012242999859154224;

}
}
#endif
