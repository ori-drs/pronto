#ifndef ANYMAL_TRANSFORMS_H_
#define ANYMAL_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace pronto {
namespace anymal {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_X_LF_FOOT : public TransformMotion<Type_fr_base_X_LF_FOOT>
    {
        Type_fr_base_X_LF_FOOT();
        const Type_fr_base_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RF_FOOT : public TransformMotion<Type_fr_base_X_RF_FOOT>
    {
        Type_fr_base_X_RF_FOOT();
        const Type_fr_base_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_LH_FOOT : public TransformMotion<Type_fr_base_X_LH_FOOT>
    {
        Type_fr_base_X_LH_FOOT();
        const Type_fr_base_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RH_FOOT : public TransformMotion<Type_fr_base_X_RH_FOOT>
    {
        Type_fr_base_X_RH_FOOT();
        const Type_fr_base_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LF_FOOT : public TransformMotion<Type_imu_link_X_LF_FOOT>
    {
        Type_imu_link_X_LF_FOOT();
        const Type_imu_link_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RF_FOOT : public TransformMotion<Type_imu_link_X_RF_FOOT>
    {
        Type_imu_link_X_RF_FOOT();
        const Type_imu_link_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LH_FOOT : public TransformMotion<Type_imu_link_X_LH_FOOT>
    {
        Type_imu_link_X_LH_FOOT();
        const Type_imu_link_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RH_FOOT : public TransformMotion<Type_imu_link_X_RH_FOOT>
    {
        Type_imu_link_X_RH_FOOT();
        const Type_imu_link_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HAA : public TransformMotion<Type_fr_base_X_fr_LF_HAA>
    {
        Type_fr_base_X_fr_LF_HAA();
        const Type_fr_base_X_fr_LF_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HFE : public TransformMotion<Type_fr_base_X_fr_LF_HFE>
    {
        Type_fr_base_X_fr_LF_HFE();
        const Type_fr_base_X_fr_LF_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_KFE : public TransformMotion<Type_fr_base_X_fr_LF_KFE>
    {
        Type_fr_base_X_fr_LF_KFE();
        const Type_fr_base_X_fr_LF_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HAA : public TransformMotion<Type_fr_base_X_fr_RF_HAA>
    {
        Type_fr_base_X_fr_RF_HAA();
        const Type_fr_base_X_fr_RF_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HFE : public TransformMotion<Type_fr_base_X_fr_RF_HFE>
    {
        Type_fr_base_X_fr_RF_HFE();
        const Type_fr_base_X_fr_RF_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_KFE : public TransformMotion<Type_fr_base_X_fr_RF_KFE>
    {
        Type_fr_base_X_fr_RF_KFE();
        const Type_fr_base_X_fr_RF_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HAA : public TransformMotion<Type_fr_base_X_fr_LH_HAA>
    {
        Type_fr_base_X_fr_LH_HAA();
        const Type_fr_base_X_fr_LH_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HFE : public TransformMotion<Type_fr_base_X_fr_LH_HFE>
    {
        Type_fr_base_X_fr_LH_HFE();
        const Type_fr_base_X_fr_LH_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_KFE : public TransformMotion<Type_fr_base_X_fr_LH_KFE>
    {
        Type_fr_base_X_fr_LH_KFE();
        const Type_fr_base_X_fr_LH_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HAA : public TransformMotion<Type_fr_base_X_fr_RH_HAA>
    {
        Type_fr_base_X_fr_RH_HAA();
        const Type_fr_base_X_fr_RH_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HFE : public TransformMotion<Type_fr_base_X_fr_RH_HFE>
    {
        Type_fr_base_X_fr_RH_HFE();
        const Type_fr_base_X_fr_RH_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_KFE : public TransformMotion<Type_fr_base_X_fr_RH_KFE>
    {
        Type_fr_base_X_fr_RH_KFE();
        const Type_fr_base_X_fr_RH_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_HAA : public TransformMotion<Type_imu_link_X_fr_LF_HAA>
    {
        Type_imu_link_X_fr_LF_HAA();
        const Type_imu_link_X_fr_LF_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_HFE : public TransformMotion<Type_imu_link_X_fr_LF_HFE>
    {
        Type_imu_link_X_fr_LF_HFE();
        const Type_imu_link_X_fr_LF_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_KFE : public TransformMotion<Type_imu_link_X_fr_LF_KFE>
    {
        Type_imu_link_X_fr_LF_KFE();
        const Type_imu_link_X_fr_LF_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_HAA : public TransformMotion<Type_imu_link_X_fr_RF_HAA>
    {
        Type_imu_link_X_fr_RF_HAA();
        const Type_imu_link_X_fr_RF_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_HFE : public TransformMotion<Type_imu_link_X_fr_RF_HFE>
    {
        Type_imu_link_X_fr_RF_HFE();
        const Type_imu_link_X_fr_RF_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_KFE : public TransformMotion<Type_imu_link_X_fr_RF_KFE>
    {
        Type_imu_link_X_fr_RF_KFE();
        const Type_imu_link_X_fr_RF_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_HAA : public TransformMotion<Type_imu_link_X_fr_LH_HAA>
    {
        Type_imu_link_X_fr_LH_HAA();
        const Type_imu_link_X_fr_LH_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_HFE : public TransformMotion<Type_imu_link_X_fr_LH_HFE>
    {
        Type_imu_link_X_fr_LH_HFE();
        const Type_imu_link_X_fr_LH_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_KFE : public TransformMotion<Type_imu_link_X_fr_LH_KFE>
    {
        Type_imu_link_X_fr_LH_KFE();
        const Type_imu_link_X_fr_LH_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_HAA : public TransformMotion<Type_imu_link_X_fr_RH_HAA>
    {
        Type_imu_link_X_fr_RH_HAA();
        const Type_imu_link_X_fr_RH_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_HFE : public TransformMotion<Type_imu_link_X_fr_RH_HFE>
    {
        Type_imu_link_X_fr_RH_HFE();
        const Type_imu_link_X_fr_RH_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_KFE : public TransformMotion<Type_imu_link_X_fr_RH_KFE>
    {
        Type_imu_link_X_fr_RH_KFE();
        const Type_imu_link_X_fr_RH_KFE& update(const state_t&);
    };
    
    struct Type_fr_LF_HIP_X_fr_base : public TransformMotion<Type_fr_LF_HIP_X_fr_base>
    {
        Type_fr_LF_HIP_X_fr_base();
        const Type_fr_LF_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HIP : public TransformMotion<Type_fr_base_X_fr_LF_HIP>
    {
        Type_fr_base_X_fr_LF_HIP();
        const Type_fr_base_X_fr_LF_HIP& update(const state_t&);
    };
    
    struct Type_fr_LF_THIGH_X_fr_LF_HIP : public TransformMotion<Type_fr_LF_THIGH_X_fr_LF_HIP>
    {
        Type_fr_LF_THIGH_X_fr_LF_HIP();
        const Type_fr_LF_THIGH_X_fr_LF_HIP& update(const state_t&);
    };
    
    struct Type_fr_LF_HIP_X_fr_LF_THIGH : public TransformMotion<Type_fr_LF_HIP_X_fr_LF_THIGH>
    {
        Type_fr_LF_HIP_X_fr_LF_THIGH();
        const Type_fr_LF_HIP_X_fr_LF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LF_SHANK_X_fr_LF_THIGH : public TransformMotion<Type_fr_LF_SHANK_X_fr_LF_THIGH>
    {
        Type_fr_LF_SHANK_X_fr_LF_THIGH();
        const Type_fr_LF_SHANK_X_fr_LF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LF_THIGH_X_fr_LF_SHANK : public TransformMotion<Type_fr_LF_THIGH_X_fr_LF_SHANK>
    {
        Type_fr_LF_THIGH_X_fr_LF_SHANK();
        const Type_fr_LF_THIGH_X_fr_LF_SHANK& update(const state_t&);
    };
    
    struct Type_fr_RF_HIP_X_fr_base : public TransformMotion<Type_fr_RF_HIP_X_fr_base>
    {
        Type_fr_RF_HIP_X_fr_base();
        const Type_fr_RF_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HIP : public TransformMotion<Type_fr_base_X_fr_RF_HIP>
    {
        Type_fr_base_X_fr_RF_HIP();
        const Type_fr_base_X_fr_RF_HIP& update(const state_t&);
    };
    
    struct Type_fr_RF_THIGH_X_fr_RF_HIP : public TransformMotion<Type_fr_RF_THIGH_X_fr_RF_HIP>
    {
        Type_fr_RF_THIGH_X_fr_RF_HIP();
        const Type_fr_RF_THIGH_X_fr_RF_HIP& update(const state_t&);
    };
    
    struct Type_fr_RF_HIP_X_fr_RF_THIGH : public TransformMotion<Type_fr_RF_HIP_X_fr_RF_THIGH>
    {
        Type_fr_RF_HIP_X_fr_RF_THIGH();
        const Type_fr_RF_HIP_X_fr_RF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RF_SHANK_X_fr_RF_THIGH : public TransformMotion<Type_fr_RF_SHANK_X_fr_RF_THIGH>
    {
        Type_fr_RF_SHANK_X_fr_RF_THIGH();
        const Type_fr_RF_SHANK_X_fr_RF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RF_THIGH_X_fr_RF_SHANK : public TransformMotion<Type_fr_RF_THIGH_X_fr_RF_SHANK>
    {
        Type_fr_RF_THIGH_X_fr_RF_SHANK();
        const Type_fr_RF_THIGH_X_fr_RF_SHANK& update(const state_t&);
    };
    
    struct Type_fr_LH_HIP_X_fr_base : public TransformMotion<Type_fr_LH_HIP_X_fr_base>
    {
        Type_fr_LH_HIP_X_fr_base();
        const Type_fr_LH_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HIP : public TransformMotion<Type_fr_base_X_fr_LH_HIP>
    {
        Type_fr_base_X_fr_LH_HIP();
        const Type_fr_base_X_fr_LH_HIP& update(const state_t&);
    };
    
    struct Type_fr_LH_THIGH_X_fr_LH_HIP : public TransformMotion<Type_fr_LH_THIGH_X_fr_LH_HIP>
    {
        Type_fr_LH_THIGH_X_fr_LH_HIP();
        const Type_fr_LH_THIGH_X_fr_LH_HIP& update(const state_t&);
    };
    
    struct Type_fr_LH_HIP_X_fr_LH_THIGH : public TransformMotion<Type_fr_LH_HIP_X_fr_LH_THIGH>
    {
        Type_fr_LH_HIP_X_fr_LH_THIGH();
        const Type_fr_LH_HIP_X_fr_LH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LH_SHANK_X_fr_LH_THIGH : public TransformMotion<Type_fr_LH_SHANK_X_fr_LH_THIGH>
    {
        Type_fr_LH_SHANK_X_fr_LH_THIGH();
        const Type_fr_LH_SHANK_X_fr_LH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LH_THIGH_X_fr_LH_SHANK : public TransformMotion<Type_fr_LH_THIGH_X_fr_LH_SHANK>
    {
        Type_fr_LH_THIGH_X_fr_LH_SHANK();
        const Type_fr_LH_THIGH_X_fr_LH_SHANK& update(const state_t&);
    };
    
    struct Type_fr_RH_HIP_X_fr_base : public TransformMotion<Type_fr_RH_HIP_X_fr_base>
    {
        Type_fr_RH_HIP_X_fr_base();
        const Type_fr_RH_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HIP : public TransformMotion<Type_fr_base_X_fr_RH_HIP>
    {
        Type_fr_base_X_fr_RH_HIP();
        const Type_fr_base_X_fr_RH_HIP& update(const state_t&);
    };
    
    struct Type_fr_RH_THIGH_X_fr_RH_HIP : public TransformMotion<Type_fr_RH_THIGH_X_fr_RH_HIP>
    {
        Type_fr_RH_THIGH_X_fr_RH_HIP();
        const Type_fr_RH_THIGH_X_fr_RH_HIP& update(const state_t&);
    };
    
    struct Type_fr_RH_HIP_X_fr_RH_THIGH : public TransformMotion<Type_fr_RH_HIP_X_fr_RH_THIGH>
    {
        Type_fr_RH_HIP_X_fr_RH_THIGH();
        const Type_fr_RH_HIP_X_fr_RH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RH_SHANK_X_fr_RH_THIGH : public TransformMotion<Type_fr_RH_SHANK_X_fr_RH_THIGH>
    {
        Type_fr_RH_SHANK_X_fr_RH_THIGH();
        const Type_fr_RH_SHANK_X_fr_RH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RH_THIGH_X_fr_RH_SHANK : public TransformMotion<Type_fr_RH_THIGH_X_fr_RH_SHANK>
    {
        Type_fr_RH_THIGH_X_fr_RH_SHANK();
        const Type_fr_RH_THIGH_X_fr_RH_SHANK& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_X_LF_FOOT fr_base_X_LF_FOOT;
    Type_fr_base_X_RF_FOOT fr_base_X_RF_FOOT;
    Type_fr_base_X_LH_FOOT fr_base_X_LH_FOOT;
    Type_fr_base_X_RH_FOOT fr_base_X_RH_FOOT;
    Type_imu_link_X_LF_FOOT imu_link_X_LF_FOOT;
    Type_imu_link_X_RF_FOOT imu_link_X_RF_FOOT;
    Type_imu_link_X_LH_FOOT imu_link_X_LH_FOOT;
    Type_imu_link_X_RH_FOOT imu_link_X_RH_FOOT;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_imu_link_X_fr_LF_HAA imu_link_X_fr_LF_HAA;
    Type_imu_link_X_fr_LF_HFE imu_link_X_fr_LF_HFE;
    Type_imu_link_X_fr_LF_KFE imu_link_X_fr_LF_KFE;
    Type_imu_link_X_fr_RF_HAA imu_link_X_fr_RF_HAA;
    Type_imu_link_X_fr_RF_HFE imu_link_X_fr_RF_HFE;
    Type_imu_link_X_fr_RF_KFE imu_link_X_fr_RF_KFE;
    Type_imu_link_X_fr_LH_HAA imu_link_X_fr_LH_HAA;
    Type_imu_link_X_fr_LH_HFE imu_link_X_fr_LH_HFE;
    Type_imu_link_X_fr_LH_KFE imu_link_X_fr_LH_KFE;
    Type_imu_link_X_fr_RH_HAA imu_link_X_fr_RH_HAA;
    Type_imu_link_X_fr_RH_HFE imu_link_X_fr_RH_HFE;
    Type_imu_link_X_fr_RH_KFE imu_link_X_fr_RH_KFE;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_SHANK_X_fr_LF_THIGH fr_LF_SHANK_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_SHANK fr_LF_THIGH_X_fr_LF_SHANK;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_SHANK_X_fr_RF_THIGH fr_RF_SHANK_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_SHANK fr_RF_THIGH_X_fr_RF_SHANK;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_SHANK_X_fr_LH_THIGH fr_LH_SHANK_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_SHANK fr_LH_THIGH_X_fr_LH_SHANK;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_SHANK_X_fr_RH_THIGH fr_RH_SHANK_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_SHANK fr_RH_THIGH_X_fr_RH_SHANK;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_X_LF_FOOT : public TransformForce<Type_fr_base_X_LF_FOOT>
    {
        Type_fr_base_X_LF_FOOT();
        const Type_fr_base_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RF_FOOT : public TransformForce<Type_fr_base_X_RF_FOOT>
    {
        Type_fr_base_X_RF_FOOT();
        const Type_fr_base_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_LH_FOOT : public TransformForce<Type_fr_base_X_LH_FOOT>
    {
        Type_fr_base_X_LH_FOOT();
        const Type_fr_base_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RH_FOOT : public TransformForce<Type_fr_base_X_RH_FOOT>
    {
        Type_fr_base_X_RH_FOOT();
        const Type_fr_base_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LF_FOOT : public TransformForce<Type_imu_link_X_LF_FOOT>
    {
        Type_imu_link_X_LF_FOOT();
        const Type_imu_link_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RF_FOOT : public TransformForce<Type_imu_link_X_RF_FOOT>
    {
        Type_imu_link_X_RF_FOOT();
        const Type_imu_link_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LH_FOOT : public TransformForce<Type_imu_link_X_LH_FOOT>
    {
        Type_imu_link_X_LH_FOOT();
        const Type_imu_link_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RH_FOOT : public TransformForce<Type_imu_link_X_RH_FOOT>
    {
        Type_imu_link_X_RH_FOOT();
        const Type_imu_link_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HAA : public TransformForce<Type_fr_base_X_fr_LF_HAA>
    {
        Type_fr_base_X_fr_LF_HAA();
        const Type_fr_base_X_fr_LF_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HFE : public TransformForce<Type_fr_base_X_fr_LF_HFE>
    {
        Type_fr_base_X_fr_LF_HFE();
        const Type_fr_base_X_fr_LF_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_KFE : public TransformForce<Type_fr_base_X_fr_LF_KFE>
    {
        Type_fr_base_X_fr_LF_KFE();
        const Type_fr_base_X_fr_LF_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HAA : public TransformForce<Type_fr_base_X_fr_RF_HAA>
    {
        Type_fr_base_X_fr_RF_HAA();
        const Type_fr_base_X_fr_RF_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HFE : public TransformForce<Type_fr_base_X_fr_RF_HFE>
    {
        Type_fr_base_X_fr_RF_HFE();
        const Type_fr_base_X_fr_RF_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_KFE : public TransformForce<Type_fr_base_X_fr_RF_KFE>
    {
        Type_fr_base_X_fr_RF_KFE();
        const Type_fr_base_X_fr_RF_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HAA : public TransformForce<Type_fr_base_X_fr_LH_HAA>
    {
        Type_fr_base_X_fr_LH_HAA();
        const Type_fr_base_X_fr_LH_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HFE : public TransformForce<Type_fr_base_X_fr_LH_HFE>
    {
        Type_fr_base_X_fr_LH_HFE();
        const Type_fr_base_X_fr_LH_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_KFE : public TransformForce<Type_fr_base_X_fr_LH_KFE>
    {
        Type_fr_base_X_fr_LH_KFE();
        const Type_fr_base_X_fr_LH_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HAA : public TransformForce<Type_fr_base_X_fr_RH_HAA>
    {
        Type_fr_base_X_fr_RH_HAA();
        const Type_fr_base_X_fr_RH_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HFE : public TransformForce<Type_fr_base_X_fr_RH_HFE>
    {
        Type_fr_base_X_fr_RH_HFE();
        const Type_fr_base_X_fr_RH_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_KFE : public TransformForce<Type_fr_base_X_fr_RH_KFE>
    {
        Type_fr_base_X_fr_RH_KFE();
        const Type_fr_base_X_fr_RH_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_HAA : public TransformForce<Type_imu_link_X_fr_LF_HAA>
    {
        Type_imu_link_X_fr_LF_HAA();
        const Type_imu_link_X_fr_LF_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_HFE : public TransformForce<Type_imu_link_X_fr_LF_HFE>
    {
        Type_imu_link_X_fr_LF_HFE();
        const Type_imu_link_X_fr_LF_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_KFE : public TransformForce<Type_imu_link_X_fr_LF_KFE>
    {
        Type_imu_link_X_fr_LF_KFE();
        const Type_imu_link_X_fr_LF_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_HAA : public TransformForce<Type_imu_link_X_fr_RF_HAA>
    {
        Type_imu_link_X_fr_RF_HAA();
        const Type_imu_link_X_fr_RF_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_HFE : public TransformForce<Type_imu_link_X_fr_RF_HFE>
    {
        Type_imu_link_X_fr_RF_HFE();
        const Type_imu_link_X_fr_RF_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_KFE : public TransformForce<Type_imu_link_X_fr_RF_KFE>
    {
        Type_imu_link_X_fr_RF_KFE();
        const Type_imu_link_X_fr_RF_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_HAA : public TransformForce<Type_imu_link_X_fr_LH_HAA>
    {
        Type_imu_link_X_fr_LH_HAA();
        const Type_imu_link_X_fr_LH_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_HFE : public TransformForce<Type_imu_link_X_fr_LH_HFE>
    {
        Type_imu_link_X_fr_LH_HFE();
        const Type_imu_link_X_fr_LH_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_KFE : public TransformForce<Type_imu_link_X_fr_LH_KFE>
    {
        Type_imu_link_X_fr_LH_KFE();
        const Type_imu_link_X_fr_LH_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_HAA : public TransformForce<Type_imu_link_X_fr_RH_HAA>
    {
        Type_imu_link_X_fr_RH_HAA();
        const Type_imu_link_X_fr_RH_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_HFE : public TransformForce<Type_imu_link_X_fr_RH_HFE>
    {
        Type_imu_link_X_fr_RH_HFE();
        const Type_imu_link_X_fr_RH_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_KFE : public TransformForce<Type_imu_link_X_fr_RH_KFE>
    {
        Type_imu_link_X_fr_RH_KFE();
        const Type_imu_link_X_fr_RH_KFE& update(const state_t&);
    };
    
    struct Type_fr_LF_HIP_X_fr_base : public TransformForce<Type_fr_LF_HIP_X_fr_base>
    {
        Type_fr_LF_HIP_X_fr_base();
        const Type_fr_LF_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HIP : public TransformForce<Type_fr_base_X_fr_LF_HIP>
    {
        Type_fr_base_X_fr_LF_HIP();
        const Type_fr_base_X_fr_LF_HIP& update(const state_t&);
    };
    
    struct Type_fr_LF_THIGH_X_fr_LF_HIP : public TransformForce<Type_fr_LF_THIGH_X_fr_LF_HIP>
    {
        Type_fr_LF_THIGH_X_fr_LF_HIP();
        const Type_fr_LF_THIGH_X_fr_LF_HIP& update(const state_t&);
    };
    
    struct Type_fr_LF_HIP_X_fr_LF_THIGH : public TransformForce<Type_fr_LF_HIP_X_fr_LF_THIGH>
    {
        Type_fr_LF_HIP_X_fr_LF_THIGH();
        const Type_fr_LF_HIP_X_fr_LF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LF_SHANK_X_fr_LF_THIGH : public TransformForce<Type_fr_LF_SHANK_X_fr_LF_THIGH>
    {
        Type_fr_LF_SHANK_X_fr_LF_THIGH();
        const Type_fr_LF_SHANK_X_fr_LF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LF_THIGH_X_fr_LF_SHANK : public TransformForce<Type_fr_LF_THIGH_X_fr_LF_SHANK>
    {
        Type_fr_LF_THIGH_X_fr_LF_SHANK();
        const Type_fr_LF_THIGH_X_fr_LF_SHANK& update(const state_t&);
    };
    
    struct Type_fr_RF_HIP_X_fr_base : public TransformForce<Type_fr_RF_HIP_X_fr_base>
    {
        Type_fr_RF_HIP_X_fr_base();
        const Type_fr_RF_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HIP : public TransformForce<Type_fr_base_X_fr_RF_HIP>
    {
        Type_fr_base_X_fr_RF_HIP();
        const Type_fr_base_X_fr_RF_HIP& update(const state_t&);
    };
    
    struct Type_fr_RF_THIGH_X_fr_RF_HIP : public TransformForce<Type_fr_RF_THIGH_X_fr_RF_HIP>
    {
        Type_fr_RF_THIGH_X_fr_RF_HIP();
        const Type_fr_RF_THIGH_X_fr_RF_HIP& update(const state_t&);
    };
    
    struct Type_fr_RF_HIP_X_fr_RF_THIGH : public TransformForce<Type_fr_RF_HIP_X_fr_RF_THIGH>
    {
        Type_fr_RF_HIP_X_fr_RF_THIGH();
        const Type_fr_RF_HIP_X_fr_RF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RF_SHANK_X_fr_RF_THIGH : public TransformForce<Type_fr_RF_SHANK_X_fr_RF_THIGH>
    {
        Type_fr_RF_SHANK_X_fr_RF_THIGH();
        const Type_fr_RF_SHANK_X_fr_RF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RF_THIGH_X_fr_RF_SHANK : public TransformForce<Type_fr_RF_THIGH_X_fr_RF_SHANK>
    {
        Type_fr_RF_THIGH_X_fr_RF_SHANK();
        const Type_fr_RF_THIGH_X_fr_RF_SHANK& update(const state_t&);
    };
    
    struct Type_fr_LH_HIP_X_fr_base : public TransformForce<Type_fr_LH_HIP_X_fr_base>
    {
        Type_fr_LH_HIP_X_fr_base();
        const Type_fr_LH_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HIP : public TransformForce<Type_fr_base_X_fr_LH_HIP>
    {
        Type_fr_base_X_fr_LH_HIP();
        const Type_fr_base_X_fr_LH_HIP& update(const state_t&);
    };
    
    struct Type_fr_LH_THIGH_X_fr_LH_HIP : public TransformForce<Type_fr_LH_THIGH_X_fr_LH_HIP>
    {
        Type_fr_LH_THIGH_X_fr_LH_HIP();
        const Type_fr_LH_THIGH_X_fr_LH_HIP& update(const state_t&);
    };
    
    struct Type_fr_LH_HIP_X_fr_LH_THIGH : public TransformForce<Type_fr_LH_HIP_X_fr_LH_THIGH>
    {
        Type_fr_LH_HIP_X_fr_LH_THIGH();
        const Type_fr_LH_HIP_X_fr_LH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LH_SHANK_X_fr_LH_THIGH : public TransformForce<Type_fr_LH_SHANK_X_fr_LH_THIGH>
    {
        Type_fr_LH_SHANK_X_fr_LH_THIGH();
        const Type_fr_LH_SHANK_X_fr_LH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LH_THIGH_X_fr_LH_SHANK : public TransformForce<Type_fr_LH_THIGH_X_fr_LH_SHANK>
    {
        Type_fr_LH_THIGH_X_fr_LH_SHANK();
        const Type_fr_LH_THIGH_X_fr_LH_SHANK& update(const state_t&);
    };
    
    struct Type_fr_RH_HIP_X_fr_base : public TransformForce<Type_fr_RH_HIP_X_fr_base>
    {
        Type_fr_RH_HIP_X_fr_base();
        const Type_fr_RH_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HIP : public TransformForce<Type_fr_base_X_fr_RH_HIP>
    {
        Type_fr_base_X_fr_RH_HIP();
        const Type_fr_base_X_fr_RH_HIP& update(const state_t&);
    };
    
    struct Type_fr_RH_THIGH_X_fr_RH_HIP : public TransformForce<Type_fr_RH_THIGH_X_fr_RH_HIP>
    {
        Type_fr_RH_THIGH_X_fr_RH_HIP();
        const Type_fr_RH_THIGH_X_fr_RH_HIP& update(const state_t&);
    };
    
    struct Type_fr_RH_HIP_X_fr_RH_THIGH : public TransformForce<Type_fr_RH_HIP_X_fr_RH_THIGH>
    {
        Type_fr_RH_HIP_X_fr_RH_THIGH();
        const Type_fr_RH_HIP_X_fr_RH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RH_SHANK_X_fr_RH_THIGH : public TransformForce<Type_fr_RH_SHANK_X_fr_RH_THIGH>
    {
        Type_fr_RH_SHANK_X_fr_RH_THIGH();
        const Type_fr_RH_SHANK_X_fr_RH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RH_THIGH_X_fr_RH_SHANK : public TransformForce<Type_fr_RH_THIGH_X_fr_RH_SHANK>
    {
        Type_fr_RH_THIGH_X_fr_RH_SHANK();
        const Type_fr_RH_THIGH_X_fr_RH_SHANK& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_X_LF_FOOT fr_base_X_LF_FOOT;
    Type_fr_base_X_RF_FOOT fr_base_X_RF_FOOT;
    Type_fr_base_X_LH_FOOT fr_base_X_LH_FOOT;
    Type_fr_base_X_RH_FOOT fr_base_X_RH_FOOT;
    Type_imu_link_X_LF_FOOT imu_link_X_LF_FOOT;
    Type_imu_link_X_RF_FOOT imu_link_X_RF_FOOT;
    Type_imu_link_X_LH_FOOT imu_link_X_LH_FOOT;
    Type_imu_link_X_RH_FOOT imu_link_X_RH_FOOT;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_imu_link_X_fr_LF_HAA imu_link_X_fr_LF_HAA;
    Type_imu_link_X_fr_LF_HFE imu_link_X_fr_LF_HFE;
    Type_imu_link_X_fr_LF_KFE imu_link_X_fr_LF_KFE;
    Type_imu_link_X_fr_RF_HAA imu_link_X_fr_RF_HAA;
    Type_imu_link_X_fr_RF_HFE imu_link_X_fr_RF_HFE;
    Type_imu_link_X_fr_RF_KFE imu_link_X_fr_RF_KFE;
    Type_imu_link_X_fr_LH_HAA imu_link_X_fr_LH_HAA;
    Type_imu_link_X_fr_LH_HFE imu_link_X_fr_LH_HFE;
    Type_imu_link_X_fr_LH_KFE imu_link_X_fr_LH_KFE;
    Type_imu_link_X_fr_RH_HAA imu_link_X_fr_RH_HAA;
    Type_imu_link_X_fr_RH_HFE imu_link_X_fr_RH_HFE;
    Type_imu_link_X_fr_RH_KFE imu_link_X_fr_RH_KFE;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_SHANK_X_fr_LF_THIGH fr_LF_SHANK_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_SHANK fr_LF_THIGH_X_fr_LF_SHANK;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_SHANK_X_fr_RF_THIGH fr_RF_SHANK_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_SHANK fr_RF_THIGH_X_fr_RF_SHANK;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_SHANK_X_fr_LH_THIGH fr_LH_SHANK_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_SHANK fr_LH_THIGH_X_fr_LH_SHANK;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_SHANK_X_fr_RH_THIGH fr_RH_SHANK_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_SHANK fr_RH_THIGH_X_fr_RH_SHANK;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_X_LF_FOOT : public TransformHomogeneous<Type_fr_base_X_LF_FOOT>
    {
        Type_fr_base_X_LF_FOOT();
        const Type_fr_base_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RF_FOOT : public TransformHomogeneous<Type_fr_base_X_RF_FOOT>
    {
        Type_fr_base_X_RF_FOOT();
        const Type_fr_base_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_LH_FOOT : public TransformHomogeneous<Type_fr_base_X_LH_FOOT>
    {
        Type_fr_base_X_LH_FOOT();
        const Type_fr_base_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_RH_FOOT : public TransformHomogeneous<Type_fr_base_X_RH_FOOT>
    {
        Type_fr_base_X_RH_FOOT();
        const Type_fr_base_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LF_FOOT : public TransformHomogeneous<Type_imu_link_X_LF_FOOT>
    {
        Type_imu_link_X_LF_FOOT();
        const Type_imu_link_X_LF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RF_FOOT : public TransformHomogeneous<Type_imu_link_X_RF_FOOT>
    {
        Type_imu_link_X_RF_FOOT();
        const Type_imu_link_X_RF_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_LH_FOOT : public TransformHomogeneous<Type_imu_link_X_LH_FOOT>
    {
        Type_imu_link_X_LH_FOOT();
        const Type_imu_link_X_LH_FOOT& update(const state_t&);
    };
    
    struct Type_imu_link_X_RH_FOOT : public TransformHomogeneous<Type_imu_link_X_RH_FOOT>
    {
        Type_imu_link_X_RH_FOOT();
        const Type_imu_link_X_RH_FOOT& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HAA : public TransformHomogeneous<Type_fr_base_X_fr_LF_HAA>
    {
        Type_fr_base_X_fr_LF_HAA();
        const Type_fr_base_X_fr_LF_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HFE : public TransformHomogeneous<Type_fr_base_X_fr_LF_HFE>
    {
        Type_fr_base_X_fr_LF_HFE();
        const Type_fr_base_X_fr_LF_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_KFE : public TransformHomogeneous<Type_fr_base_X_fr_LF_KFE>
    {
        Type_fr_base_X_fr_LF_KFE();
        const Type_fr_base_X_fr_LF_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HAA : public TransformHomogeneous<Type_fr_base_X_fr_RF_HAA>
    {
        Type_fr_base_X_fr_RF_HAA();
        const Type_fr_base_X_fr_RF_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HFE : public TransformHomogeneous<Type_fr_base_X_fr_RF_HFE>
    {
        Type_fr_base_X_fr_RF_HFE();
        const Type_fr_base_X_fr_RF_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_KFE : public TransformHomogeneous<Type_fr_base_X_fr_RF_KFE>
    {
        Type_fr_base_X_fr_RF_KFE();
        const Type_fr_base_X_fr_RF_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HAA : public TransformHomogeneous<Type_fr_base_X_fr_LH_HAA>
    {
        Type_fr_base_X_fr_LH_HAA();
        const Type_fr_base_X_fr_LH_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HFE : public TransformHomogeneous<Type_fr_base_X_fr_LH_HFE>
    {
        Type_fr_base_X_fr_LH_HFE();
        const Type_fr_base_X_fr_LH_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_KFE : public TransformHomogeneous<Type_fr_base_X_fr_LH_KFE>
    {
        Type_fr_base_X_fr_LH_KFE();
        const Type_fr_base_X_fr_LH_KFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HAA : public TransformHomogeneous<Type_fr_base_X_fr_RH_HAA>
    {
        Type_fr_base_X_fr_RH_HAA();
        const Type_fr_base_X_fr_RH_HAA& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HFE : public TransformHomogeneous<Type_fr_base_X_fr_RH_HFE>
    {
        Type_fr_base_X_fr_RH_HFE();
        const Type_fr_base_X_fr_RH_HFE& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_KFE : public TransformHomogeneous<Type_fr_base_X_fr_RH_KFE>
    {
        Type_fr_base_X_fr_RH_KFE();
        const Type_fr_base_X_fr_RH_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_HAA : public TransformHomogeneous<Type_imu_link_X_fr_LF_HAA>
    {
        Type_imu_link_X_fr_LF_HAA();
        const Type_imu_link_X_fr_LF_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_HFE : public TransformHomogeneous<Type_imu_link_X_fr_LF_HFE>
    {
        Type_imu_link_X_fr_LF_HFE();
        const Type_imu_link_X_fr_LF_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LF_KFE : public TransformHomogeneous<Type_imu_link_X_fr_LF_KFE>
    {
        Type_imu_link_X_fr_LF_KFE();
        const Type_imu_link_X_fr_LF_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_HAA : public TransformHomogeneous<Type_imu_link_X_fr_RF_HAA>
    {
        Type_imu_link_X_fr_RF_HAA();
        const Type_imu_link_X_fr_RF_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_HFE : public TransformHomogeneous<Type_imu_link_X_fr_RF_HFE>
    {
        Type_imu_link_X_fr_RF_HFE();
        const Type_imu_link_X_fr_RF_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RF_KFE : public TransformHomogeneous<Type_imu_link_X_fr_RF_KFE>
    {
        Type_imu_link_X_fr_RF_KFE();
        const Type_imu_link_X_fr_RF_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_HAA : public TransformHomogeneous<Type_imu_link_X_fr_LH_HAA>
    {
        Type_imu_link_X_fr_LH_HAA();
        const Type_imu_link_X_fr_LH_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_HFE : public TransformHomogeneous<Type_imu_link_X_fr_LH_HFE>
    {
        Type_imu_link_X_fr_LH_HFE();
        const Type_imu_link_X_fr_LH_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_LH_KFE : public TransformHomogeneous<Type_imu_link_X_fr_LH_KFE>
    {
        Type_imu_link_X_fr_LH_KFE();
        const Type_imu_link_X_fr_LH_KFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_HAA : public TransformHomogeneous<Type_imu_link_X_fr_RH_HAA>
    {
        Type_imu_link_X_fr_RH_HAA();
        const Type_imu_link_X_fr_RH_HAA& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_HFE : public TransformHomogeneous<Type_imu_link_X_fr_RH_HFE>
    {
        Type_imu_link_X_fr_RH_HFE();
        const Type_imu_link_X_fr_RH_HFE& update(const state_t&);
    };
    
    struct Type_imu_link_X_fr_RH_KFE : public TransformHomogeneous<Type_imu_link_X_fr_RH_KFE>
    {
        Type_imu_link_X_fr_RH_KFE();
        const Type_imu_link_X_fr_RH_KFE& update(const state_t&);
    };
    
    struct Type_fr_LF_HIP_X_fr_base : public TransformHomogeneous<Type_fr_LF_HIP_X_fr_base>
    {
        Type_fr_LF_HIP_X_fr_base();
        const Type_fr_LF_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LF_HIP : public TransformHomogeneous<Type_fr_base_X_fr_LF_HIP>
    {
        Type_fr_base_X_fr_LF_HIP();
        const Type_fr_base_X_fr_LF_HIP& update(const state_t&);
    };
    
    struct Type_fr_LF_THIGH_X_fr_LF_HIP : public TransformHomogeneous<Type_fr_LF_THIGH_X_fr_LF_HIP>
    {
        Type_fr_LF_THIGH_X_fr_LF_HIP();
        const Type_fr_LF_THIGH_X_fr_LF_HIP& update(const state_t&);
    };
    
    struct Type_fr_LF_HIP_X_fr_LF_THIGH : public TransformHomogeneous<Type_fr_LF_HIP_X_fr_LF_THIGH>
    {
        Type_fr_LF_HIP_X_fr_LF_THIGH();
        const Type_fr_LF_HIP_X_fr_LF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LF_SHANK_X_fr_LF_THIGH : public TransformHomogeneous<Type_fr_LF_SHANK_X_fr_LF_THIGH>
    {
        Type_fr_LF_SHANK_X_fr_LF_THIGH();
        const Type_fr_LF_SHANK_X_fr_LF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LF_THIGH_X_fr_LF_SHANK : public TransformHomogeneous<Type_fr_LF_THIGH_X_fr_LF_SHANK>
    {
        Type_fr_LF_THIGH_X_fr_LF_SHANK();
        const Type_fr_LF_THIGH_X_fr_LF_SHANK& update(const state_t&);
    };
    
    struct Type_fr_RF_HIP_X_fr_base : public TransformHomogeneous<Type_fr_RF_HIP_X_fr_base>
    {
        Type_fr_RF_HIP_X_fr_base();
        const Type_fr_RF_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RF_HIP : public TransformHomogeneous<Type_fr_base_X_fr_RF_HIP>
    {
        Type_fr_base_X_fr_RF_HIP();
        const Type_fr_base_X_fr_RF_HIP& update(const state_t&);
    };
    
    struct Type_fr_RF_THIGH_X_fr_RF_HIP : public TransformHomogeneous<Type_fr_RF_THIGH_X_fr_RF_HIP>
    {
        Type_fr_RF_THIGH_X_fr_RF_HIP();
        const Type_fr_RF_THIGH_X_fr_RF_HIP& update(const state_t&);
    };
    
    struct Type_fr_RF_HIP_X_fr_RF_THIGH : public TransformHomogeneous<Type_fr_RF_HIP_X_fr_RF_THIGH>
    {
        Type_fr_RF_HIP_X_fr_RF_THIGH();
        const Type_fr_RF_HIP_X_fr_RF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RF_SHANK_X_fr_RF_THIGH : public TransformHomogeneous<Type_fr_RF_SHANK_X_fr_RF_THIGH>
    {
        Type_fr_RF_SHANK_X_fr_RF_THIGH();
        const Type_fr_RF_SHANK_X_fr_RF_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RF_THIGH_X_fr_RF_SHANK : public TransformHomogeneous<Type_fr_RF_THIGH_X_fr_RF_SHANK>
    {
        Type_fr_RF_THIGH_X_fr_RF_SHANK();
        const Type_fr_RF_THIGH_X_fr_RF_SHANK& update(const state_t&);
    };
    
    struct Type_fr_LH_HIP_X_fr_base : public TransformHomogeneous<Type_fr_LH_HIP_X_fr_base>
    {
        Type_fr_LH_HIP_X_fr_base();
        const Type_fr_LH_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_LH_HIP : public TransformHomogeneous<Type_fr_base_X_fr_LH_HIP>
    {
        Type_fr_base_X_fr_LH_HIP();
        const Type_fr_base_X_fr_LH_HIP& update(const state_t&);
    };
    
    struct Type_fr_LH_THIGH_X_fr_LH_HIP : public TransformHomogeneous<Type_fr_LH_THIGH_X_fr_LH_HIP>
    {
        Type_fr_LH_THIGH_X_fr_LH_HIP();
        const Type_fr_LH_THIGH_X_fr_LH_HIP& update(const state_t&);
    };
    
    struct Type_fr_LH_HIP_X_fr_LH_THIGH : public TransformHomogeneous<Type_fr_LH_HIP_X_fr_LH_THIGH>
    {
        Type_fr_LH_HIP_X_fr_LH_THIGH();
        const Type_fr_LH_HIP_X_fr_LH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LH_SHANK_X_fr_LH_THIGH : public TransformHomogeneous<Type_fr_LH_SHANK_X_fr_LH_THIGH>
    {
        Type_fr_LH_SHANK_X_fr_LH_THIGH();
        const Type_fr_LH_SHANK_X_fr_LH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_LH_THIGH_X_fr_LH_SHANK : public TransformHomogeneous<Type_fr_LH_THIGH_X_fr_LH_SHANK>
    {
        Type_fr_LH_THIGH_X_fr_LH_SHANK();
        const Type_fr_LH_THIGH_X_fr_LH_SHANK& update(const state_t&);
    };
    
    struct Type_fr_RH_HIP_X_fr_base : public TransformHomogeneous<Type_fr_RH_HIP_X_fr_base>
    {
        Type_fr_RH_HIP_X_fr_base();
        const Type_fr_RH_HIP_X_fr_base& update(const state_t&);
    };
    
    struct Type_fr_base_X_fr_RH_HIP : public TransformHomogeneous<Type_fr_base_X_fr_RH_HIP>
    {
        Type_fr_base_X_fr_RH_HIP();
        const Type_fr_base_X_fr_RH_HIP& update(const state_t&);
    };
    
    struct Type_fr_RH_THIGH_X_fr_RH_HIP : public TransformHomogeneous<Type_fr_RH_THIGH_X_fr_RH_HIP>
    {
        Type_fr_RH_THIGH_X_fr_RH_HIP();
        const Type_fr_RH_THIGH_X_fr_RH_HIP& update(const state_t&);
    };
    
    struct Type_fr_RH_HIP_X_fr_RH_THIGH : public TransformHomogeneous<Type_fr_RH_HIP_X_fr_RH_THIGH>
    {
        Type_fr_RH_HIP_X_fr_RH_THIGH();
        const Type_fr_RH_HIP_X_fr_RH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RH_SHANK_X_fr_RH_THIGH : public TransformHomogeneous<Type_fr_RH_SHANK_X_fr_RH_THIGH>
    {
        Type_fr_RH_SHANK_X_fr_RH_THIGH();
        const Type_fr_RH_SHANK_X_fr_RH_THIGH& update(const state_t&);
    };
    
    struct Type_fr_RH_THIGH_X_fr_RH_SHANK : public TransformHomogeneous<Type_fr_RH_THIGH_X_fr_RH_SHANK>
    {
        Type_fr_RH_THIGH_X_fr_RH_SHANK();
        const Type_fr_RH_THIGH_X_fr_RH_SHANK& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_X_LF_FOOT fr_base_X_LF_FOOT;
    Type_fr_base_X_RF_FOOT fr_base_X_RF_FOOT;
    Type_fr_base_X_LH_FOOT fr_base_X_LH_FOOT;
    Type_fr_base_X_RH_FOOT fr_base_X_RH_FOOT;
    Type_imu_link_X_LF_FOOT imu_link_X_LF_FOOT;
    Type_imu_link_X_RF_FOOT imu_link_X_RF_FOOT;
    Type_imu_link_X_LH_FOOT imu_link_X_LH_FOOT;
    Type_imu_link_X_RH_FOOT imu_link_X_RH_FOOT;
    Type_fr_base_X_fr_LF_HAA fr_base_X_fr_LF_HAA;
    Type_fr_base_X_fr_LF_HFE fr_base_X_fr_LF_HFE;
    Type_fr_base_X_fr_LF_KFE fr_base_X_fr_LF_KFE;
    Type_fr_base_X_fr_RF_HAA fr_base_X_fr_RF_HAA;
    Type_fr_base_X_fr_RF_HFE fr_base_X_fr_RF_HFE;
    Type_fr_base_X_fr_RF_KFE fr_base_X_fr_RF_KFE;
    Type_fr_base_X_fr_LH_HAA fr_base_X_fr_LH_HAA;
    Type_fr_base_X_fr_LH_HFE fr_base_X_fr_LH_HFE;
    Type_fr_base_X_fr_LH_KFE fr_base_X_fr_LH_KFE;
    Type_fr_base_X_fr_RH_HAA fr_base_X_fr_RH_HAA;
    Type_fr_base_X_fr_RH_HFE fr_base_X_fr_RH_HFE;
    Type_fr_base_X_fr_RH_KFE fr_base_X_fr_RH_KFE;
    Type_imu_link_X_fr_LF_HAA imu_link_X_fr_LF_HAA;
    Type_imu_link_X_fr_LF_HFE imu_link_X_fr_LF_HFE;
    Type_imu_link_X_fr_LF_KFE imu_link_X_fr_LF_KFE;
    Type_imu_link_X_fr_RF_HAA imu_link_X_fr_RF_HAA;
    Type_imu_link_X_fr_RF_HFE imu_link_X_fr_RF_HFE;
    Type_imu_link_X_fr_RF_KFE imu_link_X_fr_RF_KFE;
    Type_imu_link_X_fr_LH_HAA imu_link_X_fr_LH_HAA;
    Type_imu_link_X_fr_LH_HFE imu_link_X_fr_LH_HFE;
    Type_imu_link_X_fr_LH_KFE imu_link_X_fr_LH_KFE;
    Type_imu_link_X_fr_RH_HAA imu_link_X_fr_RH_HAA;
    Type_imu_link_X_fr_RH_HFE imu_link_X_fr_RH_HFE;
    Type_imu_link_X_fr_RH_KFE imu_link_X_fr_RH_KFE;
    Type_fr_LF_HIP_X_fr_base fr_LF_HIP_X_fr_base;
    Type_fr_base_X_fr_LF_HIP fr_base_X_fr_LF_HIP;
    Type_fr_LF_THIGH_X_fr_LF_HIP fr_LF_THIGH_X_fr_LF_HIP;
    Type_fr_LF_HIP_X_fr_LF_THIGH fr_LF_HIP_X_fr_LF_THIGH;
    Type_fr_LF_SHANK_X_fr_LF_THIGH fr_LF_SHANK_X_fr_LF_THIGH;
    Type_fr_LF_THIGH_X_fr_LF_SHANK fr_LF_THIGH_X_fr_LF_SHANK;
    Type_fr_RF_HIP_X_fr_base fr_RF_HIP_X_fr_base;
    Type_fr_base_X_fr_RF_HIP fr_base_X_fr_RF_HIP;
    Type_fr_RF_THIGH_X_fr_RF_HIP fr_RF_THIGH_X_fr_RF_HIP;
    Type_fr_RF_HIP_X_fr_RF_THIGH fr_RF_HIP_X_fr_RF_THIGH;
    Type_fr_RF_SHANK_X_fr_RF_THIGH fr_RF_SHANK_X_fr_RF_THIGH;
    Type_fr_RF_THIGH_X_fr_RF_SHANK fr_RF_THIGH_X_fr_RF_SHANK;
    Type_fr_LH_HIP_X_fr_base fr_LH_HIP_X_fr_base;
    Type_fr_base_X_fr_LH_HIP fr_base_X_fr_LH_HIP;
    Type_fr_LH_THIGH_X_fr_LH_HIP fr_LH_THIGH_X_fr_LH_HIP;
    Type_fr_LH_HIP_X_fr_LH_THIGH fr_LH_HIP_X_fr_LH_THIGH;
    Type_fr_LH_SHANK_X_fr_LH_THIGH fr_LH_SHANK_X_fr_LH_THIGH;
    Type_fr_LH_THIGH_X_fr_LH_SHANK fr_LH_THIGH_X_fr_LH_SHANK;
    Type_fr_RH_HIP_X_fr_base fr_RH_HIP_X_fr_base;
    Type_fr_base_X_fr_RH_HIP fr_base_X_fr_RH_HIP;
    Type_fr_RH_THIGH_X_fr_RH_HIP fr_RH_THIGH_X_fr_RH_HIP;
    Type_fr_RH_HIP_X_fr_RH_THIGH fr_RH_HIP_X_fr_RH_THIGH;
    Type_fr_RH_SHANK_X_fr_RH_THIGH fr_RH_SHANK_X_fr_RH_THIGH;
    Type_fr_RH_THIGH_X_fr_RH_SHANK fr_RH_THIGH_X_fr_RH_SHANK;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
