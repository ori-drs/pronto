#ifndef RCG_ANYMAL_RBD_TYPES_H_
#define RCG_ANYMAL_RBD_TYPES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/scalar_traits.h>
#include <iit/rbd/InertiaMatrix.h>

namespace pronto {
namespace anymal {

typedef typename iit::rbd::DoubleTraits ScalarTraits;
typedef typename ScalarTraits::Scalar Scalar;

typedef iit::rbd::Core<Scalar> TypesGen;
typedef TypesGen::ForceVector     Force;
typedef TypesGen::VelocityVector  Velocity;
typedef TypesGen::VelocityVector  Acceleration;
typedef TypesGen::Matrix66        Matrix66;
typedef TypesGen::Column6D        Column6;
typedef TypesGen::Vector3         Vector3;

template<int R, int C>
using Matrix = iit::rbd::PlainMatrix<Scalar, R, C>;

using InertiaMatrix = iit::rbd::InertiaMat<Scalar>;

static const Scalar g = iit::rbd::g;

}
}
#endif
