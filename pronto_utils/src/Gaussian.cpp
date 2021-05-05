#include "pronto_utils/Gaussian.hpp"
#include <cstdlib>
#include <cmath>

namespace pronto {
Gaussian::Gaussian() {
  /* Setup constants */
  //	q = 15;
  //	c1 = (1 << q) - 1;
  //	c2 = ((int)(c1 / 3)) + 1;
  //	c3 = 1.f / c1;
  //
  // srand(time(0));
  //	random = 0.;
}

double Gaussian::randn() {

//	random = ((float)rand() / (float)(RAND_MAX + 1));
//	return (2.f * ((random * c2) + (random * c2) + (random * c2)) - 3.f * (c2 - 1.f)) * c3;

  R1 = (double) std::rand() / (double) RAND_MAX;
  R2 = (double) std::rand() / (double) RAND_MAX;

  return (double) std::sqrt( -2.0 * std::log( R1 )) * std::cos( 2.0 * M_PI * R2 );

}

}
