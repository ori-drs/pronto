#pragma once

namespace pronto {
// Class to generate Gaussian random noise.
// INternal state for this noise process is self containte within the class
class Gaussian {
private:
  //	double random;
  //
  //	int q;
  //	float c1;
  //	float c2;
  //	float c3;

  double R1;
  double R2;
public:
  Gaussian();

  double randn();
};
}
