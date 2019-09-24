
#include <iostream>
#include "filter_tools/HeavyLowPassFilter.hpp"

HeavyLowPassFilter::HeavyLowPassFilter():firstsample(true) {

/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 72
 * Stable            : Yes
 * Linear Phase      : Yes (Type 2)
 */

  tap_size_ = 72;

  filter_coeffs_ = {
  -0.008103419153409,-0.000810618990958,-0.0007494115982237,-0.000611153923444,
  -0.0003891449776452,-7.55205662672e-05,0.0003336647343425,0.0008466284776547,
   0.001464358952374, 0.002192444957178, 0.003029735989605, 0.003978601097679,
     0.0050333725847, 0.006194244699355, 0.007453718804274, 0.008808875945055,
    0.01024867727168,  0.01176740239529,  0.01333453044441,  0.01495437616932,
    0.01661611333651,  0.01828347862038,  0.01995665559979,  0.02160848514311,
    0.02322603146834,  0.02478735357424,  0.02627739990248,  0.02767530213939,
    0.02896708267334,  0.03013524332251,  0.03116688716871,  0.03204636216309,
    0.03276493696647,  0.03331108459848,  0.03367809550061,  0.03386660360657,
    0.03386660360657,  0.03367809550061,  0.03331108459848,  0.03276493696647,
    0.03204636216309,  0.03116688716871,  0.03013524332251,  0.02896708267334,
    0.02767530213939,  0.02627739990248,  0.02478735357424,  0.02322603146834,
    0.02160848514311,  0.01995665559979,  0.01828347862038,  0.01661611333651,
    0.01495437616932,  0.01333453044441,  0.01176740239529,  0.01024867727168,
   0.008808875945055, 0.007453718804274, 0.006194244699355,   0.0050333725847,
   0.003978601097679, 0.003029735989605, 0.002192444957178, 0.001464358952374,
  0.0008466284776547,0.0003336647343425,-7.55205662672e-05,-0.0003891449776452,
  -0.000611153923444,-0.0007494115982237,-0.000810618990958,-0.008103419153409
  };  
  
  
  // the above values dont sum to 1, re-normalize here!

  double  filter_sum = 0;
  for (size_t i=0; i < filter_coeffs_.size() ; i ++){
    filter_sum += filter_coeffs_[i];
  }

  for (size_t i=0; i < filter_coeffs_.size() ; i ++){
    filter_coeffs_[i]  = filter_coeffs_[i] / filter_sum;
  }
  
  // Initialize the filter memory states
  samples_buf.set_capacity(tap_size_);

  for (int i = 0; i<(tap_size_);i++)
  {
    samples_buf.push_back(0);
  }
}

double HeavyLowPassFilter::processSample(double sample) {
  if (firstsample) {
    firstsample = false;
    for (int i=0;i<tap_size_;i++) {
      samples_buf.push_back(sample); // we force the first sample into all state memory as an initial guess of there the filter should be initialized
    }
	  
  }
  // put a new element in the buffer for processing
  samples_buf.push_back(sample);
  // The new sample has been added to the buffer, now we must use the values in the buffer with the coefficients to achieve the IR filtering capabibliy
  
  double accumulator = 0.;
  //std::cout << "value in: " << sample << "\n";
  // accumulate the new composite value from all the filter coefficient and history values
  
  for (int i=0;i<tap_size_;i++) {
    //std::cout << filter_coeffs_[tap_size_-i-1]* samples_buf.at(i) << " | ";
    accumulator += filter_coeffs_[tap_size_-i-1] * samples_buf.at(i);
  }
  
  return accumulator;
}

HeavyLowPassFilter::~HeavyLowPassFilter() {
  samples_buf.clear();
  //std::cout << "Closing out HeavyLowPassFilter object\n";
}

int HeavyLowPassFilter::getTapSize() {
  return tap_size_;
}
