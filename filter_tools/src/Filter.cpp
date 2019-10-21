
#include <iostream>
#include "filter_tools/Filter.hpp"

LowPassFilter::LowPassFilter():firstsample(true) {
  
/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 14
 * Stable            : Yes
 * Linear Phase      : Yes (Type 2)
 */  
  
  tap_size_ = 14;

  filter_coeffs_ = {
    0.005271208909706,  0.05204636786996,  0.05315761628452,  0.07562063364867,
    0.09406855250555,    0.108343855546,   0.1160610649931,   0.1160610649931,
    0.108343855546,  0.09406855250555,  0.07562063364867,  0.05315761628452,
    0.05204636786996, 0.005271208909706
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

double LowPassFilter::processSample(double sample) {
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

LowPassFilter::~LowPassFilter() {
  samples_buf.clear();
  //std::cout << "Closing out LowPassFilter object\n";
}

int LowPassFilter::getTapSize() {
  return tap_size_;
}
