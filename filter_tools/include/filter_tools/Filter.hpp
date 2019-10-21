#ifndef FILTER_HPP_MF_
#define FILTER_HPP_MF_

#include <vector>
#include <boost/circular_buffer.hpp>

class LowPassFilter {
private:
  
  std::vector<double> filter_coeffs_;
  bool firstsample;
  int tap_size_;

  boost::circular_buffer<double> samples_buf;	
public:
  
  LowPassFilter();
  ~LowPassFilter();
  
  double processSample(double sample);
  
  bool getInit(){
    return firstsample; 
  }
  int getTapSize();

};

#endif /*FILTER_HPP_*/
