#ifndef HEAVYLOWPASSFILTER_HPP_MF_
#define HEAVYLOWPASSFILTER_HPP_MF_

#include <vector>
#include <boost/circular_buffer.hpp>

class HeavyLowPassFilter {
private:
  
  std::vector<double> filter_coeffs_;
  bool firstsample;
  int tap_size_;

  boost::circular_buffer<double> samples_buf;	
public:
  
  HeavyLowPassFilter();
  ~HeavyLowPassFilter();
  
  double processSample(double sample);
  
  bool getInit(){
    return firstsample; 
  }
  int getTapSize();

};

#endif /*HEAVYLOWPASSFILTER_HPP_MF_*/
