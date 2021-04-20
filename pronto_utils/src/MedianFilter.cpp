#include "pronto_utils/MedianFilter.hpp"
#include <iostream>

namespace pronto {

MedianFilter::MedianFilter() {
  len = 0;
  lengthset = false;
}

void MedianFilter::setLength(const int &length) {
  len = length;
  data.resize(len, 0.);
  std::cout << "MedianFilter with window size " << data.size() << " was set.\n";
  lengthset = true;
}

double MedianFilter::processSample(const double &sample) {

  double buffer[len];

  data.push_back(sample);

  for (int i=0;i<len;i++) {
    buffer[i] = data[i];
  }

  std::sort(buffer, buffer+len); // sort the data in the given memory location

  return buffer[(int)(len/2)];
}

}
