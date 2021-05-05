#include "pronto_utils/NumericalDiff.hpp"
#include <iostream>

namespace pronto {
NumericalDiff::NumericalDiff() {
  first_pass = true;
  prev_time = 0;
  setSize(1);
}

void NumericalDiff::setSize(int len) {
  prev_sample.resize(len);
  size = len;
}

/*
Eigen::VectorXd NumericalDiff::diff(const double &time, const Eigen::VectorXd &sample) {


	std::cout << "NumericalDiff::diff(double , Eigen::VectorXd ) - DONT use this function -- to be depreciated ASAP\n";

	return Eigen::VectorXd();
}*/


Eigen::VectorXd NumericalDiff::diff(const unsigned long long &ts, const Eigen::VectorXd &sample) {
	if (first_pass && sample.size() != size) {
		setSize(sample.size());
		first_pass = false;

		std::cout << "NumericalDiff::diff -- Size of the vector to be used automatically adjusted on the first pass.\n";
	}

  Eigen::VectorXd returnval(size);
  
  returnval = (sample - prev_sample)/(ts - prev_time)*1E6;
  
  prev_time = ts;
  prev_sample = sample;
  
  return returnval;
}

void NumericalDiff::diff(const unsigned long long &ts, int count, double sample[]) {

	Eigen::VectorXd data(count);
	Eigen::VectorXd returndata(count);


	for (int i=0;i<count;i++) {
		data(i) = sample[i];
	}

	returndata = diff(ts,data);

	for (int i=0;i<count;i++) {
		sample[i] = returndata(i);
	}
}

}
