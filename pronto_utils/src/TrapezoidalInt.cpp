#include "pronto_utils/TrapezoidalInt.hpp"
#include <iostream>

namespace pronto {

  TrapezoidalInt::TrapezoidalInt() {
	setSize(1);
	u_stime = 0;
	Init();
}


void TrapezoidalInt::Init() {
	first_pass = true;
	size_set = false;
	int_dx.setZero();
	prev_dx.setZero();
}

/*
TrapezoidalInt::~TrapezoidalInt() {

}
*/

void TrapezoidalInt::setSize(const int &s) {
	assert(s>-1);
	size_set = true;
	size = s;
	int_dx.resize(s);
	prev_dx.resize(s);

	prev_dx.setZero();
	reset_in_time();

}

void TrapezoidalInt::reset_in_time() {
	int_dx.setZero();
	//std::cout << "zeroed";
}

Eigen::VectorXd TrapezoidalInt::getVal() {
	return int_dx;
}

void TrapezoidalInt::setVal(const Eigen::VectorXd &val) {
	int_dx = val;
}

void TrapezoidalInt::setStateTo(const Eigen::VectorXd &set_val) {
	int_dx = set_val;
	prev_dx = set_val;

}

Eigen::VectorXd TrapezoidalInt::integrate(const unsigned long long &u_ts, const Eigen::VectorXd &dx) {
	// TODO -- This function should use the u_time stamp from zero. Then it can also be used as an integral time counter and makes best possible use of the available time variable dynamic range

	if (u_ts < u_stime) {
		std::cout << "TrapezoidalInt::integrate is jumping back in time. This was not expected -- behavior will be unpredictable.\n";
		u_stime = u_ts;
	}
	if (first_pass) {
		u_stime = u_ts;
		first_pass = false;
	}

	// Eigen does not ensure self assigned computations x = x + 1
	Eigen::VectorXd newval(size);
	newval = int_dx + 0.5*(dx + prev_dx)*(u_ts-u_stime)*1E-6;
	int_dx = newval;

	u_stime = u_ts;
	prev_dx = dx;

	return int_dx;
}

Eigen::VectorXd TrapezoidalInt::integrate(const unsigned long long &ts, const int &num_joints, const double samples[]) {
	if (first_pass && !size_set) {
		setSize(num_joints);
		first_pass = false;

		std::cout << "Size of TrapezoidalInt was not set, but automatically adjusted to the first received vector size of: " << num_joints << "\n";
	}

	Eigen::VectorXd to_int(num_joints);

	for (int i=0;i<num_joints;i++) {
		to_int(i) = samples[i];
	}

	return integrate(ts, to_int);
}

}
