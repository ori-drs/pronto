#include "pronto_utils/DistributedDiff.hpp"
#include <iostream>

namespace pronto {
DistributedDiff::DistributedDiff() {
	hist_len = 0;
	size = 0;
	period = 0;
	max_hist_utime = 0;
	individual_diffs = 0;
	sizeset = false;
	hist_len_set = false;
	firstpasses = 0;
}

DistributedDiff::~DistributedDiff() {

	if (hist_len_set && sizeset) {
		for (int i=0;i<hist_len;i++) {
			delete _state_hist[i];
		}
	}

	std::cout << "DistributedDiff object was destroyed.\n";
}

void DistributedDiff::setSize(int s) {
	size = s;
	sizeset = true;

	cum_sum.resize(size);
	cum_sum_buf.resize(size);
}

void DistributedDiff::InitializeTaps(int hist_length, const long &per, const Eigen::VectorXd &w, const Eigen::VectorXd &t) {

	assert(sizeset);
	assert(hist_length>0);
	assert(per > 0);

	// we need to allocate one more memory location for the current sample. History is stored in the elements beyond that

	std::cout << "DD: hist_len is being set to: " << hist_length << std::endl;

	hist_len = hist_length+1;
	_state_hist.resize(hist_len, NULL);
	utimes.resize(hist_len, 0);
	for (int i=0;i<hist_len;i++) {
		_state_hist[i] = new Eigen::VectorXd(size);
		_state_hist[i]->setZero();
	}

	//std::cout << "init value of 2 is: " <<  *_state_hist[2] << std::endl;

	hist_len_set = true;

	weights = w;
	timespans = t;
	period = per;
	max_hist_utime = hist_len*period;

	individual_diffs = timespans.size();

	firstpasses = hist_len;

}

void DistributedDiff::ParameterFileInit(std::string path_to_weights) {
	long period_;

	char status;

	FILE *fp;
        
        // mfallon removed call to getConfigPath() here:
	std::string weightsFile = std::string(path_to_weights) + "/subsystems/legged_odometry/weights.txt";

	if( (fp = fopen(weightsFile.c_str(), "r+")) == NULL)
	{
		std::cout << "weights.txt not found in the current directory.\n";
		exit(1);
	}

	//int test1,test2;
    //int ret = fscanf(fp,"%ld\n",&period_,&status);
    //TODO what to do with status?
    int ret = fscanf(fp,"%ld\n",&period_);
    if(ret <= 0){
        std::cout << "Period not read" << std::endl;
    }

	std::cout << "Period was set to: " << period_ << std::endl;

	int t[100];
	float w[100];

	int i=0;

    while (fscanf(fp, "%d, %f\n", t+i, w+i) != EOF) {i++;}

    fclose(fp);

    //w[i] = atof(item);

    Eigen::VectorXd timespans(i);
    Eigen::VectorXd weights(i);

    for (int j=0;j<i;j++) {
    	timespans(j) = t[j];
    	weights(j) = w[j];

    	//std::cout << "Read values are: " << timespans(j) << ", " << weights(j) << std::endl;
    }

    // TODO
    int length_;
    //length_ = (int)((int)w[29]/period_+0.5);
    length_ = (int)(((int)timespans(timespans.size()-1)/period_) + 0.5);
    //std::cout << "InitializeTaps is assumed to take 30 weights at this point.\n";
    std::cout << "DistributedDiff::ParameterFileInit -- Calculated history memory length is: " << length_ << std::endl;
    InitializeTaps(length_, period_, weights, timespans);


}

void DistributedDiff::addDataToBuffer(const unsigned long long &u_ts, const Eigen::VectorXd &samples) {

	if (samples.size() != size) {
		std::cerr << "ERROR, DistributedDiff::addDataToBuffer -- sample vector size does not match set diff vector size. Data ignored.\n";
		return;
	}

	utimes.push_back(u_ts);

	Eigen::VectorXd *_temp;
	_temp = _state_hist.front();
	_state_hist.push_back(_temp);

	*(_state_hist.back()) = (samples);

	/*
	std::cout << "BUF: ";
	for (int i=0;i<hist_len;i++) {
		std::cout << " | " << (*(_state_hist[i]))(0);
	}
	std::cout << std::endl;
	std::cout << "UTS: ";
	for (int i=0;i<hist_len;i++) {
		std::cout << " | " << (utimes[i]);
	}
	std::cout << std::endl;
	*/
}

// find Differential between the latest sample and the desired point back in history
Eigen::VectorXd DistributedDiff::findDifferentialFromLatest(const unsigned long &desired_hist_ut) {
	assert(desired_hist_ut>0);

	unsigned long actual_delta_ut;

	// now we need to find where this sample is back in the stored history state


	Eigen::VectorXd prev_sample(size);
	prev_sample.resize(size);
	prev_sample = searchHistElement(desired_hist_ut, &actual_delta_ut);

	//double firstval = (*_state_hist.back())(0);
	//double secondval = (prev_sample)(0);
	//std::cout << "period used: " << actual_delta_ut << " diff between: " << firstval << " - " << prev_sample << " = " << (firstval - prev_sample(0))/(double)actual_delta_ut*1E6;

	return (*(_state_hist.back()) - prev_sample) / ((double) actual_delta_ut*1.E-6);
}

bool DistributedDiff::ready() {
	return (sizeset && hist_len_set);
}

Eigen::VectorXd DistributedDiff::FindDifferentials() {

	// Eigen does not ensure x = x + a
	cum_sum.setZero();

	Eigen::VectorXd temp(size);

	for (int i=0;i<individual_diffs;i++) {
		temp = findDifferentialFromLatest(timespans(i));
		//std::cout << " | weight " << weights(i) << std::endl;

		if ((firstpasses-hist_len+i)>=0) {
			temp.setZero();
		}

		cum_sum_buf = cum_sum + weights(i)*temp;
		cum_sum = cum_sum_buf;
	}

	if (firstpasses>0) {
		firstpasses--;
	}

	return cum_sum;
}

Eigen::VectorXd DistributedDiff::searchHistElement(const unsigned long &delta_u_ts, unsigned long *actual_delta_u_ts) {

	if (delta_u_ts<=0) {
		std::cout << "DistributedDiff::searchHistElement -- Can not differentiate into the future, we do not know what those values are going to be yet\n";
		return Eigen::VectorXd::Zero(size);
	}

	int searchingflag = hist_len;

	int predicted_index = hist_len - (int)(delta_u_ts/period) - 2;
	if (predicted_index>= hist_len) {
		predicted_index = hist_len - 2;
	}
	if (predicted_index <=0) {
		predicted_index = 0;
	}

	/*std::cout << "BUF is: ";
	for (int i=0;i<hist_len;i++) {
		std::cout << (*_state_hist[i])<< " | ";
	}*/

	//std::cout <<std::endl;

	long delta_time;

	// may have to search up or down
 	while (searchingflag>-1) {
		searchingflag--;

		// what is the termination condition


		delta_time = (utimes.back() - utimes[predicted_index]);
		if (std::abs(static_cast<long int>(delta_time - delta_u_ts)) <= (0.5*period)) {
			// we have found it
			if (delta_time<=-((long)period)) {
				std::cerr << "DistributedDiff::searchHistElement -- bad timing: " << delta_time << ", " << (delta_time - delta_u_ts) <<  "\n";
				std::cout << "BUF is: ";
				for (int i=0;i<hist_len;i++) {
					std::cout << (utimes.back()-utimes[i]) << " | ";
				}
			}
			*actual_delta_u_ts = (unsigned long)delta_time;
			//std::cout << "A\n";
			return *(_state_hist[predicted_index]);
		} else {
			// first we can be too far back into history
			if (delta_u_ts < delta_time) {
				// reduce history search depth
				predicted_index++; // move towards the back of the circular buffer (.back() is the newest value)
				//std::cout << "Searching one newer\n";
				if ((predicted_index+1)>=hist_len) {
					// We are right at the newest data -- have to return a single period differential
					*actual_delta_u_ts = (unsigned long)(utimes.back() - utimes[hist_len-2]);
					//std::cout << "B\n";
					return *(_state_hist[hist_len-2]);
				}
			} else {
				// we may be to shallow in history
				if ((delta_time) < delta_u_ts ) {
					// not deep enough into the history -- increase depth
					predicted_index--; // move closer to the front of the circular buffer
					//std::cout << "Searching one older\n";
					if (predicted_index <= 0) {
						// there is no greater history to give, so give the max depth
						*actual_delta_u_ts = (unsigned long)(utimes.back() - utimes.front());
						//std::cout << "C\n";
						return *(_state_hist.front());
					}
				}
			}
		}
	}

#ifdef VERBOSE_DEBUG
 	if (firstpasses <= 0) {

		std::cerr << "DistributedDiff::searchHistElement -- expected data not in buffer, using nearby element -- probably dropping LCM frames due to computation or net bandwidth limitations." << std::endl;  /*something has gone wrong while trying to find delta: " << delta_u_ts << ", " << delta_time << std::endl;*/
		/*std::cout << "BUF is: ";
		for (int i=0;i<hist_len;i++) {
			std::cout << (utimes.back()-utimes[i]) << " | ";
		}
		std::cout << std::endl;*/
		//*actual_delta_u_ts = period; // the function higher up may want to divide with this delta time -- we already going to return a zero numerator
 	}
#endif

 	if ((predicted_index+1)>=hist_len) {
 		predicted_index--;
 		*actual_delta_u_ts = (unsigned long)(utimes.back() - utimes[hist_len-2]);
 		return *(_state_hist[hist_len-2]);
 	}
 	if (predicted_index<=0) {
 		*actual_delta_u_ts = (unsigned long)(utimes.back() - utimes.front());
 		return *(_state_hist.front());
 	}
 	*actual_delta_u_ts = (unsigned long)delta_time;
	return *(_state_hist[predicted_index]);
}

Eigen::VectorXd DistributedDiff::diff(const unsigned long long &u_ts, const Eigen::VectorXd &samples) {

	addDataToBuffer(u_ts, samples);

	// Next we want to calculate all the differentials
	Eigen::VectorXd diff_(size);
	diff_ = FindDifferentials();

	return diff_;
}

}
