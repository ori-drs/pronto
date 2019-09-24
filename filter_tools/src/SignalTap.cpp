

#include "filter_tools/SignalTap.hpp"
#include <iostream>
#include <sstream>

// #include <path_util/path_util.h>


// creates a new_logging file
DataFileLogger::DataFileLogger() {
  suppress_logger = false;
}

DataFileLogger::DataFileLogger(std::string filename) {
	suppress_logger = false;
	fs.open(filename.c_str());
}

void DataFileLogger::Open(bool not_suppress, std::string filename) {
	suppress_logger = !not_suppress;
	
	if (!suppress_logger) {
		fs.open(filename.c_str());
	}
	
}

void DataFileLogger::Close() {
  if (!suppress_logger) {
    fs.close();
  }
	
}

void DataFileLogger::log(std::string data) {
  if (!suppress_logger) {
    fs << data;
  }
}

void DataFileLogger::operator<<(std::string to_log) {
  if (!suppress_logger) {
    log(to_log);
  }
}

SchmittTrigger::SchmittTrigger() {
	Reset();
}

SchmittTrigger::SchmittTrigger(double lt, double ht, long low_delay, long high_delay) {
	setParameters(lt,ht,low_delay,high_delay);
}

void SchmittTrigger::setParameters(double lt, double ht, long low_delay, long high_delay) {
	low_threshold = lt;
	high_threshold = ht;
	low_time_delay = low_delay;
        high_time_delay = high_delay;
	Reset();
}

void SchmittTrigger::Reset() {
  current_status = false;
  previous_time = 0; // how to handle this when it gets called for the first time.. should we have a flag and then a reset status
  timer = 0;
  storedvalue = 0.;
  
  // first call flag is used to intialize the timer, to enable delay measuring
  first_call = true;
}
void SchmittTrigger::forceHigh() {
  current_status = true;
  timer = 0;
}

void SchmittTrigger::forceLow() {
  current_status = false;
  timer = 0;
}

void SchmittTrigger::UpdateState(long present_time, double value) {
  if (first_call) {
    first_call = false;
    previous_time = present_time;
  }
  if (present_time < previous_time) {
    std::cout << "SchmittTrigger::UpdateState -- Warning object is jumping back in time -- behavior unpredictable.\n";
  }

  storedvalue = value;
  //std::cout << "ST: " << value << "N , timer: " << timer << " us, status is: " << current_status << std::endl;
  
  bool verbose = false;

  // The timing logic should be rewritten with ExireTimer at the next opportune moment
  if (current_status){
    
    if (value <= low_threshold){
      //std::cout << "below threshold\n";
      if (timer > low_time_delay) {
        if (verbose) std::cout << "high state -> low trigger\n";
        current_status = false;
      } else {
        if (verbose) std::cout << "high state but clock rising for low\n";
        timer += (present_time-previous_time);
      }
    } else {
      if (verbose) std::cout << "high state, clock zero\n";
      timer = 0;
    }
  } else {
    if (value >= high_threshold) {
      
      //std::cout << "above threshold\n";
      if (timer > high_time_delay) {
        if (verbose) std::cout << "low state -> high trigger\n";
        current_status = true;
      } else {
        if (verbose) std::cout << "low state but clock rising for high\n";
        timer += (present_time-previous_time);
      }
    } else {
      if (verbose) std::cout << "low state, clock zero\n";
      timer = 0;
    }
  }
  previous_time = present_time;
}

float SchmittTrigger::getState() {
  return (current_status ? 1.f : 0.f);
}

double SchmittTrigger::getCurrentValue() {
	return storedvalue;
}

BipolarSchmittTrigger::BipolarSchmittTrigger(double lt, double ht, long low_delay, long high_delay) {
	_trigger = new SchmittTrigger(lt, ht, low_delay, high_delay);
	//_lowside  = new SchmittTrigger(-lt, -ht, delay);
	
	prev_value = 0.;
}

BipolarSchmittTrigger::~BipolarSchmittTrigger() {
	std::cout << "Closing out a BipolarSchmittTrigger object\n";
	
	delete _trigger;
	//delete _lowside;
}

// TODO -- made a second definition somewhere else, make this the only one
int sign(double value) {
	if (value >= 0)
		return 1;
	return -1;
}

void BipolarSchmittTrigger::UpdateState(long present_time, double value) {
	// Think we need to reset on a sign flip event
	//if (sign(value) != sign(prev_value)) {
	//	_trigger->Reset();
	//}
	
	_trigger->UpdateState(present_time, std::abs(value));
	//_lowside->UpdateState(present_time, value);
	prev_value = value;
}

void BipolarSchmittTrigger::Reset() {
	_trigger->Reset();
	//_lowside->Reset();
}

float BipolarSchmittTrigger::getState() {
	if (_trigger->getState() > 0.9f)
		return 1.f;
	return 0.f;
}


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


RateChange::RateChange() {
	std::cout << "RateChange -- assuming rate of 1Hz\n";
	setDesiredPeriod_us(0,(unsigned long)1E6);
}

RateChange::RateChange(const unsigned long &period_us) {
	setDesiredPeriod_us(0, period_us);
}

void RateChange::setDesiredPeriod_us(const unsigned long long &start_u_time, const unsigned long &period_us) {
	desired_period_us = period_us;
	prev_u_time = start_u_time;
}

bool RateChange::checkNewRateTrigger(const unsigned long long &cur_u_time) {

	//std::cout << " t: " << (cur_u_time - prev_u_time);

	if ((cur_u_time - prev_u_time) >= desired_period_us) {
		// This is a rate transition trigger event
		prev_u_time = cur_u_time;

		return true;
	}

	return false;
}

bool RateChange::genericRateChange(const unsigned long long &uts, const Eigen::VectorXd &samples, Eigen::VectorXd &returnval) {

	bool flag = false;
	int_vals = generic_integrator.integrate(uts, samples);

	if (checkNewRateTrigger(uts)) {
		state = generic_diff.diff(uts, int_vals);
		flag = true;
	}
	returnval = state;
	return flag;
}

void RateChange::setSize(const int &s) {
	size = s;

	generic_integrator.setSize(size);
	state.resize(size);
	int_vals.resize(size);
	generic_diff.setSize(size);
}

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


// Deprecated for now as auto isn't in C++ standard
/*
template <int N>
BlipFilter<N>::
BlipFilter() {
  setMedianFilterLength(15);
  setMinBlipMagnitude(0.005);
}

template <int N>
void BlipFilter<N>::
setMedianFilterLength(const int iLength) {
  mMedianFilterLength = iLength;
  mData.resize(mMedianFilterLength);
  for (size_t i = 0; i < mMedianFilterLength; ++i) {
    mData[i].setZero();
  }
  // TODO: maybe keep pool of pointers around
  // TODO: transient at startup
  // TODO: for now introduce delay
}

template <int N>
void BlipFilter<N>::
setMinBlipMagnitude(const double iMag) {
  mMinBlipMagnitude = iMag;
}

template <int N>
typename BlipFilter<N>::VecType BlipFilter<N>::
getValue(const VecType& iSample) {
  // keep this sample
  mData.push_back(iSample);

  // compute median over entire buffer
  VecType medianVal;
  double vals[mMedianFilterLength];
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < mMedianFilterLength; ++j) {
      vals[j] = mData[j][i];
    }
    std::sort(vals, vals+mMedianFilterLength);
    medianVal[i] = vals[mMedianFilterLength/2];
  }

  // compute differences to determine which values to replace
  const int centralIdx = mMedianFilterLength/2;
  VecType centralVal = mData[centralIdx];
  VecType predictedVal = (mData[centralIdx+1] + mData[centralIdx-1])*0.5;
  VecType predictedVsMedian = (predictedVal-medianVal).array().abs();
  VecType actualVsMedian = (centralVal-medianVal).array().abs();
  VecType predictedVsActual = (predictedVal-centralVal).array().abs();

  // replace output values that deviate too greatly
  auto condition1 = (predictedVsMedian.array() < actualVsMedian.array());
  auto condition2 = (predictedVsActual.array() > mMinBlipMagnitude);
  VecType outputVal = (condition1*condition2).select(predictedVal, centralVal);

  // done
  return outputVal;
}

// explicit instantiations of BlipFilter
template class BlipFilter<3>;
template class BlipFilter<6>;
*/

CumulativeAverage::CumulativeAverage() {
	setSize(1);
}

void CumulativeAverage::setSize(const int &s) {
	size = s;
	CA.resize(size);
	reset();
}

void CumulativeAverage::reset() {

	CA.setZero();
	elementcounter = 0;
}

Eigen::VectorXd CumulativeAverage::processSamples(const Eigen::VectorXd &val) {
	Eigen::VectorXd alias(size);

	elementcounter++;
	alias = (val + (elementcounter-1)*CA)/(elementcounter);
	CA = alias;

	return CA;
}

Eigen::VectorXd CumulativeAverage::getCA() {
	return CA;
}

ExpireTimer::ExpireTimer() {
	desired_uper = 0;
	uperiod = 0;
	firstpass = true;
	previous_uts = 0;
}

void ExpireTimer::setDesiredPeriod_us(const unsigned long &uper) {
	desired_uper = uper;
	reset();
}


void ExpireTimer::reset() {
	uperiod = desired_uper;
	firstpass = true;
}

bool ExpireTimer::processSample(const unsigned long long &uts) {
	if (firstpass) {
		// we need to grab the first timestamp here
		firstpass = false;
		previous_uts = uts;
	}

	if (uts < previous_uts) {
		std::cout << "ExpireTimer::processSample -- Jumping back in time, behavior unpredictable.\n";
		previous_uts = uts;
	}

	unsigned long delta;
	delta = (unsigned long)(uts - previous_uts);


	if (delta > uperiod) {
		// this would make the counter negative, so just saturate low
		delta = uperiod;
	}

	uperiod = uperiod - delta;
	previous_uts = uts;

	return getState();
}

unsigned long ExpireTimer::getRemainingTime_us() {
	return uperiod;
}

bool ExpireTimer::getState() {
	if (uperiod==0)
	{
		// negative values are not handled by this class. We check zero only
		return true;
	}
	return false;
}

Gaussian::Gaussian() {
	/* Setup constants */
	//	q = 15;
	//	c1 = (1 << q) - 1;
	//	c2 = ((int)(c1 / 3)) + 1;
	//	c3 = 1.f / c1;
	//	
	srand(time(0));
	//	random = 0.;
}
	
double Gaussian::randn() {
	
//	random = ((float)rand() / (float)(RAND_MAX + 1));
//	return (2.f * ((random * c2) + (random * c2) + (random * c2)) - 3.f * (c2 - 1.f)) * c3;
	
	R1 = (double) rand() / (double) RAND_MAX;
	R2 = (double) rand() / (double) RAND_MAX;

	return (double) sqrt( -2.0 * log( R1 )) * cos( 2.0 * PI__ * R2 );
	
}

