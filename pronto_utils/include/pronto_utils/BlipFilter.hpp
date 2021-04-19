#pragma once
#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>

namespace pronto {

template<int N>
class BlipFilter {
private:
  typedef Eigen::Matrix<double,N,1> VecType;

  int mMedianFilterLength;
  double mMinBlipMagnitude;
  boost::circular_buffer<VecType> mData;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BlipFilter();
  void setMedianFilterLength(const int iLength);
  void setMinBlipMagnitude(const double iMag);
  VecType getValue(const VecType& iSample);
};

template <int N>
BlipFilter<N>::BlipFilter() {
  setMedianFilterLength(15);
  setMinBlipMagnitude(0.005);
}

template <int N>
void BlipFilter<N>::setMedianFilterLength(const int iLength) {
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
void BlipFilter<N>::setMinBlipMagnitude(const double iMag) {
  mMinBlipMagnitude = iMag;
}

template <int N>
typename BlipFilter<N>::VecType BlipFilter<N>::getValue(const VecType& iSample) {
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

}
