/*
 * mav_state_estimator.cpp
 *
 *  Created on: Dec 9, 2011
 *      Author: abry
 */

#include "pronto_core/state_est.hpp"
#include <iterator>
#include <iostream>
using namespace Eigen;

namespace pronto {

StateEstimator::StateEstimator(RBISResetUpdate * init_state,
                                     const uint64_t& history_span) :
    history(init_state),
    utime_history_span(history_span)
{
  init_state->updateFilter(RBIS(), RBIM::Zero(), 0); //apply update from zero... should reset the state
  unprocessed_updates_start = history.updateMap.end();

  std::cout << init_state->posterior_state << std::endl;
  Eigen::IOFormat clean_format(4, 0, ", ", "\n", "[", "]");
  std::cout << init_state->posterior_covariance.format(clean_format) << std::endl;
}

StateEstimator::~StateEstimator()
{
    // the updateHistory class is taking care of memory deallocation of the
    // list of measurement pointers. Nothing to do here.

}

bool StateEstimator::getInterpolatedPose(const uint64_t &utime, Eigen::Isometry3d& pose) const {
    Vector3d pos;
    Quaterniond orient;
    if(!getInterpolatedPose(utime, pos, orient)){
        return false;
    }
    pose = Eigen::Isometry3d::Identity();
    pose.translate(pos);
    pose.rotate(orient);
    return true;
}

bool StateEstimator::getInterpolatedPose(const uint64_t &utime,
                                            Vector3d &position,
                                            Quaterniond &orientation) const
{
    if(history.updateMap.empty() || static_cast<int64_t>(utime) < history.updateMap.begin()->first || static_cast<int64_t>(utime) > history.updateMap.rbegin()->first)
    {
        return false;
    }
    // this is the first element greater than utime
    updateHistory::historyMap::const_iterator it_low = history.updateMap.upper_bound(utime);
    // if we have reached the bottom already, we return
    if(it_low == history.updateMap.end()){
        return false;
    }
    // now it_low contains the last element smaller than utime
    --it_low;
    // if by chance we have the pose at that exact time, we return it
    if(it_low->first == static_cast<int64_t>(utime)){
        RBISUpdateInterface * head_update = it_low->second;
        position = head_update->posterior_state.getPoseAsIsometry3d().translation();
        orientation = Quaterniond(head_update->posterior_state.getPoseAsIsometry3d().rotation());
        return true;
    }
    // at this point we have to interpolate, and we can't do it with less than
    // two items in history
    if(history.updateMap.size() < 2){
        return false;
    }
    // it_high is the last element with utime greater than it_low


    updateHistory::historyMap::const_iterator it_high = std::prev(history.updateMap.upper_bound((std::next(it_low,1))->first),1);

    // alpha is 1 if the requested time coincides with it_low, 0 if equal to it_high
    double alpha = (double)(it_high->first - utime) / (double)(it_high->first - it_low->first);

    Eigen::Isometry3d iso_low = (it_low->second)->posterior_state.getPoseAsIsometry3d();
    Eigen::Isometry3d iso_high = (it_high->second)->posterior_state.getPoseAsIsometry3d();

    position = iso_low.translation() * alpha + iso_high.translation() * (1-alpha);
    // in slerp, the paramter t is used as the opposite of alpha
    // (1 - t) * p0 + t * p1
    orientation = Quaterniond(iso_low.rotation()).slerp(1 - alpha, Quaterniond(iso_high.rotation()));
    return true;
}

void StateEstimator::addUpdate(RBISUpdateInterface * update, bool roll_forward)
{
#if DEBUG_MODE
   if(verbose_){
       std::cout << "[ " << update->utime <<" ] "<< update->getSensorIdString() << std::endl;
   }
 // Add current update to history

  if(update->sensor_id == RBISUpdateInterface::scan_matcher){
    std::cerr << "==================== RECEIVED SCAN MATCHER UPDATE" << std::endl;
    std::cerr << "HISTORY BEFORE: " << std::endl;
    std::cerr << history.toString(update->utime,3) << std::endl;
  }
#endif
  updateHistory::historyMapIterator added_it = history.addToHistory(update);
#if DEBUG_MODE
  if(update->sensor_id == RBISUpdateInterface::scan_matcher){
    std::cerr << "HISTORY AFTER: " << std::endl;
    std::cerr << history.toString(update->utime,3) << std::endl;
  }
#endif

  // If there are no unprocessed updates other than the current one,
  // the current one only is where the "unprocessed" queue starts
  if (unprocessed_updates_start == history.updateMap.end() ||
      added_it->first < unprocessed_updates_start->first) {
    unprocessed_updates_start = added_it;
  }
  if (!roll_forward) {
    return;
  }

  // Get the update before the first unprocessed update
  updateHistory::historyMapIterator prev_it = unprocessed_updates_start;
  prev_it--;
  updateHistory::historyMapIterator current_it = unprocessed_updates_start;
  //  fprintf(stderr, "roll forward: %s ", RBISUpdateInterface::sensor_enum_strings[prev_it->second->sensor_id]);

  // Iterate over unprocessed updates
  while (current_it != history.updateMap.end()) {
    RBISUpdateInterface * current_update = current_it->second;
    RBISUpdateInterface * prev_update = prev_it->second;

    // The prior is the previous posterior
    current_update->updateFilter(prev_update->posterior_state,
                                 prev_update->posterior_covariance,
                                 prev_update->loglikelihood);

    // Update the time for the current posterior
    current_update->posterior_state.utime = current_update->utime;

    // if (current_update->posterior_state.hasNan()){
    //   fprintf(stderr,"ERROR: %s Update Made state NAN!\n", 
    //       RBISUpdateInterface::sensor_enum_strings[current_update->sensor_id]);
    // }

    // fprintf(stderr, "->%s  ", RBISUpdateInterface::sensor_enum_strings[current_update->sensor_id]);
    prev_it = current_it;
    current_it++;
  }
  //  fprintf(stderr, "\n");

  // The newest time is the previous iterator, because the current is end()
  int64_t newest_utime = prev_it->first;
  int64_t oldest_utime = newest_utime - this->utime_history_span;
  // Resize the history
  history.clearHistoryBeforeUtime(oldest_utime);
  // Have now rolled forward
  unprocessed_updates_start = history.updateMap.end();
}

void StateEstimator::getHeadState(RBIS & head_state, RBIM & head_cov) const
{
  RBISUpdateInterface * head_update = history.updateMap.rbegin()->second;
  head_state = head_update->posterior_state;
  head_cov = head_update->posterior_covariance;
}

void StateEstimator::getHeadState(const uint64_t& utime,
                                     RBIS &head_state,
                                     RBIM &head_cov) const {
    auto head_update = history.updateMap.lower_bound(utime)->second;
    head_state = head_update->posterior_state;
    head_cov = head_update->posterior_covariance;
}

double StateEstimator::getMeasurementsLogLikelihood() const
{
  RBISUpdateInterface * head_update = history.updateMap.rbegin()->second;
  return head_update->loglikelihood;
}

void StateEstimator::EKFSmoothBackwardsPass(double dt)
{
  /**
   * EKF smoother
   *
   * We iterate backwards through the history, applying the smoothing steps at INS updates
   * If measurements have occurred after an INS update we associate the posterior of the last measurement
   * (before another INS update) with the preceding ins update.
   * This is tracked with the measurement_cur_step boolean
   */

  updateHistory::historyMapIterator current_it = unprocessed_updates_start;
  current_it--; //start with latest processed update

  bool measurement_cur_step = false;
  RBIS next_state;
  RBIM next_cov;

  RBISUpdateInterface * current_update = current_it->second;

  //rewind through measurements
  while (current_update->sensor_id != RBISUpdateInterface::ins) {
    current_update = current_it->second;
    if (!measurement_cur_step) {
      next_state = current_update->posterior_state;
      next_cov = current_update->posterior_covariance;
      measurement_cur_step = true;
    }
    current_it--;
  }

  RBIS next_state_pred = current_update->posterior_state;
  RBIM next_cov_pred = current_update->posterior_covariance;

  if (!measurement_cur_step) {
    next_state = current_update->posterior_state;
    next_cov = current_update->posterior_covariance;
  }
  current_it--;

  RBIS cur_state;
  RBIM cur_cov;
  RBIS cur_state_pred;
  RBIM cur_cov_pred;
  measurement_cur_step = false;

  updateHistory::historyMapIterator before_begin = history.updateMap.begin();
  before_begin--; //FIXME not sure if this is valid
  while (before_begin != current_it) {
    current_update = current_it->second;

    if (current_update->sensor_id == RBISUpdateInterface::ins) {
      //set the predicted values for next of the current (not used in this iteration)
      cur_state_pred = current_update->posterior_state;
      cur_cov_pred = current_update->posterior_covariance;

      if (!measurement_cur_step) {
        cur_state = cur_state_pred;
        cur_cov = cur_cov_pred;
      }
      ekfSmoothingStep(next_state_pred, next_cov_pred, next_state, next_cov, dt, cur_state, cur_cov);

      current_update->posterior_covariance = cur_cov;
      current_update->posterior_state = cur_state;

      //copy the smoothed estimate to any subsequent measurements which came at this "timestep"
      updateHistory::historyMapIterator forward_it = current_it;
      forward_it++;
      while (forward_it->second->sensor_id != RBISUpdateInterface::ins) {
        forward_it->second->posterior_state = current_update->posterior_state;
        forward_it->second->posterior_covariance = current_update->posterior_covariance;
        forward_it++;
      }

      //reset for previous filter iteration
      next_state = cur_state;
      next_cov = cur_cov;
      next_state_pred = cur_state_pred;
      next_cov_pred = cur_cov_pred;
      measurement_cur_step = false;

    }
    else {
      if (!measurement_cur_step) {
        cur_state = current_update->posterior_state;
        cur_cov = current_update->posterior_covariance;
        measurement_cur_step = true;
      }
    }
    current_it--;
  }
}

}
