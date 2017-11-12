/*
 * mav_state_estimator.cpp
 *
 *  Created on: Dec 9, 2011
 *      Author: abry
 */

#include "mav_state_est/mav_state_est.hpp"

namespace MavStateEst {

MavStateEstimator::MavStateEstimator(RBISResetUpdate * init_state, BotParam * param) :
    history(init_state)
{
  utime_history_span = bot_param_get_int_or_fail(param, "state_estimator.utime_history_span");
  init_state->updateFilter(RBIS(), RBIM::Zero(), 0); //apply update from zero... should reset the state
  unprocessed_updates_start = history.updateMap.end();

  eigen_dump(init_state->posterior_state);
  eigen_dump(init_state->posterior_covariance);
  printf("\n");
}

MavStateEstimator::~MavStateEstimator()
{
}

void MavStateEstimator::addUpdate(RBISUpdateInterface * update, bool roll_forward)
{
  // Add current update to history
  updateHistory::historyMapIterator added_it = history.addToHistory(update);

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

void MavStateEstimator::getHeadState(RBIS & head_state, RBIM & head_cov)
{
  RBISUpdateInterface * head_update = history.updateMap.rbegin()->second;
  head_state = head_update->posterior_state;
  head_cov = head_update->posterior_covariance;

//  eigen_dump(head_state);
//  eigen_dump(head_cov);
}

double MavStateEstimator::getMeasurementsLogLikelihood()
{
  RBISUpdateInterface * head_update = history.updateMap.rbegin()->second;
  return head_update->loglikelihood;
}

void MavStateEstimator::EKFSmoothBackwardsPass(double dt)
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
