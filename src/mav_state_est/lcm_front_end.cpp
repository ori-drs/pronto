#include "mav_state_est/lcm_front_end.hpp"

using namespace std;
using namespace Eigen;

namespace MavStateEst {

LCMFrontEnd::LCMFrontEnd(const std::string & in_log_fname, const std::string & out_log_fname,
    const std::string & param_fname, const std::string & param_override_str,
    const std::string & begin_timestamp, double processing_rate)
{

  state_estimator = NULL;

  bool running_from_log = !in_log_fname.empty();
  bool running_to_log = !out_log_fname.empty();

  if (running_from_log && in_log_fname == out_log_fname) {
    fprintf(stderr, "must specify different logname for output %s!\n", out_log_fname.c_str());
    exit(1);
  }

  if (running_from_log) {
    printf("running from log file: %s\n", in_log_fname.c_str());
    //std::string lcmurl = "file://" + in_log_fname + "?speed=0";
    std::stringstream lcmurl;
    lcmurl << "file://" << in_log_fname << "?speed=" << processing_rate << "&start_timestamp=" << begin_timestamp;
    lcm_recv = new lcm::LCM(lcmurl.str());
    if (!lcm_recv->good()) {
      fprintf(stderr, "Error couldn't load log file %s\n", lcmurl.str().c_str());
      exit(1);
    }
  }
  else {
    lcm_recv = new lcm::LCM(bot_lcm_get_global(NULL));
  }

  if (running_to_log) {
    printf("publishing into log file: %s\n", out_log_fname.c_str());
    std::string lcmurl = "file://" + out_log_fname + "?mode=w";
    lcm_pub = new lcm::LCM(lcmurl);
    if (!lcm_pub->good()) {
      fprintf(stderr, "Error couldn't open log file %s\n", lcmurl.c_str());
      exit(1);
    }
  }
  else {
    lcm_pub = new lcm::LCM(); // mfallon publish back to lcm if run from log
  }

  if (param_fname.empty()) {
    param = bot_param_get_global(lcm_pub->getUnderlyingLCM(), 0);
  }
  else {
    param = bot_param_new_from_file(param_fname.c_str());
  }

  if (param == NULL) {
    exit(1);
  }

  else if (!param_override_str.empty()) {
    int ret = bot_param_override_local_params(param, param_override_str.c_str());
    if (ret <= 0) {
      fprintf(stderr, "Error overriding params with %s\n", param_override_str.c_str());
      exit(1);
    }
  }

  char** active_sensor_names = bot_param_get_str_array_alloc(param, "state_estimator.active_sensors");
  if (active_sensor_names == NULL) {
    fprintf(stderr, "Error: must specify active sensors using key state_estimator.active_sensors\n");
    exit(1);
  }
  else {
    for (int i = 0; active_sensor_names[i]; i++) {
      active_sensors.insert(std::string(active_sensor_names[i]));
    }
  }
  bot_param_str_array_free(active_sensor_names);

  frames = bot_frames_get_global(lcm_recv->getUnderlyingLCM(), param);

  filter_state_channel = bot_param_get_str_or_fail(param, "state_estimator.filter_state_channel");
  pose_channel = bot_param_get_str_or_fail(param, "state_estimator.pose_channel");
  publish_filter_state = bot_param_get_boolean_or_fail(param, "state_estimator.publish_filter_state");
  publish_pose = bot_param_get_boolean_or_fail(param, "state_estimator.publish_pose");
  republish_sensors = bot_param_get_boolean_or_fail(param, "state_estimator.republish_sensors");

  char *init_message_channel_char;
  char *init_complete_channel_char;

  if (bot_param_get_str(param, "state_estimator.init_message.channel", &init_message_channel_char) != 0) {
    // failed to get this key
    init_message_channel = "";
  } else {
    init_message_channel = string(init_message_channel_char);
    free(init_message_channel_char);
  }

  if (bot_param_get_str(param, "state_estimator.init_message.init_complete_channel", &init_complete_channel_char) != 0) {
    // failed to get this key
    init_complete_channel = "";
  } else {
    init_complete_channel = string(init_complete_channel_char);
    free(init_complete_channel_char);
  }




  exit_estimator = false; // when this is true, we exit the estimator handlers, mfallon
}

LCMFrontEnd::~LCMFrontEnd()
{
  std::map<std::string, SensorHandlerInterface *>::iterator it;
  for (it = sensor_handlers.begin(); it != sensor_handlers.end(); it++) {
    delete it->second;
  }
}

void LCMFrontEnd::setStateEstimator(MavStateEstimator * _state_estimator)
{
    state_estimator = _state_estimator;
}

bool LCMFrontEnd::isActive(const std::string & sensor_prefix)
{
  return active_sensors.count(sensor_prefix) > 0;
}

void LCMFrontEnd::outputLogLikeLihood(const std::string & output_fname)
{
  double log_likelihood = state_estimator->getMeasurementsLogLikelihood();
  fprintf(stderr, "total measurement log likelihood: %f\n", log_likelihood);
  if (!output_fname.empty()) {
    Eigen::Matrix<double, 1, 1> log_likelihood_mat; //FIXME set to 2x2 so the write/read in eigen_utils works
    log_likelihood_mat.setConstant(log_likelihood);
    eigen_utils::writeToFile(output_fname, log_likelihood_mat);
  }
}

void LCMFrontEnd::publishState(const RBIS & state, const RBIM & cov)
{
  if (publish_pose) {
    //publish the pose msg
    rigid_body::pose_t pose_msg = state.getPose();
    lcm_pub->publish(pose_channel, &pose_msg);
  }

  //publish filter state
  if (publish_filter_state) {
    pronto::filter_state_t fs_msg = rbisCreateFilterStateMessageCPP(state, cov);
    lcm_pub->publish(filter_state_channel, &fs_msg);
  }
}

void LCMFrontEnd::publishHead()
{

  RBIS state;
  RBIM cov;
  state_estimator->getHeadState(state, cov);
  publishState(state, cov);
}

void LCMFrontEnd::smooth(double dt)
{
  if (state_estimator == NULL) {
    fprintf(stderr, "Error: can't smooth with LCMFrontEnd before setStateEstimator\n");
    exit(1);
  }

  fprintf(stderr, "performing smoothing forward pass\n");

  int64_t override_val = numeric_limits<int64_t>::max();
  fprintf(stderr, "overriding utime history span from %jd to %jd", state_estimator->utime_history_span, override_val);
  state_estimator->utime_history_span = override_val;

  fprintf(stderr, "turning off pose and filter state publishing going forwards\n");
  bool set_publish_filter_state = publish_filter_state;
  publish_filter_state = false;
  bool set_publish_pose = publish_pose;
  publish_pose = false;

  while (true) {
    int ret = lcm_recv->handle();
    if (ret != 0) {
      printf("log is done\n");
      break;
    }
  }

  publish_filter_state = set_publish_filter_state;
  publish_pose = set_publish_pose;

  fprintf(stderr, "performing smoothing backwards pass\n");
  state_estimator->EKFSmoothBackwardsPass(dt);

  updateHistory::historyMapIterator current_it;

  fprintf(stderr, "republishing smoothed pose messages\n");
  for (current_it = state_estimator->history.updateMap.begin();
      current_it != state_estimator->history.updateMap.end(); current_it++) {
    RBISUpdateInterface * current_update = current_it->second;
    if (current_update->sensor_id == RBISUpdateInterface::ins) {
      RBIS state = current_update->posterior_state;
      RBIM cov = current_update->posterior_covariance;

      publishState(state, cov);
    }
  }
}

void LCMFrontEnd::run()
{
  if (state_estimator == NULL) {
    fprintf(stderr, "Error: can't run LCMFrontEnd before setStateEstimator\n");
    exit(1);
  }

  while ( !exit_estimator ) { // mfallon added. when true, stops estimation instance
    int ret = lcm_recv->handle();
    if (ret != 0) {
      printf("log is done\n");
      break;
    }
  }
}

} //namespace
