#include "mav_state_est/update_history.hpp"

namespace MavStateEst {

updateHistory::updateHistory(RBISUpdateInterface * init)
{
  updateMap.insert(historyPair(init->utime, init));
}
updateHistory::~updateHistory()
{
  for (historyMapIterator it = updateMap.begin(); it != updateMap.end(); it++)
    delete it->second;
  updateMap.clear();
}

updateHistory::historyMapIterator updateHistory::addToHistory(RBISUpdateInterface * rbisu)
{
  int64_t prev_head_utime = updateMap.rbegin()->first;
  int64_t diff_utime = rbisu->utime - prev_head_utime;

//  fprintf(stderr, "update %s had timestamp %jd, which was %.3f from previous head %s\n",
//      RBISUpdateInterface::sensor_enum_strings[rbisu->sensor_id], rbisu->utime,
//      (double) diff_utime / 1.0e6,
//      RBISUpdateInterface::sensor_enum_strings[updateMap.rbegin()->second->sensor_id]);

  historyMapIterator it = updateMap.insert(updateMap.end(), historyPair(rbisu->utime, rbisu));

  if (it == updateMap.begin()) {
    historyMapIterator prev_head_it = updateMap.begin();
    prev_head_it++;
    RBISUpdateInterface * prev_head = prev_head_it->second;
    fprintf(stderr, "error: update type %s had timestamp %jd, which was before the first in history (%s, %jd)\n",
        RBISUpdateInterface::sensor_enum_strings[rbisu->sensor_id], rbisu->utime,
        RBISUpdateInterface::sensor_enum_strings[prev_head->sensor_id], prev_head->utime);
    fprintf(stderr, "discarding update!\n");
    delete rbisu;
    updateMap.erase(it);
    return updateMap.end();
  }

  return it;
}

void updateHistory::clearHistoryBeforeUtime(int64_t utime)
{
  historyMapIterator it_before;
  bool valid = stl_utils::stlmultimap_get_lower(updateMap, utime, it_before);
  if (!valid || it_before == updateMap.begin())
    return;

  for (historyMapIterator it = updateMap.begin(); it != it_before; it++)
    delete it->second;
  updateMap.erase(updateMap.begin(), it_before);
}
}
