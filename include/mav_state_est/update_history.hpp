#ifndef __update_history_h__
#define __update_history_h__

#include <map>
#include <stl_utils/stlmap_utils.hpp>
#include <stdint.h>

#include "rbis_update_interface.hpp"

namespace MavStateEst {

class updateHistory {
public:

  typedef std::multimap<int64_t, RBISUpdateInterface *> historyMap;
  typedef historyMap::iterator historyMapIterator;
  typedef std::pair<int64_t, RBISUpdateInterface*> historyPair;
  historyMap updateMap;

  updateHistory(RBISUpdateInterface * init);
  ~updateHistory();

  /**
   * add object to history
   *
   * returns iterator to obj in the history map
   */
  historyMapIterator addToHistory(RBISUpdateInterface * rbisu);

  /**
   * Clear everything in the history older than the given timestamp
   */
  void clearHistoryBeforeUtime(int64_t utime);

};

}

#endif
