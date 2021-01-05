#pragma once
#include <map>
#include <stdint.h>

#include "rbis_update_interface.hpp"

namespace pronto {

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

  std::string toString() const;
  std::string toString(uint64_t utime, int pos_shift = 1) const;

protected:
  template<typename K, typename val>
  bool getLower(std::multimap<K, val> & m,
  K key,
  typename std::multimap<K, val>::iterator &lb)
  {
    lb = m.lower_bound(key);
    if (lb == m.end()) {
      return false;
    }

    if (key != lb->first) {
      if (lb == m.begin())
        return false;
      else
        lb--;
    }
    return true;
  }

};

}

