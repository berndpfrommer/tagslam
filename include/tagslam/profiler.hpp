/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <iostream>
#include <boost/chrono.hpp>
#include <boost/unordered_map.hpp>

namespace tagslam {
  class Profiler {
  public:
    typedef boost::chrono::high_resolution_clock::time_point TimePoint;
    typedef boost::chrono::duration<long long, boost::micro> Duration;
    Profiler() {};
    virtual ~Profiler() {};

    void reset(const char *label) {
      const auto t = boost::chrono::high_resolution_clock::now();
      ProfilerMap::iterator i = map_.find(label);
      if (i == map_.end()) {
        map_[label] = MapEntry(PTimer(), t);
      } else {
        i->second.lastTime = t;
      }
    }
    int record(const char *label, int ncount = 1) {
      ProfilerMap::iterator i = map_.find(label);
      if (i == map_.end()) {
        std::cout << "ERROR: invalid timer: " << label << std::endl;
        throw std::runtime_error("invalid timer!");
      }
      auto &me = i->second;
      const TimePoint now =	boost::chrono::high_resolution_clock::now();
      const Duration usec =	boost::chrono::duration_cast<Duration>(now - me.lastTime);
      me.timer = PTimer(usec, me.timer, ncount);
      me.lastTime = now;
      return (usec.count());
    }
    friend std::ostream &operator<<(std::ostream& os, const Profiler &p);
  private:
    struct PTimer {
      PTimer();
      PTimer(const Duration &d, const PTimer&oldTimer, int ncount = 1);
      Duration	duration;		// sum of durations
      int64_t		sqduration;	// sum of squared durations
      Duration	min;				// smallest duration
      Duration	max;				// largest duration
      int64_t		count;			// number of samples
    };
    struct MapEntry {
      MapEntry(const PTimer &p = PTimer(), const TimePoint &t = TimePoint()) :
        timer(p), lastTime(t) {}
      PTimer    timer;
      TimePoint lastTime;
    };
    typedef boost::unordered_map<const char *, MapEntry> ProfilerMap;
    ProfilerMap 	map_;
  };
  std::ostream &operator<<(std::ostream& os, const Profiler &p);
}
