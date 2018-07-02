/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_PROFILER_H
#define TAGSLAM_PROFILER_H

#include <iostream>
#include <boost/chrono.hpp>
#include <boost/unordered_map.hpp>

namespace tagslam {
  class Profiler {
  public:
    Profiler() {};
    virtual ~Profiler() {};

    void reset() {
      last_ = boost::chrono::high_resolution_clock::now();
    }
    int record(const char *label, int ncount = 1) {
      boost::chrono::high_resolution_clock::time_point now =
				boost::chrono::high_resolution_clock::now();
      Duration usec =	boost::chrono::duration_cast<Duration>(now - last_);
      ProfilerMap::iterator i = map_.find(label);
      if (i == map_.end()) {
        map_[label] = PTimer(usec, ncount);
      } else {
        map_[label] = PTimer(usec, i->second, ncount);
      }
      last_ = now;
      return (usec.count());
    }
    friend std::ostream &operator<<(std::ostream& os, const Profiler &p);
  private:
    typedef boost::chrono::duration<long long, boost::micro> Duration;
    struct PTimer {
      PTimer(const Duration d = Duration(0), int ncount = 1);
      PTimer(const Duration &d, const PTimer&oldTimer, int ncount = 1);
      Duration	duration;		// sum of durations
      int64_t		sqduration;	// sum of squared durations
      Duration	min;				// smallest duration
      Duration	max;				// largest duration
      int64_t		count;			// number of samples
    };
    typedef boost::chrono::high_resolution_clock::time_point TimePoint;
    typedef boost::unordered_map<const char *, PTimer> ProfilerMap;
    TimePoint 		last_;
    ProfilerMap 	map_;
  };
  std::ostream &operator<<(std::ostream& os, const Profiler &p);
}


#endif
