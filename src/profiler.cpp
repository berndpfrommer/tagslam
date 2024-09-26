// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <math.h>

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <tagslam/profiler.hpp>

namespace tagslam
{
std::ostream & operator<<(std::ostream & os, const Profiler & p)
{
  std::multimap<int64_t, const char *> avgToStr;
  size_t maxlen(0);
  for (const Profiler::ProfilerMap::value_type & v : p.map_) {
    const Profiler::PTimer & pt = v.second.timer;
    int64_t dn = (pt.count > 0) ? (pt.duration / pt.count).count() : -1;
    avgToStr.insert(std::pair<int64_t, const char *>(dn, v.first));
    maxlen = std::max(maxlen, strlen(v.first));
  }

  for (const auto & mi : avgToStr) {
    Profiler::ProfilerMap::const_iterator it = p.map_.find(mi.second);
    if (it == p.map_.end()) {
      std::cout << "ERROR: cannot find key: " << mi.second << std::endl;
      continue;
    }
    const Profiler::PTimer & pt = it->second.timer;
    int64_t dn = (pt.count > 0) ? (pt.duration / pt.count).count() : -1;
    int64_t sumdelta = pt.duration.count();
    // too much roundoff error when doing this
    // int64_t sqdn = (pt.count > 0) ? ((pt.sqduration / pt.count) - dn * dn): 0;
    int64_t sqdn =
      (pt.count > 0)
        ? ((pt.sqduration - (sumdelta * sumdelta) / pt.count) / pt.count)
        : 0;
    float stddev = sqrt(static_cast<double>(sqdn));
    int64_t dmin = (pt.count > 0) ? pt.min.count() : -1;
    int64_t dmax = (pt.count > 0) ? pt.max.count() : -1;
    os << std::setw(maxlen + 1) << std::left << it->first
       << "= tot: " << std::setw(9) << std::right << pt.duration.count()
       << "us per: " << dn << "+-" << static_cast<int>(stddev) << "(" << dmin
       << "-" << dmax << ")"
       << " count: " << pt.count << " " << std::endl;
  }
  return os;
}

Profiler::PTimer::PTimer(
  const Duration & d, const PTimer & oldTimer, int ncount)
{
  duration = oldTimer.duration + d;
  sqduration =
    oldTimer.sqduration + (int64_t)(d.count() * d.count()) / (int64_t)ncount;
  Duration dn = d / ncount;
  min = (dn < oldTimer.min) ? dn : oldTimer.min;
  max = (dn > oldTimer.max) ? dn : oldTimer.max;
  count = oldTimer.count + ncount;
}

Profiler::PTimer::PTimer()
: duration(0),
  sqduration(0),
  min(std::numeric_limits<int64_t>::max()),
  max(std::numeric_limits<int64_t>::min()),
  count(0)
{
}

}  // namespace tagslam
