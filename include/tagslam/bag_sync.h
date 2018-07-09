/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef TAGSLAM_BAG_SYNC_H
#define TAGSLAM_BAG_SYNC_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <string>



namespace tagslam {
  template <class T>
  /*
    bag synchronizing class for a single message type
   */
  class BagSync {
      typedef boost::shared_ptr<T const> ConstPtr;
  public:
    BagSync(const std::vector<std::string> &topics,
            const std::function<void(const std::vector<ConstPtr> &)> &callback)
      : topics_(topics),
        callback_(callback)
      {
    }
    bool process(const rosbag::MessageInstance &m) {
      boost::shared_ptr<T> msg = m.instantiate<T>();
      if (msg) {
        if (msg->header.stamp > currentTime_) {
          if (msgMap_.size() == topics_.size()) {
            std::vector<ConstPtr> msgVec;
            for (const auto &m: msgMap_) {
              msgVec.push_back(m.second);
            }
            callback_(msgVec);
          }
          msgMap_.clear();
          currentTime_ = msg->header.stamp;
        }
        msgMap_[m.getTopic()] = msg;
      }
      return (true);
    }
    const ros::Time &getCurrentTime() { return (currentTime_); }
  private:
    ros::Time currentTime_{0.0};
    std::map<std::string, ConstPtr> msgMap_;
    std::vector<std::string> topics_;
    std::function<void(const std::vector<ConstPtr> &)> callback_;
  };

  /*
    bag synchronizing class for two different message types
   */
  template <typename T1, typename T2>
  class BagSync2 {
    typedef boost::shared_ptr<T1 const> ConstPtr1;
    typedef boost::shared_ptr<T2 const> ConstPtr2;
    typedef  std::map<std::string, std::map<ros::Time,ConstPtr1>> MsgMap1;
    typedef  std::map<std::string, std::map<ros::Time,ConstPtr2>> MsgMap2;
  public:
    BagSync2(const std::vector<std::string> &topics1,
             const std::vector<std::string> &topics2,
             const std::function<void(const std::vector<ConstPtr1> &, const std::vector<ConstPtr2>&)> &callback)
      : topics1_(topics1), topics2_(topics2), callback_(callback)
      {
        for (const auto &topic: topics1_) {
          msgMap1_[topic] = std::map<ros::Time, ConstPtr1>();
        }
        for (const auto &topic: topics2_) {
          msgMap2_[topic] = std::map<ros::Time, ConstPtr2>();
        }
      }
    const ros::Time &getCurrentTime() { return (currentTime_); }
    
    template<typename T>
    static std::vector<boost::shared_ptr<T const>>  makeVec(const ros::Time &t,
      std::map<std::string, std::map<ros::Time, boost::shared_ptr<T const> >> *topicToQueue) {

      std::vector<boost::shared_ptr<T const>> mvec;
      for (auto &queue: *topicToQueue) { // iterate over all topics
        auto &t2m = queue.second; // time to message
        while (t2m.begin()->first < t) {
          t2m.erase(t2m.begin());
        }
        mvec.push_back(t2m.begin()->second);
        t2m.erase(t2m.begin());
      }
      return (mvec);
    }

    void publishMessages(const ros::Time &t) {
      std::vector<boost::shared_ptr<T1 const>> mvec1 = makeVec(t, &msgMap1_);
      std::vector<boost::shared_ptr<T2 const>> mvec2 = makeVec(t, &msgMap2_);
      callback_(mvec1, mvec2);
    }
    // have per-topic queue
    // have time-to-count map
    // if new message comes in, put it at head of its queue,
    // and bump the time-to-count-map entry.
    // if time-to-count-map entry is equal to number of topics:
    //  - erase all time-to-count-map entries that are older
    //  - erase all per-topic-queue entries that are older
    //  - deliver callback
    bool process(const rosbag::MessageInstance &m) {
      boost::shared_ptr<T1> msg1 = m.instantiate<T1>();
      boost::shared_ptr<T2> msg2 = m.instantiate<T2>();
      if (!msg1 && !msg2) {
        return (false);
      }

      const ros::Time t = msg1 ? msg1->header.stamp : msg2->header.stamp;
      if (msg1) {
        msgMap1_[m.getTopic()][t] = msg1;
      } else {
        msgMap2_[m.getTopic()][t] = msg2;
      }
      auto it = msgCount_.find(t);
      if (it == msgCount_.end()) {
        msgCount_.insert(CountMap::value_type(t, 1));
        it = msgCount_.find(t);
      } else {
        it->second++;
      }
      if (it->second == (int) (topics1_.size() + topics2_.size())) {
        currentTime_ = t;
        publishMessages(t);  // also cleans out queues
        it++;
        msgCount_.erase(msgCount_.begin(), it);
      }
      return (true);
    }
  private:
    typedef std::map<ros::Time, int> CountMap;
    ros::Time currentTime_{0.0};
    CountMap msgCount_;
    MsgMap1  msgMap1_;
    MsgMap2  msgMap2_;
    std::vector<std::string> topics1_, topics2_;
    std::function<void(const std::vector<ConstPtr1> &,
                       const std::vector<ConstPtr2> &)> callback_;
  };
}

#endif
