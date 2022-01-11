//
// Created by kehan on 2022/1/11.
//

#include "dv_ros/tnoise/t_noise.h"

#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <thread>

#include "dv_ros/utils/tic_toc.h"

namespace dv_ros {

namespace {
int kThreadNum = 4;

class TNoiseThread {
 public:
  TNoiseThread(size_t id, const dv::EventStore& event_store) :
      id_(id),
      event_store_(event_store) {
    Run();
  }

  void Run() {
    const auto& process = [this]() {
      for (size_t i = id_; i < event_store_.getTotalLength(); i += kThreadNum) {
        const auto& event = event_store_.slice(i).front();
        auto key = std::make_pair(event.x(), event.y());
        if (counter_.count(key) == 0) {
          counter_.insert({key, 1});
        } else {
          counter_.insert({key, counter_.at(key)++});
        }
      }
    };
    thread_ = std::make_shared<std::thread>(process);
  }

  void Join() {
    if (thread_->joinable()) {
      thread_->join();
    }
  }

  [[nodiscard]] const std::map<std::pair<int16_t, int16_t>,
                               int8_t>& GetCounter() const {
    return counter_;
  }

 private:
  size_t id_;
  const dv::EventStore& event_store_;
  std::shared_ptr<std::thread> thread_;
  std::map<std::pair<int16_t, int16_t>, int8_t> counter_;
};

}  // namespace

void TNoise::ProcessEvents(dv::EventStore& event_store) {
  std::map<std::pair<int16_t, int16_t>, int8_t> counter;
  TicToc tic_toc;
  for (size_t i = 0; i < event_store.getTotalLength(); ++i) {
    const auto& event = event_store.slice(i).front();
    auto key = std::make_pair(event.x(), event.y());
    if (counter.count(key) == 0) {
      counter.insert({key, 1});
    } else {
      counter.insert({key, counter.at(key)++});
    }
  }
  std::cout << "------------" << std::endl;
  std::cout << "Filter1 cost: " << tic_toc.toc() << " ms" << std::endl;

  TicToc tic_toc2;
  for (auto event : event_store) {
    auto key = std::make_pair(event.x(), event.y());
    if (counter.count(key) == 0) {
      counter.insert({key, 1});
    } else {
      counter.insert({key, counter.at(key)++});
    }
  }
  std::cout << "Filter2 cost: " << tic_toc2.toc() << " ms" << std::endl;

  dv::EventStore result_event_store;
  for (auto event : event_store) {
    auto key = std::make_pair(event.x(), event.y());
    if (counter.count(key) != 0 && counter.at(key) > 1) {
      result_event_store.add(event);
    }
  }

  /*
  std::vector<TNoiseThread> threads;
  for (size_t id = 0; id < kThreadNum; ++id) {
    threads.emplace_back(id, event_store);
  }
  for (auto thread : threads) {
    thread.Join();
  }
  for (auto event : event_store) {
    size_t count = 0;
    auto key = std::make_pair(event.x(), event.y());
    for (size_t id = 0; id < kThreadNum; ++id) {
      const auto& counter = threads.at(id).GetCounter();
      if (counter.count(key) != 0) {
        count += counter.at(key);
      }
    }
    if (count > 1) {
      result_event_store.add(event);
    }
  }
   */

  event_store = result_event_store;
}

}  // namespace dv_ros
