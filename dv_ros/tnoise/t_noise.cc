//
// Created by kehan on 2022/1/11.
//

#include "dv_ros/tnoise/t_noise.h"

#include <memory>
#include <utility>
#include <thread>
#include <Eigen/Core>

namespace dv_ros {

namespace {

class TNoiseThread {
 public:
  explicit TNoiseThread(dv::EventStore event_store) :
      event_store_(std::move(event_store)) {
  }

  void Run(int frame_height, int frame_width) {
    counter_.resize(frame_height, frame_width);
    counter_.setZero();
    const auto& process = [this]() {
      for (const auto& event : event_store_) {
        counter_(event.y(), event.x())++;
      }
    };
    thread_ = std::make_shared<std::thread>(process);
  }

  void Join() {
    if (thread_->joinable()) {
      thread_->join();
    }
  }

  const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>& GetCounter() {
    return counter_;
  }

 private:
  const dv::EventStore event_store_;
  std::shared_ptr<std::thread> thread_;
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> counter_;
};

}  // namespace


TNoise::TNoise(const TNoiseOptions& options)
    : options_(options) {

}

void TNoise::ProcessEvents(dv::EventStore& event_store) const {
  size_t sub_length = event_store.getTotalLength() / options_.num_threads;
  std::vector<TNoiseThread> threads;
  for (size_t i = 0; i < event_store.getTotalLength(); i += sub_length) {
    size_t length = sub_length;
    if (i + sub_length >= event_store.getTotalLength()) {
      length = event_store.getTotalLength() - i;
    }
    auto sub_event_store = event_store.slice(i, length);
    threads.emplace_back(sub_event_store);
  }
  for (auto& thread : threads) {
    thread.Run(options_.frame_height, options_.frame_width);
  }
  for (auto& thread : threads) {
    thread.Join();
  }

  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> counter;
  counter.resize(options_.frame_height, options_.frame_width);
  counter.setZero();
  for (size_t id = 0; id < options_.num_threads; ++id) {
    const auto& sub_counter = threads.at(id).GetCounter();
    counter += sub_counter;
  }
  dv::EventStore result_event_store;
  for (auto event : event_store) {
    if (counter(event.y(), event.x()) > 1) {
      result_event_store.add(event);
    }
  }
  event_store = result_event_store;
}

}  // namespace dv_ros
