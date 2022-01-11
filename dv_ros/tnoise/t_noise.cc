//
// Created by kehan on 2022/1/11.
//

#include "dv_ros/tnoise/t_noise.h"

#include <memory>
#include <utility>
#include <thread>
#include <Eigen/Core>

namespace dv_ros {

TNoise::TNoise(const TNoiseOptions& options)
    : options_(options) {

}

void TNoise::ProcessEvents(dv::EventStore& event_store) const {
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> counter;
  counter.resize(options_.frame_height, options_.frame_width);
  counter.setZero();
  for (const auto& event : event_store) {
    counter(event.y(), event.x())++;
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
