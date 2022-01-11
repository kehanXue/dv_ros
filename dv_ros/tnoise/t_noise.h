//
// Created by kehan on 2022/1/11.
//

#ifndef DV_ROS_DV_ROS_TNOISE_T_NOISE_H_
#define DV_ROS_DV_ROS_TNOISE_T_NOISE_H_

#include "dv-sdk/processing.hpp"

#include <dv_ros/tnoise/t_noise_options.h>

namespace dv_ros {

class TNoise {
 public:
  explicit TNoise(const TNoiseOptions& options);
  void ProcessEvents(dv::EventStore& event_store) const;

 private:
  TNoiseOptions options_;
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_TNOISE_T_NOISE_H_
