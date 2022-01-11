//
// Created by kehan on 2022/1/11.
//

#ifndef DV_ROS_DV_ROS_TNOISE_T_NOISE_H_
#define DV_ROS_DV_ROS_TNOISE_T_NOISE_H_

#include "dv-sdk/processing.hpp"

namespace dv_ros {

class TNoise {
 public:
  static void ProcessEvents(dv::EventStore& event_store);
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_TNOISE_T_NOISE_H_
