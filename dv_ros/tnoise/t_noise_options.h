//
// Created by kehan on 2022/1/11.
//

#ifndef DV_ROS_DV_ROS_TNOISE_T_NOISE_OPTIONS_H_
#define DV_ROS_DV_ROS_TNOISE_T_NOISE_OPTIONS_H_

#include "dv-sdk/processing.hpp"

namespace dv_ros {

struct TNoiseOptions {
  int frame_width;
  int frame_height;
};

TNoiseOptions CreateTNoiseOptions(const cv::FileStorage& config_file_parser,
                                  size_t event_index);

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_TNOISE_T_NOISE_OPTIONS_H_
