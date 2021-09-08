//
// Created by kehan on 2021/9/1.
//

#ifndef DV_ROS_DV_ROS_ACCUMULATOR_ACCUMULATOR_OPTIONS_H_
#define DV_ROS_DV_ROS_ACCUMULATOR_ACCUMULATOR_OPTIONS_H_

#include <opencv2/opencv.hpp>

namespace dv_ros {

enum class AccumulationMethod {
  BY_TIME = 0,
  BY_COUNT,
  BY_EVENTS_HZ_AND_COUNT
};

enum class DecayFunction {
  NONE = 0,
  LINEAR,
  EXPONENTIAL,
  STEP
};

struct AccumulatorOptions {
  int frame_width;
  int frame_height;
  AccumulationMethod accumulation_method;
  int count_window_size;
  int time_window_size;
  DecayFunction decay_function;
  float decay_param;
  float min_potential;
  float max_potential;
  float neutral_potential;
  float event_contribution;
  int rectify_polarity;
  int synchronous_decay;
  std::string accumulated_frame_topic;
  int no_motion_threshold;
};

AccumulatorOptions CreateAccumulatorOptions(
    const cv::FileStorage& config_file_parser,
    size_t event_index);

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_ACCUMULATOR_ACCUMULATOR_OPTIONS_H_
