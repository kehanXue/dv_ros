//
// Created by kehan on 2022/1/11.
//

#include "dv_ros/tnoise/t_noise_options.h"
#include "dv_ros/utils/options_tools.h"

namespace dv_ros {

TNoiseOptions CreateTNoiseOptions(const cv::FileStorage& config_file_parser,
                                  size_t event_index) {
  TNoiseOptions options{};
  options.frame_width =
      config_file_parser[eventI_frame_width(event_index)];
  options.frame_height =
      config_file_parser[eventI_frame_height(event_index)];
  return options;
}

}  // namespace dv_ros
