//
// Created by kehan on 2021/9/1.
//

#include "dv_ros/event_collector.h"

#include <utility>
#include "dv_ros/event.h"

namespace dv_ros {

EventCollector::EventCollector(EventCollectorOptions options)
    : options_(std::move(options)),
      accumulator_(options_.accumulator_options_),
      nh_("~") {
  if (options_.device_type == EventDeviceType::DAVIS) {
    subscriber_ =
        nh_.subscribe<dvs_msgs::EventArray>(
            options_.event_topic,
            2,
            &EventCollector::EventsCallback,
            this);
  } else if (options_.device_type == EventDeviceType::CELEX) {
    subscriber_ =
        nh_.subscribe<celex5_msgs::EventVector>(
            options_.event_topic,
            2,
            &EventCollector::EventsCallback,
            this);
  }
}

EventCollector::~EventCollector() {

}

void EventCollector::EventsCallback(
    const dvs_msgs::EventArrayConstPtr& events_msg) {
  ProcessEvents(events_msg);
}

void EventCollector::EventsCallback(
    const celex5_msgs::EventVectorConstPtr& events_msg) {
  ProcessEvents(events_msg);
}

template <typename EventType>
void EventCollector::ProcessEvents(const EventType& events) {
  dv::EventStore dv_events_;
  for (auto event : events->events) {
    dv_events_.add(ToDVEvent(ToEvent(event)));
  }
  accumulator_.AddNewEvents(dv_events_);
}

}  // namespace dv_ros
