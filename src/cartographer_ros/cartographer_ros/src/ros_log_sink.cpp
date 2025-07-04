/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "glog/log_severity.h"
#include "glog/logging.h"

namespace cartographer_ros {

namespace {

const char* GetBasename(const char* filepath) {
  const char* base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

#if defined(ROS_CARTOGRAPHER_GLOG_GE_070)
std::chrono::system_clock::time_point ConvertTmToTimePoint(const std::tm& tm) {
    std::time_t timeT = std::mktime(const_cast<std::tm*>(&tm)); // Convert std::tm to time_t
    return std::chrono::system_clock::from_time_t(timeT);      // Convert time_t to time_point
}
#endif

}  // namespace

ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  (void) base_filename; // TODO: remove unused arg ?

#if defined(ROS_CARTOGRAPHER_GLOG_GE_070)
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, ::google::LogMessageTime(ConvertTmToTimePoint(*tm_time)), message, message_len);
  // Google glog broke the ToString API, but has no way to tell what version it is using.
  // To support both newer and older glog versions, use a nasty hack were we infer the
  // version based on whether GOOGLE_GLOG_DLL_DECL is defined
#elif defined(GOOGLE_GLOG_DLL_DECL)
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
#else
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, ::google::LogMessageTime(*tm_time), message, message_len);
#endif
  switch (severity) {
    case ::google::GLOG_INFO:
      RCLCPP_INFO_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_WARNING:
      RCLCPP_WARN_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_ERROR:
      RCLCPP_ERROR_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_FATAL:
      RCLCPP_FATAL_STREAM(logger_, message_string);
      will_die_ = true;
      break;
  }
}

void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_ros
