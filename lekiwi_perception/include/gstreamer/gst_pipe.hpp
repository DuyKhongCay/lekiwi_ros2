// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#ifndef LEKIWI_PERCEPTION__GST_PIPE_HPP_
#define LEKIWI_PERCEPTION__GST_PIPE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <string>
#include <vector>

#include "gstreamer/pipe_builder.hpp"

namespace lekiwi_perception
{

class SampleGate
{
public:
  void close();
  [[nodiscard]] bool wait_drained(std::chrono::milliseconds timeout);
  void wait_drained();

  template<typename CallbackT>
  [[nodiscard]] bool with_callback(CallbackT && callback)
  {
    if (!enter_callback()) {
      leave_callback();
      return false;
    }
    try {
      std::forward<CallbackT>(callback)();
    } catch (...) {
      leave_callback();
      throw;
    }
    leave_callback();
    return true;
  }

private:
  bool enter_callback();
  void leave_callback();

  std::mutex mutex_;
  std::condition_variable condition_;
  bool accepting_{true};
  std::size_t callbacks_{0};
};

class GstPipeController
{
public:
  struct SinkSpec
  {
    std::string name;
    StreamId stream;
  };

  using SampleCallback = std::function<void(StreamId, GstAppSink *, GstElement *)>;

  GstPipeController(
    std::string name,
    std::vector<SinkSpec> sinks,
    SampleCallback sample_callback);
  ~GstPipeController();

  GstPipeController(const GstPipeController &) = delete;
  GstPipeController & operator=(const GstPipeController &) = delete;

  [[nodiscard]] bool start(
    const std::string & description,
    GstClock * common_clock,
    std::chrono::milliseconds timeout,
    std::string & error);
  [[nodiscard]] bool stop(std::chrono::milliseconds timeout, std::string & error);
  [[nodiscard]] bool poll_error(std::string & error);

private:
  struct SinkBinding;

  static GstFlowReturn on_new_sample(GstAppSink * sink, gpointer user_data);
  void drain_bus_diags(std::string & diags, bool & has_error);
  void disconnect_sinks();
  void release_pipe();

  std::string name_;
  std::vector<SinkSpec> sink_specs_;
  SampleCallback sample_callback_;
  GstElement * pipe_{nullptr};
  GstBus * bus_{nullptr};
  std::vector<std::unique_ptr<SinkBinding>> sink_bindings_;
  std::shared_ptr<SampleGate> gate_;
};

}  // namespace lekiwi_perception

#endif  // LEKIWI_PERCEPTION__GST_PIPE_HPP_
