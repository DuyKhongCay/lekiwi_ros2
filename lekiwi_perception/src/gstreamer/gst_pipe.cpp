// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "gstreamer/gst_pipe.hpp"

#include <gst/gstdebugutils.h>

#include <sstream>
#include <utility>

namespace lekiwi_perception
{

bool SampleGate::enter_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++callbacks_;
  return accepting_;
}

void SampleGate::leave_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (callbacks_ > 0U) {
    --callbacks_;
  }
  condition_.notify_all();
}

void SampleGate::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  accepting_ = false;
  condition_.notify_all();
}

bool SampleGate::wait_drained(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return condition_.wait_for(lock, timeout, [this]() {return callbacks_ == 0U;});
}

void SampleGate::wait_drained()
{
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this]() {return callbacks_ == 0U;});
}


struct GstPipeController::SinkBinding
{
  GstPipeController * controller{nullptr};
  StreamId stream{StreamId::kStereoLeft};
  std::shared_ptr<SampleGate> gate;
  GstElement * sink{nullptr};
  gulong sample_signal_id{0};

  ~SinkBinding()
  {
    if (sink != nullptr && sample_signal_id != 0U) {
      g_signal_handler_disconnect(sink, sample_signal_id);
    }
    if (sink != nullptr) {
      gst_object_unref(sink);
    }
  }
};

void GstPipeController::disconnect_sinks()
{
  for (const auto & binding : sink_bindings_) {
    if (binding->sink != nullptr && binding->sample_signal_id != 0U) {
      g_signal_handler_disconnect(binding->sink, binding->sample_signal_id);
      binding->sample_signal_id = 0U;
    }
  }
}

GstPipeController::GstPipeController(
  std::string name,
  std::vector<SinkSpec> sinks,
  SampleCallback sample_callback)
: name_(std::move(name)),
  sink_specs_(std::move(sinks)),
  sample_callback_(std::move(sample_callback))
{
}

GstPipeController::~GstPipeController()
{
  std::string ignored;
  static_cast<void>(stop(std::chrono::milliseconds(1000), ignored));
}

bool GstPipeController::start(
  const std::string & description,
  GstClock * common_clock,
  std::chrono::milliseconds timeout,
  std::string & error)
{
  if (pipe_ != nullptr) {
    error = name_ + " is already initialized";
    return false;
  }

  GError * parse_error = nullptr;
  pipe_ = gst_parse_launch(description.c_str(), &parse_error);
  if (parse_error != nullptr || pipe_ == nullptr) {
    error = name_ + " parse failed: " +
      (parse_error != nullptr ? parse_error->message : "unknown GStreamer error");
    if (parse_error != nullptr) {
      g_error_free(parse_error);
    }
    std::string cleanup_error;
    static_cast<void>(stop(timeout, cleanup_error));
    return false;
  }
  if (!GST_IS_PIPELINE(pipe_)) {
    error = name_ + " description did not create a GstPipeline";
    std::string cleanup_error;
    static_cast<void>(stop(timeout, cleanup_error));
    return false;
  }

  if (common_clock != nullptr) {
    gst_pipeline_use_clock(GST_PIPELINE(pipe_), common_clock);
  }
  bus_ = gst_element_get_bus(pipe_);
  gate_ = std::make_shared<SampleGate>();
  for (const auto & sink_spec : sink_specs_) {
    auto binding = std::make_unique<SinkBinding>();
    binding->controller = this;
    binding->stream = sink_spec.stream;
    binding->gate = gate_;
    binding->sink = gst_bin_get_by_name(GST_BIN(pipe_), sink_spec.name.c_str());
    if (binding->sink == nullptr || !GST_IS_APP_SINK(binding->sink)) {
      error = name_ + " is missing appsink " + sink_spec.name;
      std::string cleanup_error;
      static_cast<void>(stop(timeout, cleanup_error));
      return false;
    }
    binding->sample_signal_id = g_signal_connect(
      binding->sink, "new-sample", G_CALLBACK(GstPipeController::on_new_sample),
      binding.get());
    sink_bindings_.push_back(std::move(binding));
  }

  const auto state_result = gst_element_set_state(pipe_, GST_STATE_PLAYING);
  GstState current = GST_STATE_NULL;
  GstState pending = GST_STATE_VOID_PENDING;
  const auto wait_result = state_result == GST_STATE_CHANGE_FAILURE ? GST_STATE_CHANGE_FAILURE :
    gst_element_get_state(
    pipe_, &current, &pending, static_cast<GstClockTime>(timeout.count()) * GST_MSECOND);
  if (wait_result == GST_STATE_CHANGE_FAILURE ||
    (wait_result == GST_STATE_CHANGE_ASYNC && current != GST_STATE_PLAYING))
  {
    bool has_error = false;
    std::string diags;
    drain_bus_diags(diags, has_error);
    error = name_ + (wait_result == GST_STATE_CHANGE_FAILURE ?
      " failed while entering PLAYING" : " timed out while entering PLAYING");
    if (!diags.empty()) {
      error += ": " + diags;
    }
    std::string cleanup_error;
    static_cast<void>(stop(timeout, cleanup_error));
    return false;
  }

  gst_debug_bin_to_dot_file_with_ts(
    GST_BIN(pipe_), GST_DEBUG_GRAPH_SHOW_ALL, (name_ + "_playing").c_str());
  return true;
}

bool GstPipeController::stop(std::chrono::milliseconds timeout, std::string & error)
{
  if (pipe_ == nullptr) {
    return true;
  }
  if (gate_) {
    gate_->close();
  }
  disconnect_sinks();

  const auto state_result = gst_element_set_state(pipe_, GST_STATE_NULL);
  GstState current = GST_STATE_VOID_PENDING;
  GstState pending = GST_STATE_VOID_PENDING;
  const auto wait_result = state_result == GST_STATE_CHANGE_FAILURE ? GST_STATE_CHANGE_FAILURE :
    gst_element_get_state(
    pipe_, &current, &pending, static_cast<GstClockTime>(timeout.count()) * GST_MSECOND);
  if (wait_result == GST_STATE_CHANGE_FAILURE || current != GST_STATE_NULL) {
    error = name_ + " did not reach NULL (current=" +
      gst_element_state_get_name(current) + ", pending=" +
      gst_element_state_get_name(pending) + ")";
    return false;
  }
  if (gate_ && !gate_->wait_drained(timeout)) {
    error = name_ + " timed out waiting for callbacks";
    return false;
  }
  release_pipe();
  return true;
}

bool GstPipeController::poll_error(std::string & error)
{
  bool has_error = false;
  drain_bus_diags(error, has_error);
  return has_error;
}

GstFlowReturn GstPipeController::on_new_sample(GstAppSink * sink, gpointer user_data)
{
  auto * binding = static_cast<SinkBinding *>(user_data);
  try {
    if (!binding->gate->with_callback([binding, sink]() {
        binding->controller->sample_callback_(binding->stream, sink, binding->controller->pipe_);
      }))
    {
      return GST_FLOW_FLUSHING;
    }
  } catch (...) {
    return GST_FLOW_ERROR;
  }
  return GST_FLOW_OK;
}

void GstPipeController::drain_bus_diags(std::string & diags, bool & has_error)
{
  if (bus_ == nullptr) {
    return;
  }
  std::ostringstream messages;
  GstMessage * message = nullptr;
  while ((message = gst_bus_pop_filtered(
      bus_, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING))) != nullptr)
  {
    GError * gst_error = nullptr;
    gchar * debug = nullptr;
    const bool is_error = GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR;
    if (is_error) {
      gst_message_parse_error(message, &gst_error, &debug);
      has_error = true;
    } else {
      gst_message_parse_warning(message, &gst_error, &debug);
    }
    if (messages.tellp() > 0) {
      messages << " | ";
    }
    messages << (is_error ? "error" : "warning") << " from " << GST_OBJECT_NAME(message->src)
             << ": " << (gst_error != nullptr ? gst_error->message : "unknown GStreamer message");
    if (debug != nullptr && debug[0] != '\0') {
      messages << " (" << debug << ")";
    }
    if (debug != nullptr) {
      g_free(debug);
    }
    if (gst_error != nullptr) {
      g_error_free(gst_error);
    }
    gst_message_unref(message);
  }
  diags = messages.str();
}

void GstPipeController::release_pipe()
{
  sink_bindings_.clear();
  if (bus_ != nullptr) {
    gst_object_unref(bus_);
    bus_ = nullptr;
  }
  if (pipe_ != nullptr) {
    gst_object_unref(pipe_);
    pipe_ = nullptr;
  }
  gate_.reset();
}

}  // namespace lekiwi_perception
