#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/i2c/i2c_bus.h"
#include "esphome/core/helpers.h"

#include <vector>
#include <mutex>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "driver/jpeg_encode.h"

namespace esphome {
namespace p4_csi_camera {

using namespace esphome::camera;

class P4CSICamera;

// Frame buffer wrapper
struct P4CameraFrameBuffer {
  uint8_t *data{nullptr};
  size_t length{0};
  size_t width{0};
  size_t height{0};
};

// CameraImage implementation
class P4CSICameraImage : public camera::CameraImage {
 public:
  P4CSICameraImage(uint8_t *data, size_t length, uint8_t requesters);
  uint8_t *get_data_buffer() override;
  size_t get_data_length() override;
  bool was_requested_by(camera::CameraRequester requester) const override;
 protected:
  uint8_t *data_{nullptr};
  size_t length_{0};
  uint8_t requesters_{0};
};

// CameraImageReader implementation
class P4CSICameraImageReader : public camera::CameraImageReader {
 public:
  P4CSICameraImageReader() {}
  ~P4CSICameraImageReader() {}
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;
 protected:
  std::shared_ptr<P4CSICameraImage> image_;
  size_t offset_{0};
};

// Main Camera Component
class P4CSICamera : public camera::Camera {
 public:
  P4CSICamera();

  void set_i2c_bus(i2c::I2CBus *bus) { this->i2c_bus_ = bus; }
  void set_data_lanes(uint8_t lanes) { this->data_lanes_ = lanes; }
  void set_max_update_interval(uint32_t interval) { this->max_update_interval_ = interval; }
  void set_idle_update_interval(uint32_t interval) { this->idle_update_interval_ = interval; }
  void set_reset_pin(int8_t pin) { this->reset_pin_ = pin; }
  void set_power_pin(int8_t pin) { this->power_pin_ = pin; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_vertical_flip(bool flip) { this->vertical_flip_ = flip; }
  void set_horizontal_flip(bool flip) { this->horizontal_flip_ = flip; }
  void set_red_gain(float gain) { this->red_gain_ = gain; this->recompute_lut_(); }
  void set_green_gain(float gain) { this->green_gain_ = gain; this->recompute_lut_(); }
  void set_blue_gain(float gain) { this->blue_gain_ = gain; this->recompute_lut_(); }
  void set_color_correction(bool enable) { this->color_correction_ = enable; }
  void set_auto_white_balance(bool enable) { this->auto_white_balance_ = enable; }

  // Runtime setters (apply immediately via V4L2/LUT if initialized)
  void set_jpeg_quality_runtime(uint8_t quality);
  void set_vertical_flip_runtime(bool flip);
  void set_horizontal_flip_runtime(bool flip);
  void set_red_gain_runtime(float gain);
  void set_green_gain_runtime(float gain);
  void set_blue_gain_runtime(float gain);
  void set_color_correction_runtime(bool enable);
  void set_auto_white_balance_runtime(bool enable);

  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  bool get_vertical_flip() const { return this->vertical_flip_; }
  bool get_horizontal_flip() const { return this->horizontal_flip_; }
  float get_red_gain() const { return this->red_gain_; }
  float get_green_gain() const { return this->green_gain_; }
  float get_blue_gain() const { return this->blue_gain_; }
  bool get_color_correction() const { return this->color_correction_; }
  bool get_auto_white_balance() const { return this->auto_white_balance_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  const std::string &get_status_msg() const { return this->status_msg_; }

  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;
  void request_image(camera::CameraRequester requester) override;
  void add_listener(camera::CameraListener *listener) override;
  void add_image_callback(std::function<void(std::shared_ptr<camera::CameraImage>)> &&callback);
  camera::CameraImageReader *create_image_reader() override;

 protected:
  bool has_requested_image_() const;
  bool can_return_image_() const;
  void recompute_lut_();
  void apply_hardware_controls_();

  // Configuration
  i2c::I2CBus *i2c_bus_{nullptr};
  uint8_t data_lanes_{2};
  int8_t reset_pin_{-1};
  int8_t power_pin_{-1};
  uint32_t max_update_interval_{500};
  uint32_t idle_update_interval_{10000};
  uint8_t jpeg_quality_{40};
  bool vertical_flip_{false};
  bool horizontal_flip_{false};

  // Color correction & white balance
  float red_gain_{1.45f};
  float green_gain_{0.85f};
  float blue_gain_{1.55f};
  bool color_correction_{true};
  bool auto_white_balance_{true};
  uint8_t r_lut_[32]{};
  uint8_t g_lut_[64]{};
  uint8_t b_lut_[32]{};

  // V4L2 video & ISP devices
  int video_fd_{-1};
  int isp_fd_{-1};
  uint8_t *frame_buffers_[6]{};
  size_t frame_buf_size_{0};
  uint16_t frame_width_{0};
  uint16_t frame_height_{0};

  // JPEG encoder
  jpeg_encoder_handle_t jpeg_handle_{nullptr};
  uint8_t *jpeg_out_buf_{nullptr};
  size_t jpeg_out_buf_size_{0};
  uint8_t *encode_src_buf_{nullptr};
  P4CameraFrameBuffer jpeg_buffer_;

  // State
  bool initialized_{false};
  std::string status_msg_{"not started"};
  std::string chosen_path_{};
  std::shared_ptr<P4CSICameraImage> current_image_;
  uint8_t single_requesters_{0};
  uint8_t stream_requesters_{0};
  uint64_t last_update_{0};
  uint32_t last_idle_request_{0};

  static void frame_task(void *pvParameters);
  void frame_task_loop_();

  TaskHandle_t task_handle_{nullptr};
  std::mutex image_mutex_;
  std::shared_ptr<P4CSICameraImage> new_image_{nullptr};
  std::vector<uint16_t> row_buffer_;

  // Callbacks
  CallbackManager<void(std::shared_ptr<camera::CameraImage>)> new_image_callback_{};
  std::vector<camera::CameraListener *> listeners_;
};

extern P4CSICamera *global_p4_csi_camera;

}  // namespace p4_csi_camera
}  // namespace esphome

// Include platform headers so codegen can find the classes
#include "p4_csi_camera_number.h"
#include "p4_csi_camera_switch.h"

#endif  // USE_ESP32_VARIANT_ESP32P4
