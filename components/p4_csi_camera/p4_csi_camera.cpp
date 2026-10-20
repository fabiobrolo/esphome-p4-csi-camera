#ifdef USE_ESP32_VARIANT_ESP32P4

#include "p4_csi_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#if defined(USE_ESP32_FRAMEWORK_ESP_IDF)
#include "esphome/components/i2c/i2c_bus_esp_idf.h"
#endif

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <cmath>
#include <algorithm>

#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_cam_sensor_detect.h"
#include "ov02c10.h"
#include "esp_task_wdt.h"

namespace esphome {
namespace p4_csi_camera {

static const char *const TAG = "p4_csi_camera";
P4CSICamera *global_p4_csi_camera = nullptr;

#define SENSOR_WIDTH 1920
#define SENSOR_HEIGHT 1080

// ===========================================================================
// P4CSICameraImage
// ===========================================================================

P4CSICameraImage::P4CSICameraImage(uint8_t *data, size_t length, uint8_t requesters)
    : data_(data), length_(length), requesters_(requesters) {}

uint8_t *P4CSICameraImage::get_data_buffer() { return this->data_; }
size_t P4CSICameraImage::get_data_length() { return this->length_; }
bool P4CSICameraImage::was_requested_by(camera::CameraRequester requester) const {
  return (this->requesters_ & (1 << (uint8_t)requester)) != 0;
}

// ===========================================================================
// P4CSICameraImageReader
// ===========================================================================

void P4CSICameraImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<P4CSICameraImage>(image);
  this->offset_ = 0;
}
size_t P4CSICameraImageReader::available() const {
  if (!this->image_) return 0;
  size_t len = this->image_->get_data_length();
  if (this->offset_ >= len) return 0;
  return len - this->offset_;
}
uint8_t *P4CSICameraImageReader::peek_data_buffer() {
  if (!this->image_) return nullptr;
  if (this->offset_ >= this->image_->get_data_length()) return nullptr;
  return this->image_->get_data_buffer() + this->offset_;
}
void P4CSICameraImageReader::consume_data(size_t consumed) { this->offset_ += consumed; }
void P4CSICameraImageReader::return_image() {
  this->image_.reset();
  this->offset_ = 0;
}

// ===========================================================================
// P4CSICamera
// ===========================================================================

P4CSICamera::P4CSICamera() {
  global_p4_csi_camera = this;
  this->recompute_lut_();
}

void P4CSICamera::recompute_lut_() {
  for (int i = 0; i < 32; i++) {
    int val = (int)std::round(i * this->red_gain_);
    this->r_lut_[i] = (uint8_t)std::min(31, std::max(0, val));
  }
  for (int i = 0; i < 64; i++) {
    int val = (int)std::round(i * this->green_gain_);
    this->g_lut_[i] = (uint8_t)std::min(63, std::max(0, val));
  }
  for (int i = 0; i < 32; i++) {
    int val = (int)std::round(i * this->blue_gain_);
    this->b_lut_[i] = (uint8_t)std::min(31, std::max(0, val));
  }
}

void P4CSICamera::apply_hardware_controls_() {
  if (this->video_fd_ < 0) return;

  // Try standard V4L2 Auto White Balance on video_fd and isp_fd
  struct v4l2_control awb_ctrl = {};
  awb_ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
  awb_ctrl.value = this->auto_white_balance_ ? 1 : 0;
  if (ioctl(this->video_fd_, VIDIOC_S_CTRL, &awb_ctrl) == 0) {
    ESP_LOGD(TAG, "V4L2 AWB set to %d on video fd", awb_ctrl.value);
  } else if (this->isp_fd_ >= 0) {
    ioctl(this->isp_fd_, VIDIOC_S_CTRL, &awb_ctrl);
  }

  // Try Red / Blue balance controls if supported by hardware ISP
  struct v4l2_control r_ctrl = {};
  r_ctrl.id = V4L2_CID_RED_BALANCE;
  r_ctrl.value = (int)(this->red_gain_ * 1000);
  if (ioctl(this->video_fd_, VIDIOC_S_CTRL, &r_ctrl) != 0 && this->isp_fd_ >= 0) {
    ioctl(this->isp_fd_, VIDIOC_S_CTRL, &r_ctrl);
  }

  struct v4l2_control b_ctrl = {};
  b_ctrl.id = V4L2_CID_BLUE_BALANCE;
  b_ctrl.value = (int)(this->blue_gain_ * 1000);
  if (ioctl(this->video_fd_, VIDIOC_S_CTRL, &b_ctrl) != 0 && this->isp_fd_ >= 0) {
    ioctl(this->isp_fd_, VIDIOC_S_CTRL, &b_ctrl);
  }
}

void P4CSICamera::setup() {
  ESP_LOGI(TAG, "Setting up ESP32-P4 CSI Camera (OV02C10) via esp_video...");
  ESP_LOGI(TAG, "  Data lanes: %d, Max update interval: %d ms", this->data_lanes_, (int)this->max_update_interval_);

  // Step 1: Initialize the video subsystem (CSI + ISP + sensor detection)
  esp_video_init_csi_config_t csi_config = {};
#if defined(USE_ESP32_FRAMEWORK_ESP_IDF)
  if (this->i2c_bus_ != nullptr) {
    auto *idf_bus = static_cast<esphome::i2c::IDFI2CBus *>(this->i2c_bus_);
    if (idf_bus->bus_ == nullptr) {
      this->status_msg_ = "I2C master bus handle is null";
      ESP_LOGE(TAG, "%s", this->status_msg_.c_str());
      this->status_set_error(LOG_STR("I2C bus null"));
      this->mark_failed();
      return;
    }
    ESP_LOGD(TAG, "Using shared ESPHome I2C bus for sensor SCCB (handle=%p)", idf_bus->bus_);
    csi_config.sccb_config.init_sccb = false;
    csi_config.sccb_config.i2c_handle = idf_bus->bus_;
    csi_config.sccb_config.freq = 400000;
  } else {
#endif
    csi_config.sccb_config.init_sccb = true;
    csi_config.sccb_config.i2c_config.port = 0;
    csi_config.sccb_config.i2c_config.scl_pin = (gpio_num_t)8;
    csi_config.sccb_config.i2c_config.sda_pin = (gpio_num_t)7;
    csi_config.sccb_config.freq = 400000;
#if defined(USE_ESP32_FRAMEWORK_ESP_IDF)
  }
#endif
  csi_config.reset_pin = (gpio_num_t)-1;
  csi_config.pwdn_pin = (gpio_num_t)-1;

  esp_video_init_config_t cam_config = {};
  cam_config.csi = &csi_config;

  esp_err_t err = esp_video_init(&cam_config);
  if (err != ESP_OK) {
    char buf[128];
    snprintf(buf, sizeof(buf), "esp_video_init failed: %s (0x%x)", esp_err_to_name(err), err);
    this->status_msg_ = buf;
    ESP_LOGE(TAG, "%s", buf);
    this->status_set_error(LOG_STR("esp_video_init failed"));
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "esp_video initialized successfully");

  // Step 2: Open and discover video devices
  const char *dev_paths[] = {"/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3", "/dev/video4", "/dev/video10", "/dev/video20"};

  struct OpenDev {
    const char *path;
    int fd;
    bool supports_rgb565;
    uint32_t first_fmt;
    uint32_t default_fmt;
    uint32_t default_w;
    uint32_t default_h;
  };
  std::vector<OpenDev> open_devs;

  for (const char *path : dev_paths) {
    int fd = open(path, O_RDWR, 0);
    if (fd < 0) {
      int fd_ro = open(path, O_RDONLY, 0);
      if (fd_ro >= 0) {
        fd = fd_ro;
      } else {
        continue;
      }
    }

    OpenDev dev = {};
    dev.path = path;
    dev.fd = fd;

    struct v4l2_fmtdesc fdesc = {};
    fdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fdesc.index = 0;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {
      if (fdesc.index == 0) {
        dev.first_fmt = fdesc.pixelformat;
      }
      if (fdesc.pixelformat == V4L2_PIX_FMT_RGB565) {
        dev.supports_rgb565 = true;
      }
      fdesc.index++;
    }

    struct v4l2_format test_fmt = {};
    test_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &test_fmt) == 0) {
      dev.default_fmt = test_fmt.fmt.pix.pixelformat;
      dev.default_w = test_fmt.fmt.pix.width;
      dev.default_h = test_fmt.fmt.pix.height;
    }

    open_devs.push_back(dev);
  }

  if (open_devs.empty()) {
    char buf[128];
    snprintf(buf, sizeof(buf), "No video device openable (errno=%d)", errno);
    this->status_msg_ = buf;
    ESP_LOGE(TAG, "%s", buf);
    this->status_set_error(LOG_STR("No video device found"));
    this->mark_failed();
    return;
  }

  // Choose the best capture device
  int chosen_idx = -1;
  for (size_t i = 0; i < open_devs.size(); i++) {
    if (strcmp(open_devs[i].path, "/dev/video20") == 0 && open_devs.size() > 1) {
      continue;
    }
    if (open_devs[i].supports_rgb565) {
      chosen_idx = (int)i;
      break;
    }
  }
  if (chosen_idx < 0) {
    for (size_t i = 0; i < open_devs.size(); i++) {
      if (strcmp(open_devs[i].path, "/dev/video20") != 0) {
        chosen_idx = (int)i;
        break;
      }
    }
  }
  if (chosen_idx < 0 && !open_devs.empty()) {
    chosen_idx = 0;
  }

  this->video_fd_ = open_devs[chosen_idx].fd;
  this->chosen_path_ = open_devs[chosen_idx].path;
  ESP_LOGI(TAG, "Selected video device: %s (fd=%d)", this->chosen_path_.c_str(), this->video_fd_);

  // Check if ISP device /dev/video20 is present and keep its fd for controls
  this->isp_fd_ = -1;
  for (size_t i = 0; i < open_devs.size(); i++) {
    if (strcmp(open_devs[i].path, "/dev/video20") == 0) {
      this->isp_fd_ = open_devs[i].fd;
      ESP_LOGD(TAG, "ISP device /dev/video20 retained (fd=%d)", this->isp_fd_);
      break;
    }
  }

  // Close unused devices
  for (size_t i = 0; i < open_devs.size(); i++) {
    if ((int)i != chosen_idx && open_devs[i].fd != this->isp_fd_) {
      close(open_devs[i].fd);
    }
  }

  // Step 3: Set pixel format
  struct v4l2_format req_fmt = {};
  req_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req_fmt.fmt.pix.width = SENSOR_WIDTH;
  req_fmt.fmt.pix.height = SENSOR_HEIGHT;
  req_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;

  if (ioctl(this->video_fd_, VIDIOC_S_FMT, &req_fmt) != 0) {
    ESP_LOGW(TAG, "VIDIOC_S_FMT RGB565 failed: errno=%d (%s)", errno, strerror(errno));

    bool fallback_ok = false;
    if (open_devs[chosen_idx].default_fmt != 0) {
      struct v4l2_format def_fmt = {};
      def_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      def_fmt.fmt.pix.width = open_devs[chosen_idx].default_w;
      def_fmt.fmt.pix.height = open_devs[chosen_idx].default_h;
      def_fmt.fmt.pix.pixelformat = open_devs[chosen_idx].default_fmt;
      if (ioctl(this->video_fd_, VIDIOC_S_FMT, &def_fmt) == 0) {
        req_fmt = def_fmt;
        fallback_ok = true;
      }
    }

    if (!fallback_ok && open_devs[chosen_idx].first_fmt != 0) {
      struct v4l2_format first_fmt = {};
      first_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      first_fmt.fmt.pix.width = SENSOR_WIDTH;
      first_fmt.fmt.pix.height = SENSOR_HEIGHT;
      first_fmt.fmt.pix.pixelformat = open_devs[chosen_idx].first_fmt;
      if (ioctl(this->video_fd_, VIDIOC_S_FMT, &first_fmt) == 0) {
        req_fmt = first_fmt;
      }
    }
  }

  // Verify format
  struct v4l2_format cur_fmt = {};
  cur_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_G_FMT, &cur_fmt) == 0) {
    this->frame_width_ = cur_fmt.fmt.pix.width;
    this->frame_height_ = cur_fmt.fmt.pix.height;
  } else {
    this->frame_width_ = SENSOR_WIDTH;
    this->frame_height_ = SENSOR_HEIGHT;
  }
  ESP_LOGI(TAG, "Camera format: %dx%d", this->frame_width_, this->frame_height_);

  // Step 4: Request frame buffers
  struct v4l2_requestbuffers req = {};
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) != 0) {
    char buf[128];
    snprintf(buf, sizeof(buf), "VIDIOC_REQBUFS failed: errno=%d (%s)", errno, strerror(errno));
    this->status_msg_ = buf;
    ESP_LOGE(TAG, "%s", buf);
    this->status_set_error(LOG_STR("VIDIOC_REQBUFS failed"));
    this->mark_failed();
    return;
  }

  uint32_t actual_buffers = req.count;
  if (actual_buffers > 6) actual_buffers = 6;

  // Step 5: Query and mmap buffers
  for (uint32_t i = 0; i < actual_buffers; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(this->video_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      char ebuf[64];
      snprintf(ebuf, sizeof(ebuf), "VIDIOC_QUERYBUF failed for buf %d (errno=%d)", (int)i, errno);
      this->status_msg_ = ebuf;
      ESP_LOGE(TAG, "%s", ebuf);
      this->status_set_error(LOG_STR("VIDIOC_QUERYBUF failed"));
      this->mark_failed();
      return;
    }

    this->frame_buffers_[i] = (uint8_t *)esp_video_mmap(
        nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, this->video_fd_, buf.m.offset);
    if (!this->frame_buffers_[i]) {
      char ebuf[64];
      snprintf(ebuf, sizeof(ebuf), "mmap failed for buffer %d", (int)i);
      this->status_msg_ = ebuf;
      ESP_LOGE(TAG, "%s", ebuf);
      this->status_set_error(LOG_STR("mmap failed"));
      this->mark_failed();
      return;
    }
    this->frame_buf_size_ = buf.length;

    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
      char ebuf[64];
      snprintf(ebuf, sizeof(ebuf), "VIDIOC_QBUF failed for buf %d (errno=%d)", (int)i, errno);
      this->status_msg_ = ebuf;
      ESP_LOGE(TAG, "%s", ebuf);
      this->status_set_error(LOG_STR("VIDIOC_QBUF failed"));
      this->mark_failed();
      return;
    }
  }

  // Step 6: Start streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_STREAMON, &type) != 0) {
    char ebuf[64];
    snprintf(ebuf, sizeof(ebuf), "VIDIOC_STREAMON failed: errno=%d", errno);
    this->status_msg_ = ebuf;
    ESP_LOGE(TAG, "%s", ebuf);
    this->status_set_error(LOG_STR("VIDIOC_STREAMON failed"));
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Video streaming started!");

  // Step 7: Apply hardware ISP white balance / controls
  this->apply_hardware_controls_();

  // Step 8: Init JPEG encoder
  jpeg_encode_engine_cfg_t jpeg_cfg = {};
  jpeg_cfg.timeout_ms = 1000;
  err = jpeg_new_encoder_engine(&jpeg_cfg, &this->jpeg_handle_);
  if (err != ESP_OK) {
    char ebuf[64];
    snprintf(ebuf, sizeof(ebuf), "JPEG encoder init failed: %s", esp_err_to_name(err));
    this->status_msg_ = ebuf;
    ESP_LOGE(TAG, "%s", ebuf);
    this->status_set_error(LOG_STR("JPEG encoder init failed"));
    this->mark_failed();
    return;
  }

  this->jpeg_out_buf_size_ = this->frame_buf_size_;
  jpeg_encode_memory_alloc_cfg_t mem_cfg = {};
  mem_cfg.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
  this->jpeg_out_buf_ = (uint8_t *)jpeg_alloc_encoder_mem(this->jpeg_out_buf_size_, &mem_cfg, &this->jpeg_out_buf_size_);
  if (!this->jpeg_out_buf_) {
    this->status_msg_ = "Failed to alloc JPEG output buffer";
    ESP_LOGE(TAG, "%s", this->status_msg_.c_str());
    this->status_set_error(LOG_STR("JPEG buf alloc failed"));
    this->mark_failed();
    return;
  }

  this->jpeg_buffer_.data = this->jpeg_out_buf_;
  this->jpeg_buffer_.width = this->frame_width_;
  this->jpeg_buffer_.height = this->frame_height_;

  this->status_msg_ = "Initialized successfully";
  this->initialized_ = true;
  ESP_LOGI(TAG, "Camera initialized successfully! Streaming %dx%d (Color correction: %s, R:%.2f G:%.2f B:%.2f)",
           this->frame_width_, this->frame_height_,
           this->color_correction_ ? "ON" : "OFF",
           this->red_gain_, this->green_gain_, this->blue_gain_);
}

void P4CSICamera::loop() {
  if (!this->initialized_) return;

  esp_task_wdt_reset();

  if (!this->can_return_image_()) {
    return;
  }
  this->current_image_.reset();

  uint32_t now = millis();

  if (this->idle_update_interval_ != 0 && (now - this->last_idle_request_ >= this->idle_update_interval_)) {
    this->last_idle_request_ = now;
    this->request_image(camera::IDLE);
  }

  if (!this->has_requested_image_()) {
    return;
  }

  bool streaming = this->stream_requesters_ != 0;
  uint32_t interval = streaming ? this->max_update_interval_ : this->idle_update_interval_;
  if (interval > 0 && (now - this->last_update_ < interval)) {
    return;
  }

  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->video_fd_, VIDIOC_DQBUF, &buf) != 0) {
    if (errno != EAGAIN) {
      ESP_LOGW(TAG, "VIDIOC_DQBUF failed: errno=%d", errno);
    }
    return;
  }

  uint8_t *frame_data = this->frame_buffers_[buf.index];
  size_t frame_size = buf.bytesused > 0 ? buf.bytesused : this->frame_buf_size_;

  // Apply color correction / white balance LUT if enabled to fix green cast
  if (this->color_correction_ && (this->red_gain_ != 1.0f || this->green_gain_ != 1.0f || this->blue_gain_ != 1.0f)) {
    uint16_t *pixels = (uint16_t *)frame_data;
    size_t num_pixels = (size_t)this->frame_width_ * this->frame_height_;
    for (size_t i = 0; i < num_pixels; i++) {
      uint16_t p = pixels[i];
      pixels[i] = (this->r_lut_[(p >> 11) & 0x1F] << 11) |
                  (this->g_lut_[(p >> 5) & 0x3F] << 5) |
                  this->b_lut_[p & 0x1F];
    }
  }

  // Software vertical / horizontal flip only if enabled
  if (this->vertical_flip_ || this->horizontal_flip_) {
    uint16_t *pixels = (uint16_t *)frame_data;
    if (this->vertical_flip_) {
      std::vector<uint16_t> row_buf(this->frame_width_);
      for (int y = 0; y < this->frame_height_ / 2; y++) {
        uint16_t *top = pixels + y * this->frame_width_;
        uint16_t *bot = pixels + (this->frame_height_ - 1 - y) * this->frame_width_;
        memcpy(row_buf.data(), top, this->frame_width_ * sizeof(uint16_t));
        memcpy(top, bot, this->frame_width_ * sizeof(uint16_t));
        memcpy(bot, row_buf.data(), this->frame_width_ * sizeof(uint16_t));
      }
    }
    if (this->horizontal_flip_) {
      for (int y = 0; y < this->frame_height_; y++) {
        uint16_t *row = pixels + y * this->frame_width_;
        for (int x = 0; x < this->frame_width_ / 2; x++) {
          uint16_t tmp = row[x];
          row[x] = row[this->frame_width_ - 1 - x];
          row[this->frame_width_ - 1 - x] = tmp;
        }
      }
    }
  }

  esp_task_wdt_reset();

  jpeg_encode_cfg_t enc_cfg = {};
  enc_cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
  enc_cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
  enc_cfg.image_quality = this->jpeg_quality_;
  enc_cfg.width = this->frame_width_;
  enc_cfg.height = this->frame_height_;

  uint32_t jpeg_size = 0;
  esp_err_t err = jpeg_encoder_process(this->jpeg_handle_, &enc_cfg,
                                       frame_data, frame_size,
                                       this->jpeg_out_buf_, this->jpeg_out_buf_size_,
                                       &jpeg_size);

  if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGW(TAG, "VIDIOC_QBUF failed");
  }

  if (err != ESP_OK || jpeg_size == 0) {
    ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(err));
    return;
  }

  this->last_update_ = now;
  ESP_LOGD(TAG, "Frame: %dx%d, JPEG: %u bytes", this->frame_width_, this->frame_height_, (unsigned int)jpeg_size);

  uint8_t requesters = this->single_requesters_ | this->stream_requesters_;
  this->single_requesters_ = 0;

  auto image = std::make_shared<P4CSICameraImage>(this->jpeg_out_buf_, (size_t)jpeg_size, requesters);
  this->current_image_ = image;
  this->new_image_callback_.call(image);
  for (auto *listener : this->listeners_) {
    listener->on_camera_image(image);
  }
}

void P4CSICamera::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32-P4 CSI Camera (OV02C10):");
  ESP_LOGCONFIG(TAG, "  Device: %s (fd=%d)", this->chosen_path_.c_str(), this->video_fd_);
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Data Lanes: %d", this->data_lanes_);
  ESP_LOGCONFIG(TAG, "  Max Update Interval: %d ms", (int)this->max_update_interval_);
  ESP_LOGCONFIG(TAG, "  Color Correction: %s (R: %.2f, G: %.2f, B: %.2f)",
                this->color_correction_ ? "ON" : "OFF",
                this->red_gain_, this->green_gain_, this->blue_gain_);
  ESP_LOGCONFIG(TAG, "  Auto White Balance: %s", this->auto_white_balance_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  Initialized: %s", this->initialized_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Status: %s", this->status_msg_.c_str());
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Camera FAILED: %s", this->status_msg_.c_str());
  }
}

// Camera interface
void P4CSICamera::start_stream(camera::CameraRequester requester) {
  this->stream_requesters_ |= (1 << (uint8_t)requester);
}
void P4CSICamera::stop_stream(camera::CameraRequester requester) {
  this->stream_requesters_ &= ~(1 << (uint8_t)requester);
}
void P4CSICamera::request_image(camera::CameraRequester requester) {
  this->single_requesters_ |= (1 << (uint8_t)requester);
}
void P4CSICamera::add_listener(camera::CameraListener *listener) {
  this->listeners_.push_back(listener);
}
void P4CSICamera::add_image_callback(
    std::function<void(std::shared_ptr<camera::CameraImage>)> &&callback) {
  this->new_image_callback_.add(std::move(callback));
}
camera::CameraImageReader *P4CSICamera::create_image_reader() {
  return new P4CSICameraImageReader();
}
bool P4CSICamera::has_requested_image_() const {
  return this->single_requesters_ != 0 || this->stream_requesters_ != 0;
}

bool P4CSICamera::can_return_image_() const {
  return this->current_image_ == nullptr || this->current_image_.use_count() == 1;
}

void P4CSICamera::set_jpeg_quality_runtime(uint8_t quality) {
  this->jpeg_quality_ = quality;
  ESP_LOGI(TAG, "JPEG quality set to %d", quality);
}

void P4CSICamera::set_vertical_flip_runtime(bool flip) {
  this->vertical_flip_ = flip;
  ESP_LOGI(TAG, "Vertical flip set to %s", flip ? "ON" : "OFF");
}

void P4CSICamera::set_horizontal_flip_runtime(bool flip) {
  this->horizontal_flip_ = flip;
  ESP_LOGI(TAG, "Horizontal flip set to %s", flip ? "ON" : "OFF");
}

void P4CSICamera::set_red_gain_runtime(float gain) {
  this->red_gain_ = gain;
  this->recompute_lut_();
  this->apply_hardware_controls_();
  ESP_LOGI(TAG, "Red gain set to %.2f", gain);
}

void P4CSICamera::set_green_gain_runtime(float gain) {
  this->green_gain_ = gain;
  this->recompute_lut_();
  this->apply_hardware_controls_();
  ESP_LOGI(TAG, "Green gain set to %.2f", gain);
}

void P4CSICamera::set_blue_gain_runtime(float gain) {
  this->blue_gain_ = gain;
  this->recompute_lut_();
  this->apply_hardware_controls_();
  ESP_LOGI(TAG, "Blue gain set to %.2f", gain);
}

void P4CSICamera::set_color_correction_runtime(bool enable) {
  this->color_correction_ = enable;
  ESP_LOGI(TAG, "Color correction set to %s", enable ? "ON" : "OFF");
}

void P4CSICamera::set_auto_white_balance_runtime(bool enable) {
  this->auto_white_balance_ = enable;
  this->apply_hardware_controls_();
  ESP_LOGI(TAG, "Auto white balance set to %s", enable ? "ON" : "OFF");
}

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
