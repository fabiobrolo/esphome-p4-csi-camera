#include "p4_csi_camera.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_video_init.h"
#include "esp_task_wdt.h"
#include "linux/videodev2.h"
#include "driver/jpeg_encode.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>

namespace esphome {
namespace p4_csi_camera {

static const char *const TAG = "p4_csi_camera";

static constexpr int NUM_FRAME_BUFFERS = 4;
static constexpr uint16_t SENSOR_WIDTH = 1928;
static constexpr uint16_t SENSOR_HEIGHT = 1092;

P4CSICamera *global_p4_csi_camera = nullptr;  // NOLINT

// ===========================================================================
// P4CSICameraImage
// ===========================================================================

P4CSICameraImage::P4CSICameraImage(P4CameraFrameBuffer *buffer, uint8_t requesters)
    : buffer_(buffer), requesters_(requesters) {}

uint8_t *P4CSICameraImage::get_data_buffer() { return this->buffer_->data; }
size_t P4CSICameraImage::get_data_length() { return this->buffer_->length; }
bool P4CSICameraImage::was_requested_by(camera::CameraRequester requester) const {
  return this->requesters_ & (1 << (uint8_t)requester);
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
  return this->image_->get_data_length() - this->offset_;
}
uint8_t *P4CSICameraImageReader::peek_data_buffer() {
  if (!this->image_) return nullptr;
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

P4CSICamera::P4CSICamera() { global_p4_csi_camera = this; }

void P4CSICamera::setup() {
  ESP_LOGI(TAG, "Setting up ESP32-P4 CSI Camera (OV02C10) via esp_video...");
  ESP_LOGI(TAG, "  Data lanes: %d", this->data_lanes_);
  ESP_LOGI(TAG, "  Max update interval: %d ms", this->max_update_interval_);

  // Step 1: Initialize the video subsystem (handles CSI + ISP + sensor detection)
  esp_video_init_csi_config_t csi_config = {};
  csi_config.sccb_config.init_sccb = true;  // Let esp_video manage sensor I2C
  csi_config.sccb_config.i2c_config.port = 0;  // Port 0 - same as where sensor lives
  csi_config.sccb_config.i2c_config.scl_pin = (gpio_num_t)8;
  csi_config.sccb_config.i2c_config.sda_pin = (gpio_num_t)7;
  csi_config.sccb_config.freq = 100000;
  csi_config.reset_pin = (gpio_num_t)-1;  // No reset pin
  csi_config.pwdn_pin = (gpio_num_t)-1;   // No power down pin

  esp_video_init_config_t cam_config = {};
  cam_config.csi = &csi_config;

  esp_err_t err = esp_video_init(&cam_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "esp_video initialized successfully");

  // Try multiple device paths
  const char *dev_paths[] = {"/dev/video0", "/dev/video1", "/dev/video2", "/dev/video10", "/dev/video20"};
  for (int i = 0; i < 5; i++) {
    int fd = open(dev_paths[i], O_RDONLY | O_NONBLOCK, 0);
    if (fd >= 0) {
      ESP_LOGI(TAG, "Found video device at %s (fd=%d)", dev_paths[i], fd);
      this->video_fd_ = fd;
      break;
    } else {
      ESP_LOGD(TAG, "  %s: not found (errno=%d)", dev_paths[i], errno);
    }
  }

  if (this->video_fd_ < 0) {
    ESP_LOGE(TAG, "No video device found at any path");
    this->mark_failed();
    return;
  }

  // Step 3: Set the pixel format to RGB565
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_G_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "VIDIOC_G_FMT failed");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Default format: %dx%d, pixfmt=0x%08x",
           fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat);

  // Request RGB565 output format
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  if (ioctl(this->video_fd_, VIDIOC_S_FMT, &fmt) != 0) {
    ESP_LOGW(TAG, "VIDIOC_S_FMT RGB565 failed, using default format");
  }

  this->frame_width_ = fmt.fmt.pix.width;
  this->frame_height_ = fmt.fmt.pix.height;
  ESP_LOGI(TAG, "Using format: %dx%d", this->frame_width_, this->frame_height_);

  // Step 4: Request frame buffers
  struct v4l2_requestbuffers req = {};
  req.count = NUM_FRAME_BUFFERS;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
    this->mark_failed();
    return;
  }

  // Step 5: Map frame buffers
  for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(this->video_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QUERYBUF failed for buf %d", i);
      this->mark_failed();
      return;
    }

    this->frame_buffers_[i] = (uint8_t *)mmap(NULL, buf.length,
        PROT_READ | PROT_WRITE, MAP_SHARED, this->video_fd_, buf.m.offset);
    if (this->frame_buffers_[i] == nullptr) {
      ESP_LOGE(TAG, "mmap failed for buf %d", i);
      this->mark_failed();
      return;
    }
    this->frame_buf_size_ = buf.length;

    // Queue the buffer
    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QBUF failed for buf %d", i);
      this->mark_failed();
      return;
    }
  }

  ESP_LOGI(TAG, "Mapped %d frame buffers (%d bytes each)", NUM_FRAME_BUFFERS, this->frame_buf_size_);

  // Step 6: Start streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Video streaming started!");

  // Apply V4L2 controls (flip/mirror)
  if (this->vertical_flip_ || this->horizontal_flip_) {
    struct v4l2_ext_controls controls = {};
    struct v4l2_ext_control control[1] = {};
    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    controls.count = 1;
    controls.controls = control;

    if (this->vertical_flip_) {
      control[0].id = V4L2_CID_VFLIP;
      control[0].value = 1;
      if (ioctl(this->video_fd_, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "Failed to set vertical flip");
      }
    }
    if (this->horizontal_flip_) {
      control[0].id = V4L2_CID_HFLIP;
      control[0].value = 1;
      if (ioctl(this->video_fd_, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "Failed to set horizontal flip");
      }
    }
  }

  // Step 7: Init JPEG encoder
  jpeg_encode_engine_cfg_t jpeg_cfg = {};
  jpeg_cfg.timeout_ms = 1000;
  err = jpeg_new_encoder_engine(&jpeg_cfg, &this->jpeg_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "JPEG encoder init failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  // Allocate JPEG output buffer using the JPEG encoder's aligned allocator
  this->jpeg_out_buf_size_ = this->frame_buf_size_;
  jpeg_encode_memory_alloc_cfg_t mem_cfg = {};
  mem_cfg.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
  this->jpeg_out_buf_ = (uint8_t *)jpeg_alloc_encoder_mem(this->jpeg_out_buf_size_, &mem_cfg, &this->jpeg_out_buf_size_);
  if (!this->jpeg_out_buf_) {
    ESP_LOGE(TAG, "Failed to alloc JPEG output buffer");
    this->mark_failed();
    return;
  }

  this->jpeg_buffer_.data = this->jpeg_out_buf_;
  this->jpeg_buffer_.width = this->frame_width_;
  this->jpeg_buffer_.height = this->frame_height_;

  this->initialized_ = true;
  ESP_LOGI(TAG, "Camera initialized successfully! Streaming %dx%d", this->frame_width_, this->frame_height_);
}

void P4CSICamera::loop() {
  if (!this->initialized_) return;

  uint32_t now = millis();
  bool needs_image = this->has_requested_image_();
  bool streaming = this->stream_requesters_ != 0;
  uint32_t interval = streaming ? this->max_update_interval_ : this->idle_update_interval_;

  if (now - this->last_update_ < interval) return;

  // Dequeue a frame
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

  // Feed the watchdog before heavy encode operation
  esp_task_wdt_reset();

  // Encode directly from the V4L2 buffer (hardware JPEG encoder uses DMA, fast)
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

  // Return the buffer after encoding
  if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGW(TAG, "VIDIOC_QBUF failed");
  }

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(err));
    return;
  }

  this->jpeg_buffer_.length = (size_t)jpeg_size;
  this->last_update_ = now;

  ESP_LOGD(TAG, "Frame: %dx%d, JPEG: %d bytes", this->frame_width_, this->frame_height_, jpeg_size);

  if (needs_image || streaming) {
    uint8_t requesters = this->single_requesters_ | this->stream_requesters_;
    this->single_requesters_ = 0;

    auto image = std::make_shared<P4CSICameraImage>(&this->jpeg_buffer_, requesters);
    this->current_image_ = image;
    this->new_image_callback_.call(image);
    for (auto *listener : this->listeners_) {
      listener->on_camera_image(image);
    }
    this->current_image_.reset();
  }
}

void P4CSICamera::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32-P4 CSI Camera (OV02C10):");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Data Lanes: %d", this->data_lanes_);
  ESP_LOGCONFIG(TAG, "  Max Update Interval: %d ms", this->max_update_interval_);
  ESP_LOGCONFIG(TAG, "  Initialized: %s", this->initialized_ ? "YES" : "NO");
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
  return this->single_requesters_ != 0;
}

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
