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
#include "esp_cam_sensor_detect.h"
#include "ov02c10.h"

// Force symbol reference so linker extracts ov02c10_sensor.o from static library
extern "C" esp_cam_sensor_device_t *ov02c10_detect(esp_cam_sensor_config_t *config);
void *const force_ov02c10_link_ref __attribute__((used)) = (void *)ov02c10_detect;

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>

#if defined(USE_ESP32_FRAMEWORK_ESP_IDF)
#include "esphome/components/i2c/i2c_bus.h"
#define protected public
#define private public
#include "esphome/components/i2c/i2c_bus_esp_idf.h"
#undef protected
#undef private
#endif

namespace esphome {
namespace p4_csi_camera {

static const char *const TAG = "p4_csi_camera";

static constexpr int NUM_FRAME_BUFFERS = 2;
static constexpr uint16_t SENSOR_WIDTH = 1920;
static constexpr uint16_t SENSOR_HEIGHT = 1080;

P4CSICamera *global_p4_csi_camera = nullptr;  // NOLINT

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

P4CSICamera::P4CSICamera() { global_p4_csi_camera = this; }

void P4CSICamera::setup() {
  ESP_LOGI(TAG, "Setting up ESP32-P4 CSI Camera (OV02C10) via esp_video...");
  ESP_LOGI(TAG, "  Data lanes: %d", this->data_lanes_);
  ESP_LOGI(TAG, "  Max update interval: %d ms", (int)this->max_update_interval_);

  // Step 1: Initialize the video subsystem (handles CSI + ISP + sensor detection)
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
    ESP_LOGI(TAG, "  Using shared ESPHome I2C bus for sensor SCCB (handle=%p)", idf_bus->bus_);
    csi_config.sccb_config.init_sccb = false;
    csi_config.sccb_config.i2c_handle = idf_bus->bus_;
    csi_config.sccb_config.freq = 400000;
  } else {
#endif
    ESP_LOGI(TAG, "  Using dedicated SCCB I2C config on GPIO 7/8");
    csi_config.sccb_config.init_sccb = true;  // Let esp_video manage sensor I2C
    csi_config.sccb_config.i2c_config.port = 0;  // Port 0 - same as where sensor lives
    csi_config.sccb_config.i2c_config.scl_pin = (gpio_num_t)8;
    csi_config.sccb_config.i2c_config.sda_pin = (gpio_num_t)7;
    csi_config.sccb_config.freq = 400000;
#if defined(USE_ESP32_FRAMEWORK_ESP_IDF)
  }
#endif
  csi_config.reset_pin = (gpio_num_t)-1;  // No reset pin
  csi_config.pwdn_pin = (gpio_num_t)-1;   // No power down pin

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

  // Try multiple device paths with O_RDWR
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

  this->diag_info_.clear();
  this->diag_info_ += "=== P4 CSI Video Devices ===\n";
  char dbuf[256];

  ESP_LOGI(TAG, "OV02C10 driver symbol reference: %p", (void *)ov02c10_detect);

  // Inspect the registered camera sensor detection table via official API
  esp_cam_sensor_detect_fn_t *detect_start = nullptr;
  esp_cam_sensor_detect_fn_t *detect_end = nullptr;
  esp_cam_sensor_detect_get_array(&detect_start, &detect_end);
  int num_detect_fn = (detect_end && detect_start) ? (int)(detect_end - detect_start) : 0;
  snprintf(dbuf, sizeof(dbuf), "Registered cam sensor detect functions: %d\n", num_detect_fn);
  this->diag_info_ += dbuf;
  ESP_LOGI(TAG, "Registered camera sensor detect functions: %d", num_detect_fn);
  for (int i = 0; i < num_detect_fn; i++) {
    snprintf(dbuf, sizeof(dbuf), "  detect_fn[%d]: port=%d, sccb_addr=0x%02x, fn=%p\n",
             i, detect_start[i].port, detect_start[i].sccb_addr, (void *)detect_start[i].detect);
    this->diag_info_ += dbuf;
    ESP_LOGI(TAG, "  detect_fn[%d]: port=%d, sccb_addr=0x%02x, fn=%p",
             i, detect_start[i].port, detect_start[i].sccb_addr, (void *)detect_start[i].detect);
  }

  for (const char *path : dev_paths) {
    int fd = open(path, O_RDWR, 0);
    if (fd < 0) {
      int err = errno;
      int fd_ro = open(path, O_RDONLY, 0);
      if (fd_ro >= 0) {
        fd = fd_ro;
        snprintf(dbuf, sizeof(dbuf), "Device %s: opened with O_RDONLY (fd=%d)\n", path, fd);
        this->diag_info_ += dbuf;
      } else {
        snprintf(dbuf, sizeof(dbuf), "Device %s: open failed errno=%d (%s)\n", path, err, strerror(err));
        this->diag_info_ += dbuf;
        continue;
      }
    }

    OpenDev dev = {};
    dev.path = path;
    dev.fd = fd;

    struct v4l2_capability cap = {};
    int cap_ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    snprintf(dbuf, sizeof(dbuf), "Device %s (fd=%d): driver='%s' card='%s' caps=0x%08x\n",
             path, fd, (cap_ret == 0 ? (char*)cap.driver : "err"),
             (cap_ret == 0 ? (char*)cap.card : "err"), (cap_ret == 0 ? (unsigned int)cap.capabilities : 0));
    this->diag_info_ += dbuf;

    // Enumerate formats
    struct v4l2_fmtdesc fdesc = {};
    fdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fdesc.index = 0;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {
      char fourcc[5] = {
        (char)((fdesc.pixelformat >> 0) & 0xFF),
        (char)((fdesc.pixelformat >> 8) & 0xFF),
        (char)((fdesc.pixelformat >> 16) & 0xFF),
        (char)((fdesc.pixelformat >> 24) & 0xFF),
        0
      };
      snprintf(dbuf, sizeof(dbuf), "  fmt[%d]: '%s' (0x%08x) - %s\n",
               (int)fdesc.index, fourcc, (unsigned int)fdesc.pixelformat, fdesc.description);
      this->diag_info_ += dbuf;

      if (fdesc.index == 0) {
        dev.first_fmt = fdesc.pixelformat;
      }
      if (fdesc.pixelformat == V4L2_PIX_FMT_RGB565) {
        dev.supports_rgb565 = true;
      }
      fdesc.index++;
    }

    // Default format
    struct v4l2_format test_fmt = {};
    test_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &test_fmt) == 0) {
      char fourcc[5] = {
        (char)((test_fmt.fmt.pix.pixelformat >> 0) & 0xFF),
        (char)((test_fmt.fmt.pix.pixelformat >> 8) & 0xFF),
        (char)((test_fmt.fmt.pix.pixelformat >> 16) & 0xFF),
        (char)((test_fmt.fmt.pix.pixelformat >> 24) & 0xFF),
        0
      };
      dev.default_fmt = test_fmt.fmt.pix.pixelformat;
      dev.default_w = test_fmt.fmt.pix.width;
      dev.default_h = test_fmt.fmt.pix.height;
      snprintf(dbuf, sizeof(dbuf), "  G_FMT: %dx%d pixfmt='%s' (0x%08x)\n",
               (int)test_fmt.fmt.pix.width, (int)test_fmt.fmt.pix.height, fourcc, (unsigned int)test_fmt.fmt.pix.pixelformat);
      this->diag_info_ += dbuf;
    } else {
      snprintf(dbuf, sizeof(dbuf), "  G_FMT failed: errno=%d (%s)\n", errno, strerror(errno));
      this->diag_info_ += dbuf;
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

  // Choose the best device: skip metadata devices like /dev/video20
  int chosen_idx = -1;
  for (size_t i = 0; i < open_devs.size(); i++) {
    if (strcmp(open_devs[i].path, "/dev/video20") == 0 && open_devs.size() > 1) {
      continue;
    }
    if (open_devs[i].supports_rgb565) {
      chosen_idx = i;
      break;
    }
  }
  if (chosen_idx < 0) {
    for (size_t i = 0; i < open_devs.size(); i++) {
      if (strcmp(open_devs[i].path, "/dev/video20") != 0) {
        chosen_idx = i;
        break;
      }
    }
  }
  if (chosen_idx < 0 && !open_devs.empty()) {
    chosen_idx = 0;
  }

  this->video_fd_ = open_devs[chosen_idx].fd;
  this->chosen_path_ = open_devs[chosen_idx].path;
  snprintf(dbuf, sizeof(dbuf), "Selected device: %s (fd=%d, supports_rgb565=%d)\n",
           this->chosen_path_.c_str(), this->video_fd_, open_devs[chosen_idx].supports_rgb565);
  this->diag_info_ += dbuf;
  ESP_LOGI(TAG, "%s", dbuf);

  // Close other fds
  for (size_t i = 0; i < open_devs.size(); i++) {
    if ((int)i != chosen_idx) {
      close(open_devs[i].fd);
    }
  }

  // Step 3: Set the pixel format
  struct v4l2_format req_fmt = {};
  req_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req_fmt.fmt.pix.width = SENSOR_WIDTH;
  req_fmt.fmt.pix.height = SENSOR_HEIGHT;
  req_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;

  snprintf(dbuf, sizeof(dbuf), "Calling S_FMT RGB565 (%dx%d)...", SENSOR_WIDTH, SENSOR_HEIGHT);
  this->diag_info_ += dbuf;

  if (ioctl(this->video_fd_, VIDIOC_S_FMT, &req_fmt) != 0) {
    snprintf(dbuf, sizeof(dbuf), " FAILED errno=%d (%s)\n", errno, strerror(errno));
    this->diag_info_ += dbuf;
    ESP_LOGW(TAG, "VIDIOC_S_FMT RGB565 failed: errno=%d (%s)", errno, strerror(errno));

    // Try setting the device's default format from G_FMT
    bool fallback_ok = false;
    if (open_devs[chosen_idx].default_fmt != 0) {
      struct v4l2_format def_fmt = {};
      def_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      def_fmt.fmt.pix.width = open_devs[chosen_idx].default_w;
      def_fmt.fmt.pix.height = open_devs[chosen_idx].default_h;
      def_fmt.fmt.pix.pixelformat = open_devs[chosen_idx].default_fmt;

      snprintf(dbuf, sizeof(dbuf), "Calling S_FMT with default (%dx%d, 0x%08x)...",
               (int)def_fmt.fmt.pix.width, (int)def_fmt.fmt.pix.height, (unsigned int)def_fmt.fmt.pix.pixelformat);
      this->diag_info_ += dbuf;

      if (ioctl(this->video_fd_, VIDIOC_S_FMT, &def_fmt) == 0) {
        this->diag_info_ += " OK!\n";
        req_fmt = def_fmt;
        fallback_ok = true;
      } else {
        snprintf(dbuf, sizeof(dbuf), " FAILED errno=%d (%s)\n", errno, strerror(errno));
        this->diag_info_ += dbuf;
      }
    }

    if (!fallback_ok && open_devs[chosen_idx].first_fmt != 0) {
      struct v4l2_format first_fmt = {};
      first_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      first_fmt.fmt.pix.width = SENSOR_WIDTH;
      first_fmt.fmt.pix.height = SENSOR_HEIGHT;
      first_fmt.fmt.pix.pixelformat = open_devs[chosen_idx].first_fmt;

      snprintf(dbuf, sizeof(dbuf), "Calling S_FMT with first enumerated fmt (0x%08x)...",
               (unsigned int)first_fmt.fmt.pix.pixelformat);
      this->diag_info_ += dbuf;

      if (ioctl(this->video_fd_, VIDIOC_S_FMT, &first_fmt) == 0) {
        this->diag_info_ += " OK!\n";
        req_fmt = first_fmt;
        fallback_ok = true;
      } else {
        snprintf(dbuf, sizeof(dbuf), " FAILED errno=%d (%s)\n", errno, strerror(errno));
        this->diag_info_ += dbuf;
      }
    }

    if (!fallback_ok) {
      char buf[64];
      snprintf(buf, sizeof(buf), "VIDIOC_S_FMT failed: errno=%d", errno);
      this->status_msg_ = buf;
      this->status_set_error(LOG_STR("VIDIOC_S_FMT failed"));
      this->mark_failed();
      return;
    }
  } else {
    this->diag_info_ += " OK!\n";
  }

  // Re-read format to see what was committed
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_G_FMT, &fmt) == 0) {
    char fourcc[5] = {
      (char)((fmt.fmt.pix.pixelformat >> 0) & 0xFF),
      (char)((fmt.fmt.pix.pixelformat >> 8) & 0xFF),
      (char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF),
      (char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF),
      0
    };
    snprintf(dbuf, sizeof(dbuf), "Final format after S_FMT: %dx%d, pixfmt='%s' (0x%08x)\n",
             (int)fmt.fmt.pix.width, (int)fmt.fmt.pix.height, fourcc, (unsigned int)fmt.fmt.pix.pixelformat);
    this->diag_info_ += dbuf;
    ESP_LOGI(TAG, "%s", dbuf);
    this->frame_width_ = fmt.fmt.pix.width;
    this->frame_height_ = fmt.fmt.pix.height;
  } else {
    this->frame_width_ = req_fmt.fmt.pix.width;
    this->frame_height_ = req_fmt.fmt.pix.height;
  }
  ESP_LOGI(TAG, "Using format: %dx%d", this->frame_width_, this->frame_height_);

  // Step 4: Request frame buffers
  struct v4l2_requestbuffers req = {};
  req.count = NUM_FRAME_BUFFERS;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) != 0 || req.count == 0) {
    char buf[64];
    snprintf(buf, sizeof(buf), "VIDIOC_REQBUFS failed: errno=%d, count=%d", errno, (int)req.count);
    this->status_msg_ = buf;
    ESP_LOGE(TAG, "%s", buf);
    this->status_set_error(LOG_STR("VIDIOC_REQBUFS failed"));
    this->mark_failed();
    return;
  }

  uint32_t actual_buffers = req.count;
  ESP_LOGI(TAG, "VIDIOC_REQBUFS allocated %d buffers", (int)actual_buffers);

  // Step 5: Map frame buffers
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

    this->frame_buffers_[i] = (uint8_t *)mmap(NULL, buf.length,
        PROT_READ | PROT_WRITE, MAP_SHARED, this->video_fd_, buf.m.offset);
    if (this->frame_buffers_[i] == nullptr || this->frame_buffers_[i] == MAP_FAILED) {
      char ebuf[64];
      snprintf(ebuf, sizeof(ebuf), "mmap failed for buf %d (errno=%d)", (int)i, errno);
      this->status_msg_ = ebuf;
      ESP_LOGE(TAG, "%s", ebuf);
      this->status_set_error(LOG_STR("mmap failed"));
      this->mark_failed();
      return;
    }
    this->frame_buf_size_ = buf.length;

    // Queue the buffer
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

  ESP_LOGI(TAG, "Mapped %d frame buffers (%d bytes each)", (int)actual_buffers, (int)this->frame_buf_size_);

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

  // Step 7: Init JPEG encoder
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

  // Allocate JPEG output buffer using the JPEG encoder's aligned allocator
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
  ESP_LOGI(TAG, "Camera initialized successfully! Streaming %dx%d", this->frame_width_, this->frame_height_);
}

void P4CSICamera::loop() {
  if (!this->initialized_) return;

  // Feed watchdog every loop iteration to prevent WDT reset during idle
  esp_task_wdt_reset();

  // Step 1: Check if previous frame is still being read/transmitted by any listener
  if (!this->can_return_image_()) {
    return;
  }
  // Previous transmission finished; release our holding reference
  this->current_image_.reset();

  uint32_t now = millis();

  // Step 2: Handle idle frame requests
  if (this->idle_update_interval_ != 0 && (now - this->last_idle_request_ >= this->idle_update_interval_)) {
    this->last_idle_request_ = now;
    this->request_image(camera::IDLE);
  }

  // Step 3: Only process if someone actually requested an image
  if (!this->has_requested_image_()) {
    return;
  }

  // Step 4: Rate limit according to max_update_interval_ (or idle_update_interval_)
  bool streaming = this->stream_requesters_ != 0;
  uint32_t interval = streaming ? this->max_update_interval_ : this->idle_update_interval_;
  if (interval > 0 && (now - this->last_update_ < interval)) {
    return;
  }

  // Step 5: Dequeue a frame
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

  // Feed the watchdog before heavy encode operation
  esp_task_wdt_reset();

  // Step 6: Encode directly from the V4L2 buffer (hardware JPEG encoder uses DMA, fast)
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

  if (err != ESP_OK || jpeg_size == 0) {
    ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(err));
    return;
  }

  this->last_update_ = now;
  ESP_LOGD(TAG, "Frame: %dx%d, JPEG: %u bytes", this->frame_width_, this->frame_height_, (unsigned int)jpeg_size);

  // Step 7: Dispatch new image and hold reference in current_image_ while listeners transmit
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
  ESP_LOGCONFIG(TAG, "  Max Update Interval: %d ms", this->max_update_interval_);
  ESP_LOGCONFIG(TAG, "  Initialized: %s", this->initialized_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Status: %s", this->status_msg_.c_str());
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Camera FAILED: %s", this->status_msg_.c_str());
  }
  if (!this->diag_info_.empty()) {
    ESP_LOGI(TAG, "--- Diagnostics Begin ---");
    size_t start = 0;
    while (start < this->diag_info_.size()) {
      size_t end = this->diag_info_.find('\n', start);
      if (end == std::string::npos) end = this->diag_info_.size();
      std::string line = this->diag_info_.substr(start, end - start);
      if (!line.empty()) {
        ESP_LOGI(TAG, "  %s", line.c_str());
      }
      start = end + 1;
    }
    ESP_LOGI(TAG, "--- Diagnostics End ---");
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

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
