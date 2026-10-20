#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "p4_csi_camera.h"

namespace esphome {
namespace p4_csi_camera {

enum NumberType : uint8_t {
  NUMBER_TYPE_JPEG_QUALITY = 0,
  NUMBER_TYPE_RED_GAIN = 1,
  NUMBER_TYPE_GREEN_GAIN = 2,
  NUMBER_TYPE_BLUE_GAIN = 3,
};

class P4CSICameraNumber : public number::Number, public Component {
 public:
  void set_camera(P4CSICamera *camera) { this->camera_ = camera; }
  void set_number_type(NumberType type) { this->type_ = type; }

  void setup() override {
    float val = 0.0f;
    switch (this->type_) {
      case NUMBER_TYPE_JPEG_QUALITY:
        val = this->camera_->get_jpeg_quality();
        break;
      case NUMBER_TYPE_RED_GAIN:
        val = this->camera_->get_red_gain();
        break;
      case NUMBER_TYPE_GREEN_GAIN:
        val = this->camera_->get_green_gain();
        break;
      case NUMBER_TYPE_BLUE_GAIN:
        val = this->camera_->get_blue_gain();
        break;
    }
    this->publish_state(val);
  }

  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  void control(float value) override {
    this->publish_state(value);
    switch (this->type_) {
      case NUMBER_TYPE_JPEG_QUALITY:
        this->camera_->set_jpeg_quality_runtime((uint8_t) value);
        break;
      case NUMBER_TYPE_RED_GAIN:
        this->camera_->set_red_gain_runtime(value);
        break;
      case NUMBER_TYPE_GREEN_GAIN:
        this->camera_->set_green_gain_runtime(value);
        break;
      case NUMBER_TYPE_BLUE_GAIN:
        this->camera_->set_blue_gain_runtime(value);
        break;
    }
  }

  P4CSICamera *camera_{nullptr};
  NumberType type_{NUMBER_TYPE_JPEG_QUALITY};
};

using P4CSICameraQualityNumber = P4CSICameraNumber;

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
