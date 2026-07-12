#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "p4_csi_camera.h"

namespace esphome {
namespace p4_csi_camera {

class P4CSICameraQualityNumber : public number::Number, public Component {
 public:
  void set_camera(P4CSICamera *camera) { this->camera_ = camera; }

  void setup() override {
    float val = this->camera_->get_jpeg_quality();
    this->publish_state(val);
  }

  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  void control(float value) override {
    this->publish_state(value);
    this->camera_->set_jpeg_quality_runtime((uint8_t) value);
  }

  P4CSICamera *camera_{nullptr};
};

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
