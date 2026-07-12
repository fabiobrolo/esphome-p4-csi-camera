#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "p4_csi_camera.h"

namespace esphome {
namespace p4_csi_camera {

enum FlipMode : uint8_t {
  FLIP_MODE_VERTICAL = 0,
  FLIP_MODE_HORIZONTAL = 1,
};

class P4CSICameraFlipSwitch : public switch_::Switch, public Component {
 public:
  void set_camera(P4CSICamera *camera) { this->camera_ = camera; }
  void set_flip_mode(uint8_t mode) { this->flip_mode_ = (FlipMode) mode; }

  void setup() override {
    bool state;
    if (this->flip_mode_ == FLIP_MODE_VERTICAL) {
      state = this->camera_->get_vertical_flip();
    } else {
      state = this->camera_->get_horizontal_flip();
    }
    this->publish_state(state);
  }

  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  void write_state(bool state) override {
    if (this->flip_mode_ == FLIP_MODE_VERTICAL) {
      this->camera_->set_vertical_flip_runtime(state);
    } else {
      this->camera_->set_horizontal_flip_runtime(state);
    }
    this->publish_state(state);
  }

  P4CSICamera *camera_{nullptr};
  FlipMode flip_mode_{FLIP_MODE_VERTICAL};
};

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
