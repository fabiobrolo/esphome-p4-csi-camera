#pragma once

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "p4_csi_camera.h"

namespace esphome {
namespace p4_csi_camera {

enum SwitchMode : uint8_t {
  SWITCH_MODE_VERTICAL_FLIP = 0,
  SWITCH_MODE_HORIZONTAL_FLIP = 1,
  SWITCH_MODE_COLOR_CORRECTION = 2,
  SWITCH_MODE_AUTO_WHITE_BALANCE = 3,
};

class P4CSICameraSwitch : public switch_::Switch, public Component {
 public:
  void set_camera(P4CSICamera *camera) { this->camera_ = camera; }
  void set_switch_mode(uint8_t mode) { this->switch_mode_ = (SwitchMode) mode; }

  void setup() override {
    bool state = false;
    switch (this->switch_mode_) {
      case SWITCH_MODE_VERTICAL_FLIP:
        state = this->camera_->get_vertical_flip();
        break;
      case SWITCH_MODE_HORIZONTAL_FLIP:
        state = this->camera_->get_horizontal_flip();
        break;
      case SWITCH_MODE_COLOR_CORRECTION:
        state = this->camera_->get_color_correction();
        break;
      case SWITCH_MODE_AUTO_WHITE_BALANCE:
        state = this->camera_->get_auto_white_balance();
        break;
    }
    this->publish_state(state);
  }

  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  void write_state(bool state) override {
    switch (this->switch_mode_) {
      case SWITCH_MODE_VERTICAL_FLIP:
        this->camera_->set_vertical_flip_runtime(state);
        break;
      case SWITCH_MODE_HORIZONTAL_FLIP:
        this->camera_->set_horizontal_flip_runtime(state);
        break;
      case SWITCH_MODE_COLOR_CORRECTION:
        this->camera_->set_color_correction_runtime(state);
        break;
      case SWITCH_MODE_AUTO_WHITE_BALANCE:
        this->camera_->set_auto_white_balance_runtime(state);
        break;
    }
    this->publish_state(state);
  }

  P4CSICamera *camera_{nullptr};
  SwitchMode switch_mode_{SWITCH_MODE_VERTICAL_FLIP};
};

using P4CSICameraFlipSwitch = P4CSICameraSwitch;

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
