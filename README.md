# ESPHome ESP32-P4 MIPI CSI Camera Component

An ESPHome external component that adds MIPI CSI camera support for ESP32-P4 boards with the OV02C10 sensor.

> ⚠️ **EXPERIMENTAL — LOOKING FOR A MAINTAINER**
>
> This component was 100% vibe-coded with AI assistance (Kiro/Claude) and has not been
> reviewed by an experienced ESPHome or ESP-IDF developer. It works (frames stream to
> Home Assistant!) but has rough edges: colour accuracy, stability under load, and proper
> ESPHome lifecycle management all need attention.
>
> **If you're an experienced ESPHome/ESP-IDF developer, please fork this and take
> ownership.** The hard discovery work is done — the init sequence, pin mappings, and
> V4L2 pipeline are proven. It just needs someone who knows the framework deeply to
> polish it into a proper component.

## What Works

- ✅ OV02C10 sensor detection and initialization via `esp_video`
- ✅ MIPI CSI 2-lane frame capture at 1288x728
- ✅ Hardware JPEG encoding
- ✅ Streaming to Home Assistant via ESPHome native API
- ✅ Camera entity visible in HA dashboard

## What Needs Work

- 🔧 **Colour accuracy** — ISP bayer pattern may not be correctly tuned (images can appear blue-shifted)
- 🔧 **Frame glitches** — occasional tearing/red lines at frame boundaries
- 🔧 **Watchdog pressure** — JPEG encoding of 1.8MB frames can trigger WDT if interval is too aggressive
- 🔧 **No idle frame management** — should only capture when HA is actively viewing
- 🔧 **No V4L2 controls** — exposure, white balance, gain not exposed
- 🔧 **Coexistence with display** — untested alongside MIPI DSI display (EspControl/LVGL)
- 🔧 **Memory management** — frame buffers in PSRAM could be better managed
- 🔧 **Error recovery** — no handling for sensor disconnect/reconnect
- 🔧 **Multi-resolution support** — currently fixed at sensor default

## Hardware Support

- **SoC:** ESP32-P4
- **Camera Sensor:** OV02C10 (2MP, 1288x728 output via ISP)
- **Interface:** MIPI CSI-2 (2-lane)
- **Tested Board:** Guition JC8012P4A1C_I_W_Y (10.1" panel)

## Quick Start

```yaml
substitutions:
  name: "p4-camera"
  friendly_name: "P4 Camera"

esphome:
  name: ${name}
  friendly_name: ${friendly_name}

esp32:
  variant: esp32p4
  flash_size: 16MB
  cpu_frequency: 360MHZ
  engineering_sample: true
  framework:
    type: esp-idf
    advanced:
      enable_idf_experimental_features: true
      execute_from_psram: true
      disable_vfs_support_termios: false
      disable_vfs_support_select: false
      disable_vfs_support_dir: false

logger:
  hardware_uart: USB_SERIAL_JTAG

psram:
  mode: hex
  speed: 200MHz

esp32_hosted:
  variant: esp32c6
  reset_pin: GPIO54
  cmd_pin: GPIO19
  clk_pin: GPIO18
  d0_pin: GPIO14
  d1_pin: GPIO15
  d2_pin: GPIO16
  d3_pin: GPIO17
  active_high: true

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:

ota:
  - platform: esphome

external_components:
  - source:
      type: local
      path: components
    components: [p4_csi_camera]

# Or from GitHub:
# external_components:
#   - source: github://sullb/esphome-p4-csi-camera
#     components: [p4_csi_camera]

p4_csi_camera:
  name: "Panel Camera"
  resolution: 1928X1092
  data_lanes: 2
  max_framerate: 2 fps
  idle_framerate: 0.1 fps
```

## Important Notes

- **Do NOT include `esp_ldo:` in your YAML** — the `esp_video` component manages the MIPI PHY LDO internally
- **Do NOT include `i2c:` for the camera bus** — `esp_video` manages its own SCCB/I2C for the sensor
- **Requires `engineering_sample: true`** for ESP32-P4 rev1.3 boards (most Guition panels)
- **VFS support must stay enabled** — the video device needs `/dev/video0` filesystem access

## How It Works

```
OV02C10 Sensor → MIPI CSI-2 (2-lane) → ESP32-P4 CSI Controller
    → ISP (debayer/colour) → V4L2 /dev/video0
    → Hardware JPEG Encoder → ESPHome Camera API → Home Assistant
```

The component uses Espressif's `esp_video` component to handle the CSI→ISP pipeline
(the same approach used in Guition's stock firmware), then captures frames via the
standard V4L2 interface and encodes them with the P4's hardware JPEG encoder.

## Acknowledgments

This component would not exist without the work of:

- **[profi-max](https://github.com/profi-max/JC8012P4A1_BSP_ESP32P4)** — Board Support Package for the JC8012P4A1 with working camera demo code and OV02C10 driver. The BSP's local `esp_cam_sensor` with OV02C10 support was essential.
- **[p1ngb4ck](https://github.com/p1ngb4ck/unofficial_guition_esp32p4_repo)** — Unofficial Guition ESP32-P4 schematics and ESPHome examples, particularly for the JC8012P4A1C_I_W_Y model.
- **[jtenniswood / EspControl](https://github.com/jtenniswood/espcontrol)** — The EspControl firmware project whose device configuration and ESP32-P4 board setup served as the reference for pin assignments, build flags, and peripheral init.
- **[katzrydz](https://community.home-assistant.io/t/jc8012p4a1-guition-esp32-p4-esp32-c6/939971)** — HA Community thread with early ESP32-P4 camera/audio exploration that identified the ES8311 codec pins and OV02C10 sensor.
- **[Espressif](https://github.com/espressif/esp-video-components)** — The `esp_video` and `esp_cam_sensor` components that handle the complex CSI→ISP pipeline.
- **Linux kernel OV02C10 driver** (`drivers/media/i2c/ov02c10.c`, Copyright Intel Corporation, GPL-2.0) — Register init sequences referenced during early development.
- **[anton-malakhov/esphome_webcam](https://github.com/anton-malakhov/esphome_webcam)** — USB webcam ESPHome component whose camera API implementation pattern we followed.

## License

MIT — Use freely, attribution appreciated.

## Contributing

Please do! This needs a real developer. Key areas:

1. Fix colour/bayer pattern tuning
2. Proper frame lifecycle management (don't encode during DMA)
3. Add V4L2 control passthrough for exposure/WB
4. Test coexistence with MIPI DSI display
5. Add proper ESPHome entity controls (resolution select, quality slider)
6. Profile and fix the WDT pressure from JPEG encoding
7. Add support for other sensors (SC2336, OV5645 — already in esp_cam_sensor)
