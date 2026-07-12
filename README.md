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
- ✅ Hardware JPEG encoding (configurable quality 10–100)
- ✅ Streaming to Home Assistant via ESPHome native API
- ✅ Camera entity visible in HA dashboard
- ✅ Runtime-adjustable JPEG quality (HA number entity)
- ✅ Runtime vertical/horizontal flip controls (HA switch entities, software pixel buffer flip)
- ✅ Software white balance correction (compensates for ISP green Bayer bias)
- ✅ Stable streaming at 2fps with no WDT crashes

## What Needs Work

- 🔧 **Colour accuracy** — software WB gains are static (R×1.3, G×0.9, B×1.3); may need per-environment tuning
- 🔧 **USB serial stability** — USB_SERIAL_JTAG triggers spurious resets under DMA load; use WiFi/OTA logs instead
- 🔧 **Flip performance** — software flip processes ~940K pixels per frame; adds ~5ms on P4 360MHz
- 🔧 **USB serial stability** — USB_SERIAL_JTAG can trigger spurious resets under DMA load (use WiFi logs instead)
- 🔧 **Coexistence with display** — untested alongside MIPI DSI display (EspControl/LVGL)
- 🔧 **Memory management** — frame buffers in PSRAM could be better managed
- 🔧 **Error recovery** — no handling for sensor disconnect/reconnect
- 🔧 **Multi-resolution support** — currently fixed at sensor default (1288x728)

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
  id: csi_cam
  name: "Panel Camera"
  resolution: 1928X1092
  data_lanes: 2
  max_framerate: 2 fps
  jpeg_quality: 40
  idle_framerate: 0.1 fps

# Runtime-adjustable controls (appear as HA entities)
number:
  - platform: p4_csi_camera
    camera_id: csi_cam
    name: "Camera JPEG Quality"

switch:
  - platform: p4_csi_camera
    camera_id: csi_cam
    vertical_flip:
      name: "Camera Vertical Flip"
    horizontal_flip:
      name: "Camera Horizontal Flip"
```

## Home Assistant Entities

When configured, the component exposes the following entities in Home Assistant:

| Entity | Type | Description |
|--------|------|-------------|
| Panel Camera | Camera | Live MJPEG stream |
| Camera JPEG Quality | Number (10–100) | Adjusts JPEG compression quality per-frame |
| Camera Vertical Flip | Switch | Flips image vertically (software pixel buffer swap) |
| Camera Horizontal Flip | Switch | Mirrors image horizontally (software pixel buffer swap) |

All control entities have `entity_category: config` and appear under the device's configuration section.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `resolution` | `1928X1092` | Sensor resolution (currently only native supported) |
| `data_lanes` | `2` | MIPI CSI data lanes (1 or 2) |
| `max_framerate` | `2 fps` | Maximum frame rate when streaming |
| `idle_framerate` | `0.1 fps` | Frame rate when no client is viewing |
| `jpeg_quality` | `40` | Initial JPEG quality (10–100, higher = better quality) |
| `vertical_flip` | `false` | Initial vertical flip state |
| `horizontal_flip` | `false` | Initial horizontal flip state |
| `reset_pin` | — | GPIO for sensor reset (optional) |
| `power_pin` | — | GPIO for sensor power (optional) |

## Important Notes

- **Do NOT include `esp_ldo:` in your YAML** — the `esp_video` component manages the MIPI PHY LDO internally
- **Do NOT include `i2c:` for the camera bus** — `esp_video` manages its own SCCB/I2C for the sensor
- **Requires `engineering_sample: true`** for ESP32-P4 rev1.3 boards (most Guition panels)
- **VFS support must stay enabled** — the video device needs `/dev/video0` filesystem access
- **USB serial can be unstable** — the P4's USB_SERIAL_JTAG may reset under heavy DMA load; prefer WiFi-based log monitoring once connected

## Architecture

```
OV02C10 Sensor → MIPI CSI-2 (2-lane) → ESP32-P4 CSI Controller
    → ISP (debayer) → V4L2 /dev/video0 (RGB565)
    → Software WB correction (R×1.3, G×0.9, B×1.3)
    → Software flip/mirror (if enabled)
    → Hardware JPEG Encoder → ESPHome Camera API → Home Assistant
```

The component uses Espressif's `esp_video` framework for the CSI→ISP pipeline.
The ISP handles Bayer demosaicing but does not apply auto white balance to the
V4L2 capture path, so software WB correction is applied per-frame before JPEG
encoding. Flip/mirror is also done in software since the OV02C10 esp_cam_sensor
driver does not expose V4L2_CID_VFLIP/HFLIP controls.

## Component Structure

```
components/p4_csi_camera/
├── __init__.py                 # Main component config & code generation
├── p4_csi_camera.h             # C++ header (Camera class)
├── p4_csi_camera.cpp           # C++ implementation (V4L2 + JPEG)
├── p4_csi_camera_number.h      # Number entity (JPEG quality)
├── p4_csi_camera_switch.h      # Switch entity (flip controls)
├── idf_component.yml           # IDF dependency declaration
├── number/
│   └── __init__.py             # Number platform registration
└── switch/
    └── __init__.py             # Switch platform registration
```

## Acknowledgments

This component would not exist without the work of:

- **[profi-max](https://github.com/profi-max/JC8012P4A1_BSP_ESP32P4)** — Board Support Package for the JC8012P4A1 with working camera demo code and OV02C10 driver.
- **[p1ngb4ck](https://github.com/p1ngb4ck/unofficial_guition_esp32p4_repo)** — Unofficial Guition ESP32-P4 schematics and ESPHome examples.
- **[jtenniswood / EspControl](https://github.com/jtenniswood/espcontrol)** — Device configuration and ESP32-P4 board setup reference.
- **[katzrydz](https://community.home-assistant.io/t/jc8012p4a1-guition-esp32-p4-esp32-c6/939971)** — HA Community thread with early ESP32-P4 camera/audio exploration.
- **[Espressif](https://github.com/espressif/esp-video-components)** — The `esp_video`, `esp_cam_sensor`, and `esp_ipa` components.
- **Linux kernel OV02C10 driver** (`drivers/media/i2c/ov02c10.c`, Copyright Intel Corporation, GPL-2.0) — Register init sequences referenced during early development.
- **[anton-malakhov/esphome_webcam](https://github.com/anton-malakhov/esphome_webcam)** — USB webcam ESPHome component whose camera API implementation pattern we followed.

## License

MIT — Use freely, attribution appreciated.

## Contributing

Please do! Key areas that need attention:

1. Validate `esp_ipa` AWB/CCM tuning for OV02C10 (green tint fix)
2. Confirm V4L2 flip/mirror control support in current esp_video versions
3. Test coexistence with MIPI DSI display
4. Add support for other sensors (SC2336, OV5645 — already in esp_cam_sensor)
5. Profile memory usage and optimize buffer management
6. Add exposure/gain controls as additional HA entities
