// Compatibility defines for the OV02C10 sensor driver
// Replaces Kconfig-generated values

#pragma once

// Config options that would normally come from Kconfig
#define CONFIG_CAMERA_OV02C10 1
#define CONFIG_CAMERA_OV02C10_AUTO_DETECT 1
#define CONFIG_CAMERA_OV02C10_AUTO_DETECT_MIPI_INTERFACE_SENSOR 1
#define CONFIG_CAMERA_OV02C10_MAX_SUPPORT 1
#define CONFIG_CAMERA_OV02C10_CSI_LINESYNC_ENABLE 0
#define CONFIG_CAMERA_OV02C10_MIPI_IF_FORMAT_INDEX_DAFAULT 0

// Register definitions that the driver expects from the header
#ifndef OV02C10_REG_END
#define OV02C10_REG_END 0xFFFF
#endif

#ifndef OV02C10_REG_DELAY
#define OV02C10_REG_DELAY 0xFFFE
#endif

#ifndef OV02C10_REG_SENSOR_ID_H
#define OV02C10_REG_SENSOR_ID_H 0x300A
#endif

#ifndef OV02C10_REG_SENSOR_ID_L
#define OV02C10_REG_SENSOR_ID_L 0x300B
#endif
