// This header declares a generated binary firmware blob, not original
// source code -- it is not covered by balance_bot's PolyForm Noncommercial
// license (see LICENSE in this project).
//
// The data it declares is the InvenSense MPU DMP ("digital motion processor")
// firmware image, extracted via the BeagleBoard/StrawsonDesign
// librobotcontrol project's MotionApps support. InvenSense's original
// header (see include/dmpKey.h, include/dmpmap.h in this project) reads:
//   Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
// librobotcontrol's exact redistribution terms for this generated
// artifact were not independently verified when this note was written;
// consult https://github.com/beagleboard/librobotcontrol before
// redistributing this file on its own, outside this project.
/**
 * @file dmp_firmware.h
 * @brief MPU-9250 DMP firmware binary (MotionApps v6.12).
 *
 * The DMP firmware binary must be extracted from a librobotcontrol
 * installation and compiled into this project.  On your BBB run:
 *
 *   python3 scripts/extract_dmp_firmware.py
 *
 * This generates src/dmp_firmware.c which defines:
 *   const uint8_t  dmp_firmware[];
 *   const size_t   dmp_firmware_size;
 *
 * Alternatively, if librobotcontrol is installed, the firmware lives at:
 *   /usr/lib/librobotcontrol.so  (embedded as a data section)
 *
 * The extraction script reads it with objcopy.  See scripts/ for details.
 *
 * Until the firmware is extracted, the robot will fall back to a
 * "no DMP" mode that reads raw gyro + accel and integrates pitch manually.
 */

#ifndef DMP_FIRMWARE_H
#define DMP_FIRMWARE_H

#include <stdint.h>
#include <stddef.h>

extern const uint8_t  dmp_firmware[];
extern const size_t   dmp_firmware_size;

#endif /* DMP_FIRMWARE_H */
