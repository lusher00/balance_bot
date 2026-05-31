#!/bin/bash
# patch_imu_yup.sh
# Updates IMU to Y-up orientation (Y points up, -Z forward, pitch around X).
# Adds pitch_axis (0=X, 1=Y, 2=Z) to imu_offsets_t, loaded from
# /etc/balance_bot_imu.conf. Change pitch_axis without recompiling.
#
# Usage: bash patch_imu_yup.sh [path/to/balance_bot/]
# Default: current directory

set -e
ROOT="${1:-.}"
H="$ROOT/include/balance_bot.h"
C="$ROOT/src/imu_config.c"
M="$ROOT/src/mpu_dmp.c"

for f in "$H" "$C" "$M"; do
    [ -f "$f" ] || { echo "ERROR: $f not found"; exit 1; }
done

echo "Patching $H ..."
python3 - "$H" << 'PYEOF'
import sys
path = sys.argv[1]
src = open(path).read()

old = (
    "typedef struct\n"
    "{\n"
    "    float pitch_offset;     // angle when upright (degrees)\n"
    "    float yaw_offset;       // yaw at heading zero (degrees)\n"
    "    float pitch_dot_offset; // gyro Y bias (deg/s) — corrects constant drift\n"
    "} imu_offsets_t;"
)
new = (
    "typedef struct\n"
    "{\n"
    "    float pitch_offset;     // angle when upright (degrees)\n"
    "    float yaw_offset;       // yaw at heading zero (degrees)\n"
    "    float pitch_dot_offset; // gyro Y bias (deg/s) — corrects constant drift\n"
    "    int   pitch_axis;       // gravity axis for pitch: 0=X  1=Y(default)  2=Z\n"
    "} imu_offsets_t;"
)
if old not in src:
    print("ERROR: imu_offsets_t block not found — already patched?")
    sys.exit(1)
open(path, 'w').write(src.replace(old, new))
print("  imu_offsets_t updated")
PYEOF

echo "Patching $C ..."
python3 - "$C" << 'PYEOF'
import sys
path = sys.argv[1]
src = open(path).read()
errors = []

# 1. Global init
old = "imu_offsets_t g_imu_offsets = {.pitch_offset = 0.0f, .yaw_offset = 0.0f};"
new = "imu_offsets_t g_imu_offsets = {.pitch_offset = 0.0f, .yaw_offset = 0.0f, .pitch_axis = 1};"
if old not in src: errors.append("global init")
else: src = src.replace(old, new)

# 2. Load — add pitch_axis after yaw_offset
old = (
    '        if (strcmp(key, "yaw_offset") == 0)\n'
    '            offsets->yaw_offset = val;\n'
    '    }\n'
    '    fclose(f);'
)
new = (
    '        if (strcmp(key, "yaw_offset") == 0)\n'
    '            offsets->yaw_offset = val;\n'
    '        if (strcmp(key, "pitch_axis") == 0)\n'
    '            offsets->pitch_axis = (int)val;\n'
    '    }\n'
    '    if (offsets->pitch_axis < 0 || offsets->pitch_axis > 2)\n'
    '        offsets->pitch_axis = 1;\n'
    '    fclose(f);'
)
if old not in src: errors.append("load block")
else: src = src.replace(old, new)

# 3. Save — add pitch_axis after yaw_offset save
old = '    fprintf(f, "yaw_offset   %.4f\\n", offsets->yaw_offset);'
new = ('    fprintf(f, "yaw_offset   %.4f\\n", offsets->yaw_offset);\n'
       '    fprintf(f, "pitch_axis   %d\\n",   offsets->pitch_axis);')
if old not in src: errors.append("save line")
else: src = src.replace(old, new)

# 4. imu_offsets_calibrate — update gravity extraction to match axis
old = (
    '    float gx = -(2.0f * (qx * qz - qw * qy));\n'
    '    float gz = qw * qw - qx * qx - qy * qy + qz * qz;\n'
    '\n'
    '    offsets->pitch_offset = atan2f(gz, gx) * RAD_TO_DEG;'
)
new = (
    '    // Y-up: gravity = [0,-1,0] upright, pitch around X axis\n'
    '    float gy = 2.0f * (qy * qz + qw * qx);\n'
    '    float gz = qw * qw - qx * qx - qy * qy + qz * qz;\n'
    '\n'
    '    offsets->pitch_offset = atan2f(-gz, -gy) * RAD_TO_DEG;'
)
if old not in src: errors.append("calibrate gravity block")
else: src = src.replace(old, new)

# 5. imu_apply_transform — replace gravity block and gyro axis
old = (
    '    // Gravity vector in body frame.\n'
    '    // BBB +X points UP when upright, so negate gx to get gravity direction.\n'
    '    float gx = -(2.0f * (qx * qz - qw * qy));\n'
    '    float gz = qw * qw - qx * qx - qy * qy + qz * qz;\n'
    '\n'
    '    // atan2(gz, gx): 0° upright, + forward, - backward. Full ±180° range.\n'
    '    float pitch_deg = atan2f(gz, gx) * RAD_TO_DEG;\n'
    '\n'
    '    out->pitch = -(pitch_deg - offsets->pitch_offset);\n'
    '    out->pitch_dot = -raw->gyro[1] * RAD_TO_DEG;'
)
new = (
    '    // Gravity vector — axis selected by offsets->pitch_axis.\n'
    '    // 0=X  1=Y (default, BBB Y-up -Z forward)  2=Z\n'
    '    float ga, gb;\n'
    '    int gyro_idx;\n'
    '    if (offsets->pitch_axis == 0) {\n'
    '        // Pitch around X — gravity in Y/Z plane\n'
    '        ga = -(2.0f * (qx * qz - qw * qy));\n'
    '        gb = qw * qw - qx * qx - qy * qy + qz * qz;\n'
    '        gyro_idx = 0;\n'
    '    } else if (offsets->pitch_axis == 2) {\n'
    '        // Pitch around Z — gravity in X/Y plane\n'
    '        ga = 2.0f * (qx * qy + qw * qz);\n'
    '        gb = qw * qw + qx * qx - qy * qy - qz * qz;\n'
    '        gyro_idx = 2;\n'
    '    } else {\n'
    '        // Pitch around Y (BBB Y-up, -Z forward)\n'
    '        ga = 2.0f * (qy * qz + qw * qx);\n'
    '        gb = qw * qw - qx * qx - qy * qy + qz * qz;\n'
    '        gyro_idx = 1;\n'
    '    }\n'
    '\n'
    '    // atan2(-gb, -ga): 0° upright, + forward tilt, - backward tilt.\n'
    '    float pitch_deg = atan2f(-gb, -ga) * RAD_TO_DEG;\n'
    '\n'
    '    out->pitch = -(pitch_deg - offsets->pitch_offset);\n'
    '    out->pitch_dot = -raw->gyro[gyro_idx] * RAD_TO_DEG;'
)
if old not in src: errors.append("imu_apply_transform gravity block")
else: src = src.replace(old, new)

if errors:
    print("ERROR: could not find: %s" % ', '.join(errors))
    print("Already patched, or source has changed.")
    sys.exit(1)

open(path, 'w').write(src)
print("  imu_config.c updated")
PYEOF

echo "Patching $M ..."
python3 - "$M" << 'PYEOF'
import sys
path = sys.argv[1]
src = open(path).read()

# Change orientation from 266 (X_DOWN) to 136 (Y_UP)
# 136 = 0x88, the standard Y-up orientation value
old = '    if (dmp_set_orientation(266) < 0)'
new = '    if (dmp_set_orientation(136) < 0)  /* 136 = Y_UP: +Y up, -Z forward */'
if old not in src:
    print("ERROR: orientation(266) not found — already patched?")
    sys.exit(1)
open(path, 'w').write(src.replace(old, new))
print("  mpu_dmp.c orientation updated: 266 (X_DOWN) -> 136 (Y_UP)")
PYEOF

echo ""
echo "Done. After rebuilding, update /etc/balance_bot_imu.conf:"
echo "  pitch_axis 1   # Y axis (Y-up default)"
echo "Then run zero_imu with the bot upright to recalibrate pitch_offset."
