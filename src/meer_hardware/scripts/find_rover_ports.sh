#!/usr/bin/env bash
# Lists every USB serial / camera device with the physical USB port (KERNELS)
# you need for src/meer_hardware/udev/99-meer-rover.rules.
# Tip: unplug one device, run again, and see which line disappeared.
for dev in /dev/ttyUSB* /dev/ttyACM* /dev/video*; do
  [ -e "$dev" ] || continue
  kernels=$(udevadm info -a -n "$dev" 2>/dev/null | grep -m1 -oE 'KERNELS=="[0-9]+-[0-9.]+"' | cut -d'"' -f2)
  model=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -E '^ID_MODEL=' | cut -d= -f2)
  serial=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -E '^ID_SERIAL_SHORT=' | cut -d= -f2)
  printf '%-14s KERNELS=%-8s model=%-30s serial=%s\n' "$dev" "$kernels" "$model" "$serial"
done
echo
echo "Current rover symlinks:"
ls -l /dev/rover_motor /dev/rover_encoder /dev/rplidar /dev/rover_camera 2>/dev/null || echo "  (none yet - install the udev rules)"
