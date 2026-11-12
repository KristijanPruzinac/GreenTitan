# flash_as.sh NAME COMx (Git Bash on Windows): flash the images in out/fw_NAME (fw_var.sh) to the board on COMx with
# PlatformIO's esptool (PLATFORMIO_CORE_DIR, default ~/.platformio)
C=$(cygpath -u "${PLATFORMIO_CORE_DIR:-$USERPROFILE/.platformio}")
cd "$(dirname "$0")/out/fw_$1" && "$C/penv/Scripts/python.exe" "$C/packages/tool-esptoolpy/esptool.py" --chip esp32 --port "$2" --baud 921600 \
  write_flash -z 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin 2>&1 | tail -1
