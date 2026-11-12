# fw_var.sh NAME N 'SED' (WSL): a firmware variant for a parallel HIL board: the checkout's src with a sed on
# src/definitions.h, micro-ROS domain 40+N, built in ~/gt_NAME; images go to out/fw_NAME (flash with flash_as.sh)
source "$(dirname "$0")/common.sh"; N=$2; B=$HOME/gt_$1
rm -rf "$B" && mkdir -p "$B" && cd "$REPO" && cp -r src lib platformio.ini "$B/" && cp microros.meta "$B/" 2>/dev/null
cd "$B" && sed -i "$3" src/definitions.h; diff "$REPO/src/definitions.h" src/definitions.h
export PLATFORMIO_WORKSPACE_DIR=$HOME/.cache/greentitan_pio PLATFORMIO_BUILD_DIR=$HOME/.cache/greentitan_pio/build_i$N PLATFORMIO_BUILD_FLAGS="-DMICRO_ROS_DOMAIN_ID=$((40+N))"
PIO=$(command -v pio || echo "$HOME/.local/pio-venv/bin/pio")
"$PIO" run -e esp32dev_hil 2>&1 | grep -E " error|SUCCESS|FAILED"
O=$PLATFORMIO_BUILD_DIR/esp32dev_hil; F=$OUT/fw_$1; mkdir -p "$F"
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin "$O/bootloader.bin" "$O/partitions.bin" "$O/firmware.bin" "$F/"
