# ESP-IDF environment for building ArduPilot for ESP32-S3 only.
# Stands in for modules/esp_idf/export.sh, which demands the RISC-V/ULP/openocd
# tools for every chip. Usage: source esp32s3-env.sh
AP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export IDF_PATH="$AP_ROOT/modules/esp_idf"
export IDF_PYTHON_ENV_PATH="$HOME/.espressif/python_env/idf5.3_py3.10_env"
export PATH="$(echo $HOME/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin):$(echo $HOME/.espressif/tools/ninja/*):$(echo $HOME/.espressif/tools/esp-rom-elfs/*):$IDF_PYTHON_ENV_PATH/bin:$IDF_PATH/tools:$PATH"
export ESP_ROM_ELF_DIR="$(echo $HOME/.espressif/tools/esp-rom-elfs/*)/"
