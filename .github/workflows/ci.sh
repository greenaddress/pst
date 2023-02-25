#!/bin/bash
set -eo pipefail

KCONFIG_FILE=main/Kconfig.projbuild

${IDF_PATH}/tools/ci/check_kconfigs.py ${KCONFIG_FILE} || true
mv ${KCONFIG_FILE}.new ${KCONFIG_FILE} &> /dev/null || true

if command -v clang-format &> /dev/null
then
    clang-format -i main/psx_serial_tunnel.c
fi

git diff --exit-code

idf.py build size size-components size-files
