#!/bin/bash
# Build PetaLinux project and package BOOT.BIN.
# Run inside the Docker container with the repo mounted at /work.
#
# Usage: build.sh [--bitstream <path-to-bit>]
set -euo pipefail

PROJECT_DIR="/work/petalinux/project/vi_petalinux"
OUTPUT_DIR="/work/petalinux/output"
BITSTREAM=""

while [ $# -gt 0 ]; do
    case "$1" in
        --bitstream) BITSTREAM="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

if [ ! -d "${PROJECT_DIR}" ]; then
    echo "ERROR: Project not found at ${PROJECT_DIR}. Run create_project.sh first."
    exit 1
fi

cd "${PROJECT_DIR}"

echo "==> Building PetaLinux project..."
petalinux-build

echo "==> Packaging BOOT.BIN..."
BOOT_ARGS=(
    --boot
    --fsbl images/linux/zynqmp_fsbl.elf
    --u-boot images/linux/u-boot.elf
    --pmufw images/linux/pmufw.elf
    --force
)
if [ -n "${BITSTREAM}" ]; then
    BOOT_ARGS+=(--fpga "${BITSTREAM}")
fi
petalinux-package "${BOOT_ARGS[@]}"

echo "==> Copying artifacts to ${OUTPUT_DIR}..."
mkdir -p "${OUTPUT_DIR}"
cp images/linux/BOOT.BIN    "${OUTPUT_DIR}/"
cp images/linux/image.ub    "${OUTPUT_DIR}/"
cp images/linux/boot.scr    "${OUTPUT_DIR}/"
cp images/linux/rootfs.tar.gz "${OUTPUT_DIR}/" 2>/dev/null || true

echo "==> Done. Artifacts in ${OUTPUT_DIR}/"
ls -lh "${OUTPUT_DIR}/"
