#!/usr/bin/env bash
#
# Upload Bootloader Script for UF2 SAMD/SAML Bootloader
#
# This script builds the latest bootloader for sensorwatch_pro and uploads it
# using edbg. It automatically finds the newest build based on file modification time.
#
# Usage: ./upload-bootloader.sh
#

set -e  # Exit on any error

BOARD="sensorwatch_pro"
TARGET="saml22-sw-pro"
BUILD_DIR="build/$BOARD"

echo "Building bootloader for $BOARD..."
make all BOARD=$BOARD

# Find the most recently built bootloader binary
LATEST_BIN=$(ls -t "$BUILD_DIR"/bootloader-$BOARD-*.bin | head -1)

if [ -z "$LATEST_BIN" ]; then
    echo "Error: No bootloader binary found in $BUILD_DIR"
    exit 1
fi

echo "Found latest bootloader: $LATEST_BIN"

# Extract the version part from the binary filename
VERSION=$(basename "$LATEST_BIN" | sed "s/bootloader-$BOARD-//" | sed 's/\.bin$//')
LATEST_ELF="$BUILD_DIR/bootloader-$BOARD-$VERSION.elf"

echo "Programming bootloader with edbg..."
edbg -t "$TARGET" -p -f "$LATEST_BIN"

echo "Resetting target..."
edbg -t "$TARGET" -x 20
sleep 0.2
edbg -t "$TARGET" -x 20

echo "Copying ELF file to uf2-bootloader.elf for debugging..."
cp "$LATEST_ELF" ./uf2-bootloader.elf 

echo "Bootloader upload complete!"
