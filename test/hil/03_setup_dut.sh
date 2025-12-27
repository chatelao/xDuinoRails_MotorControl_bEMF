#!/bin/bash
# Script 3: Setup Device Under Test (DUT)
# This script compiles and uploads the debugPWM firmware to the DUT.

set -e

echo "--- Device Under Test (DUT) Setup ---"
echo "This script will build and upload the firmware for the DUT."
echo ""
echo "Step 1: Connect the DUT"
echo "--------------------------"
echo "Connect the second XIAO RP2040 board (the DUT) to your computer."
echo "Ensure it is in normal boot mode (do NOT hold the 'Boot' button)."
echo ""
read -p "Press [Enter] when the DUT is connected..."
echo ""
echo "Step 2: Build and Upload Firmware"
echo "-----------------------------------"
echo "PlatformIO will now compile and upload the 'debugPWM' example."
echo "This may take a few moments..."
echo ""

# Set the source directory to the debugPWM example and run PlatformIO
export PIOPROJECT_SRC_DIR=examples/debugPWM
python -m platformio run -e seeed_xiao_rp2040 -t upload

echo ""
echo "Firmware upload complete."
echo "DUT setup is complete."
