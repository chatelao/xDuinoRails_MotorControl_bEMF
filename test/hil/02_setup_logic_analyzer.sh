#!/bin/bash
# Script 2: Setup Logic Analyzer
# This script provides instructions for downloading and flashing the pico-logic-analyzer firmware.

set -e

echo "--- Logic Analyzer Setup ---"
echo "This part of the process requires manual steps."
echo ""
echo "Step 1: Download the Firmware"
echo "--------------------------------"
echo "The HIL test requires the 'pico-logic-analyzer' firmware."
echo "Since there is no stable, direct download link, you must download it manually."
echo ""
echo "Please download the latest '.uf2' file from the official repository:"
echo "https://github.com/pico-coder/sigrok-pico/releases"
echo ""
echo "If no releases are available, you will need to build the firmware from source."
echo ""
read -p "Press [Enter] after you have downloaded the UF2 file to continue..."
echo ""
echo "Step 2: Flash the Board"
echo "--------------------------"
echo "1. Connect your XIAO RP2040 to your computer while holding the 'Boot' button."
echo "2. The board will appear as a USB drive named 'RPI-RP2'."
echo "3. Drag and drop the downloaded '.uf2' file onto this drive."
echo "4. The board will automatically reboot with the new firmware."
echo ""
echo "Logic analyzer setup is complete."
