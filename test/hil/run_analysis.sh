#!/bin/bash
# Script 4: Run HIL Test and Analyze Results
# This script captures PWM data from the DUT and generates a report.

set -e

echo "--- HIL Test Execution and Analysis ---"
echo ""
echo "Step 1: Connect the Logic Analyzer"
echo "-----------------------------------"
echo "Connect the XIAO RP2040 running the 'pico-logic-analyzer' firmware to your computer."
echo ""
read -p "Press [Enter] when the Logic Analyzer is connected..."
echo ""
echo "Step 2: Prepare for Capture"
echo "---------------------------"
echo "The script is about to start capturing data for 10 seconds."
echo "As soon as the capture begins, press the 'Reset' button on the DUT (the other XIAO board)."
echo "This will ensure the test signals are generated while the logic analyzer is running."
echo ""
read -p "Press [Enter] to start the 10-second capture..."
echo ""
echo "Starting capture... RESET THE DUT NOW!"

# Capture 10 seconds of data (1M samples/sec * 10s = 10M samples)
# This assumes the raspberrypi-pico driver is correctly installed and recognized.
sigrok-cli -d raspberrypi-pico --samples 10M -c samplerate=1m -o capture.sr

echo "Capture complete. Saved to capture.sr"
echo ""
echo "Step 3: Analyze Data and Generate Report"
echo "-----------------------------------------"
echo "Analyzing the captured data and generating a report..."

# Analyze the capture file and decode the PWM and Pulse signals, saving the output to a text file.
sigrok-cli -i capture.sr -A pwm,pulse > report.txt

echo ""
echo "Analysis complete. Report saved to report.txt"
echo "You can view the report by opening the 'report.txt' file."
echo "For a graphical view, open 'capture.sr' in the PulseView application."
echo ""
echo "HIL test finished."
