#!/bin/bash
# Script 1: Install Dependencies
# This script installs the necessary tools for the HIL test.

set -e

echo "Updating package lists..."
sudo apt-get update

echo "Installing sigrok-cli and git..."
sudo apt-get install -y sigrok-cli git

echo "Installation complete."
