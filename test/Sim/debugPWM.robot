*** Settings ***
Documentation       A test suite for running a simple PWM test in Renode.
Suite Setup         Setup
Suite Teardown      Teardown
Library             renode_api_ws.so

*** Variables ***
${FIRMWARE_ELF_PATH}    ../../.pio/build/seeed_xiao_rp2040/firmware.elf

*** Test Cases ***
Should Toggle Pulse Pin
    [Documentation]    Runs the simulation and checks if the pulse pin is toggled.
    [Timeout]    10s
    ${machine}=    Get Machine Id
    Load ELF    ${machine}    ${FIRMWARE_ELF_PATH}
    Start Emulation
    Wait For Log    (sysbus.gpio) Setting GPIO0 to High    10
    Log    Pulse pin toggled successfully.

*** Keywords ***
Setup
    # This keyword is run once before the test suite starts.
    # It can be used for any setup that is needed for the tests.
    Log    Starting Renode simulation for debugPWM.

Teardown
    # This keyword is run once after the test suite has finished.
    # It can be used for any cleanup that is needed.
    Log    Renode simulation finished.
    Stop Emulation
