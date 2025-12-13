*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Variables ***
${SCRIPT}       ${CURDIR}/simulation.resc
${UART}         sysbus.uart0

*** Test Cases ***
Should Boot And Print Message
    [Documentation]             Verifies that the firmware boots and prints the welcome message.
    Execute Command             include @${SCRIPT}
    Create Terminal Tester      ${UART}
    Start Emulation
    Wait For Line On Uart       Sine Wave Motor Speed Control Example  timeout=10

    # Common Pattern: Verify Main Loop Activity
    # We wait for the "PWM: " string which is printed repeatedly in the loop.
    # This confirms the firmware didn't crash after setup.
    Wait For Line On Uart       PWM:   timeout=5
