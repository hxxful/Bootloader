Overview
========
The flexio_i2s_interrupt example shows how to use flexio_i2s driver with interrupt:

In this example, flexio acts as I2S module to record data from line-in line and playbacks the recorded data at the same time using interrupt.

Toolchain supported
===================
- IAR embedded Workbench 8.11.3
- Keil MDK 5.23
- GCC ARM Embedded 6-2017-q2
- MCUXpresso10.1.0

Hardware requirements
=====================
- Mini/micro USB cable
- EVKB-IMXRT1050 board
- Headphone
- Personal Computer

Board settings
==============
- J12, headphone connected
- remove R98,R99,R100,R101 and connect FLEXIO pins to the pad close to U13 as follows:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Pin Name   Board Location     Pin Name    Board Location
RX_DATA    SW5-1              RX_DATA     U13-16
TX_DATA    SW5-2              TX_DATA     U13-14
SYNC       SW5-3              SYNC        U13-13
BCLK       SW5-4              BCLK        U13-12
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Prepare the Demo
================
1.  Connect a USB cable between the host PC and the OpenSDA USB port on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the example.

Running the demo
================
When the demo runs successfully, the log would be seen on the OpenSDA terminal like as below.

~~~~~~~~~~~~~~~~~~~~~
FLEXIO_I2S interrupt example started!

FLEXIO_I2S interrupt example finished!
~~~~~~~~~~~~~~~~~~~~~
Customization options
=====================

