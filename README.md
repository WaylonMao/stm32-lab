# STM32 Lab Project

This repository contains the source code and documentation for my STM32 development project. The project is designed for
learning purposes and is based on the STM32F103ZET6 microcontroller board, which is manufactured by ALIENTEK tech. co.,
LTD. (Zhengdian atom).

## Branch: 3-external-interrupt

There are two LEDs, LED_0(PB5) and LED_1(PE5), both negative poles are connected to GPIO pins, and their positive poles
are pulled
up to 3.3V. And 3 keys on KEY_UP(PA0), KEY_1(PE3), KEY_0(PE4). The KEY_UP is connected to high level (3.3 V),
KEY_0 and KEY_1 are connected to low level (GND).

This project is designed to learn how to use external interrupt. LED_0 and LED_1 are blinking in main loop.
After KEY_0 is pressed, the LED on PB5 will be toggled. This is implemented by external interrupt.

## Learning:

- GPIO Input
- GPIO Output
- External Interrupt
- NVIC Priority

## Date and Version

Date: 2023-05-30

Version: 0.1

## Tools and Dependencies

To build and program the project, the following tools and dependencies are required:

- CMake
- arm-none-eabi-gcc
- openOCD
- Programmer

The integrated development environment (IDE) used for this project is CLion. The debugger and programmer employed is
ST-Link V2.

## File Structure

- `Drivers`: contains HAL library and CMSIS library.
- `Inc`: contains header files.
- `Src`: contains source files.
- `startup`: contains startup files.
- `CMakeLists.txt`: CMake configuration file.
- `STM32F103.svd`: STM32F103ZET6 SVD file for debugging.
- `stm32f103zet6.cfg`: openOCD configuration file.

## Getting Started

To get started with this project, follow the steps below:

1. Install arm-none-eabi-gcc, openOCD.
2. Set up the environment variables for the tools and dependencies.
3. Copy stm32f103zet6.cfg to the openOCD/scripts/board directory.
4. Open the project in CLion. And select stm32f103zet6.cfg as the openOCD configuration file.
5. Build the project. Make sure the build is successful.
6. Connect the programmer to the board and the computer.
7. Run the project, and the program will be downloaded to the board.

## Author

[Weilong Mao](https://github.com/WaylonMao)

## License

This project is licensed under the MIT License. Please see the LICENSE file for more details.