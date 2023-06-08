# STM32 Lab Project

This repository contains the source code and documentation for my STM32 development project. The project is designed for
learning purposes and is based on the STM32F103ZET6 microcontroller board, which is manufactured by ALIENTEK tech. co.,
LTD. (Zhengdian atom).

![Board](https://raw.githubusercontent.com/WaylonMao/stm32-lab/master/DevelopmentBoard.jpeg)

## Branch: Master

All labs will be updated in sub-branches.

- [1-blink-leds](https://github.com/WaylonMao/stm32-lab/tree/1-blink-leds)
- [2-key-input](https://github.com/WaylonMao/stm32-lab/tree/2-key-input)
- [3-external-interrupt](https://github.com/WaylonMao/stm32-lab/tree/3-external-interrupt)
- [4-clock-init](https://github.com/WaylonMao/stm32-lab/tree/4-clock-init)
- [5-UART-demo](https://github.com/WaylonMao/stm32-lab/tree/5-UART-demo)
- learning......

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

## Date

Update Date: 2023-06-08

## License

This project is licensed under the MIT License. Please see the LICENSE file for more details.