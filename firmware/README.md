# Tail Firmware

This directory contains the source code, build files, and libraries for the firmware running on the Tail Tag devices. The firmware is likely written in C/C++ and targets an EFM32 microcontroller.

## Directory Structure

*   **`Makefile`**: The main makefile used to build the firmware (bootloader and main application).
*   **`bootloader.ld`**: Linker script for the bootloader firmware. Defines memory layout and sections.
*   **`Tail.ld`**: Linker script for the main Tail application firmware. Defines memory layout and sections.
*   **`bootloader_src/`**: Source code specific to the bootloader.
*   **`src/`**: Source code for the main Tail application firmware. This likely includes the UWB communication logic, sensor interactions (if any), power management, etc.
*   **`CMSIS/`**: Contains the Cortex Microcontroller Software Interface Standard (CMSIS) core files, providing a vendor-independent hardware abstraction layer for ARM Cortex-M processors (like the EFM32).
*   **`efm32-base/`**: Base support files or libraries specific to the EFM32 platform, potentially provided by Silicon Labs or customized for this project.
*   **`emlib/`**: Silicon Labs' EFM32 peripheral library (emlib), providing drivers and APIs for microcontroller peripherals (GPIO, SPI, Timers, etc.).

## Building

The firmware can likely be built by running `make` within this directory, provided the necessary ARM GCC toolchain is installed and configured correctly. Refer to the main project `README.md` or specific build scripts for detailed instructions if available.
