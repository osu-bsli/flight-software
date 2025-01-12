# BSLI IREC Avionics Flight Software

Flight Software for the BSLI IREC (International Rocket Engineering Competition) student researched and developed flight computer.
The flight computer connects to a radio board for telemetry and has onboard power electronics for ejection charge detonation (parachute deployment). 

- [BSLI IREC Avionics Flight Software](#bsli-irec-avionics-flight-software)
  - [Hardware Specifications](#hardware-specifications)
  - [Code Examples](#code-examples)
  - [Required Dependencies](#required-dependencies)
  - [Compiling and Uploading to Flight Computer](#compiling-and-uploading-to-flight-computer)
- [Contact](#contact)

The Flight Software aims to be an educational tool by providing high quality examples of sensor drivers, FreeRTOS task management, and embedded development best practices.

## Hardware Specifications
**This codebase is currently being developed for the 24-F01-001 revision of the flight computer.** 

| Device Type               | Description             | Connected to microcontroller via |
| ------------------------- | ----------------------- | -------------------------------- |
| Microcontroller           | STM32H753IIT6 @ 480 MHz | *N/A*                            |
| Accelerometer             | ADXL375                 | I2C1                             |
| Inertial Measurement Unit | BMI323                  | I2C1                             |
| Magnetometer              | BM1422                  | I2C4                             |
| Barometer                 | MS5607                  | I2C4                             |
| GPS                       | TESEO-LIV3F             | I2C3                             |

## Code Examples
* ADXL375 accelerometer driver `Core/Inc/Sensors/adxl375.h`, `Core/Src/Sensors/adxl375.c`
* FreeRTOS task creation example: `Core/Inc/Tasks/task_blinky.h`, `Core/Src/Tasks/task_blinky.c`
  
Strive to follow the standard set by these examples.

## Required Dependencies

Most of these dependencies are obtainable through **MSYS2 on Windows** or **Homebrew on Mac**.

- **arm-none-eabi GNU C/C++ Toolchain** 
  - MSYS2 install command: `pacman -S mingw-w64-x86_64-arm-none-eabi-toolchain`
  - Homebrew install command: `brew install --cask gcc-arm-embedded`
- **cppcheck** for checking code for subtle errors
  - MSYS2 install command: `pacman -S mingw-w64-x86_64-cppcheck`
  - Homebrew install command: `brew install cppcheck`
- **probe-rs** for uploading the software to the flight computer
  - MSYS2 install command: `cargo install probe-rs-tools`
  - Homebrew install command: `brew install probe-rs-tools`
- **STM32CubeMX** for generating code for the flight software
  - Get it from the STMicroelectronics website: https://www.st.com/en/development-tools/stm32cubemx.html
- **Python 3.11 or later**
  - Get it from https://python.org 

**Required preliminary actions:**  
Run these inside the flight-software repo directory.  
Install dependencies for MAVLink (aerial vehicle communication protocol):
```
python -m pip install -r External/mavlink/pymavlink/requirements.txt
```

## Compiling and Uploading to Flight Computer

Build the flight software:
```
make -j20
```

`-j20` tells Make to use 20 threads. Change if needed.

Build, flash, and run using probe-rs:
```
make -j20 run
```

Check code using cppcheck:
```
make check
```

# Contact

For any questions, please contact:  
**Flight Software Responsible Engineer:**  
Brian Jia  
Sophomore  
Electrical & Computer Engineering  
jia.659@osu.edu   