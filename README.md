# BSLI Spaceport Avionics Flight Software

Flight Software for BSLI Spaceport's custom STM32 student researched and developed flight computer. The software is responsible for telemetry and parachute deployment. 

**This codebase is currently being developed for the 24-F01-001 revision of the flight computer.** 

### System Resource Allocation

| Sensor                  | Peripheral |
| ----------------------- | ---------- |
| ADXL375 Accelerometer   | I2C1       |
| BMI323 IMU (Accel/Gyro) | I2C1       |
| BM1422 Magnetometer     | I2C4       |
| MS5607 Barometer        | I2C4       |

Check out the ADXL375 accelerometer sensor driver at `Core/Src/Sensors/adxl375.h` and `Core/Src/Sensors/adxl375.c` for an example of good flight software code. Strive to follow the example set by that driver.

## Setup

Dependencies:
- Arm GCC
- Make
- probe-rs
- cppcheck
- Python 3.12 or newer

## Building

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

For any questions, please contact:  
**Flight Software Responsible Engineer:**  
Brian Jia  
Sophomore  
Electrical & Computer Engineering  
jia.659@osu.edu   