# motorcontrol
Motor control firmware for 3-phase motor using the STM32F4x series of microcontrollers.  

## Origins
This is a fork of the motor control firmware by Ben Katz, which can be found at https://github.com/bgkatz/motorcontrol

## Hardware
Written specifically for these motor controllers
https://github.com/bgkatz/3phase_integrated
but intended to be easy to port.

Most hardware configuration can be done through hw_config.h

## Building
### STCube
This project has used STCubeMX to configure and generate code for the STM32 microcontroller. Because of this you can follow ST's instructions to build the project. The project is configured to use the STM32CubeIDE.  See their website for more information. Flashing can be done with ST's Cube Programmer. 

### CMake
Development has mainly used CMake, any going foreward we will assume using CMake. You must install the following dependencies:
```
sudo apt install cmake gcc-arm-none-eabi make stlink-tools openocd
```

### Cross Compiling Native
To build using cmake, run the following commands:
```
mkdir -p build/arm
cd build/arm
cmake -DBUILD_ARM=On ../..
make
```

#### Flashing
To flash the firmware, you can use openocd or stlink-tools.  Targets have been created to lock and unlock the flash memory.  To flash the firmware, run the following commands:
```
make st-info
make unlock
make flash
make lock
```

### Building Tests
Tests are build using [CppUTest](https://github.com/cpputest/cpputest), and are build natively.  To build the tests, run the following commands:
```
mkdir -p build/native
cd build/native
cmake -DBUILD_TESTING=On ../..
make
make run_tests
```
