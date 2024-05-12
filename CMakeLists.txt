#############################################################################################################################
# file:  CMakeLists.txt
# brief: Template "CMakeLists.txt" for building of executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements.
#        For debug build:
#          cmake -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#          make -C Debug VERBOSE=1 -j
#        For release build:
#          cmake -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
#          make -C Release VERBOSE=1 -j
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M0				      "-mcpu=cortex-m0")
set (MCPU_CORTEX_M0PLUS				  "-mcpu=cortex-m0plus")
set (MCPU_CORTEX_M3				      "-mcpu=cortex-m3")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MCPU_CORTEX_M7				      "-mcpu=cortex-m7")
set (MCPU_CORTEX_M33				  "-mcpu=cortex-m33")
set (MCPU_CORTEX_M55				  "-mcpu=cortex-m55")
set (MCPU_CORTEX_M85				  "-mcpu=cortex-m85")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")

set(CMAKE_SIZE_UTIL arm-none-eabi-size)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "motorcontrol")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            "../STM32F446RETX_FLASH.ld")
set (MCPU                     "-mcpu=Cortex-M4")
set (MFPU                 "-mfpu=fpv4-sp-d16")
set (MFLOAT_ABI               "")
set (RUNTIME_LIBRARY          "--specs=nano.specs")
set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")

set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# set(NEWLIB_PATH "/usr/include/newlib/" CACHE PATH "Path to the newlib c library headers")
	
file(GLOB PROJECT_SOURCES "${CMAKE_SOURCE_DIR}/Core/Src/*.c")
list(APPEND PROJECT_SOURCES "${CMAKE_SOURCE_DIR}/Core/Startup/startup_stm32f446retx.s")
file(GLOB_RECURSE STM32_DRIVER_SOURCES "${CMAKE_SOURCE_DIR}/Drivers/*.c")
list(APPEND PROJECT_SOURCES ${STM32_DRIVER_SOURCES})

file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/Core/Inc/*.h")
file(GLOB_RECURSE STM32_DRIVER_HEADER_FILES "${CMAKE_SOURCE_DIR}/Drivers/*.h")
list(APPEND HEADER_FILES ${STM32_DRIVER_HEADER_FILES})

message(${HEADER_FILES})

# Initialize an empty list to hold include directories
set(INCLUDE_DIRS)

# Extract directory paths from each header file
foreach(HEADER ${HEADER_FILES})
    get_filename_component(DIR ${HEADER} PATH)
    list(APPEND INCLUDE_DIRS ${DIR})
endforeach()

# Remove duplicates to only include unique directories
list(REMOVE_DUPLICATES INCLUDE_DIRS)

message(STATUS "Include directories:")
foreach(DIR ${INCLUDE_DIRS})
    message(STATUS "  ${DIR}")
endforeach()

set (PROJECT_DEFINES )

set (CMAKE_SYSTEM_PROCESSOR "arm" CACHE STRING "")
set (CMAKE_SYSTEM_NAME "Generic" CACHE STRING "")

# Skip link step during toolchain validation.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Specify toolchain. NOTE When building from inside STM32CubeIDE the location of the toolchain is resolved by the "MCU Toolchain" project setting (via PATH).  
set(TOOLCHAIN_PREFIX   "arm-none-eabi-")
set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
set(CMAKE_AR           "${TOOLCHAIN_PREFIX}ar")
set(CMAKE_LINKER       "{TOOLCHAIN_PREFIX}ld")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_RANLIB       "${TOOLCHAIN_PREFIX}ranlib")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size")
set(CMAKE_STRIP        "${TOOLCHAIN_PREFIX}ld")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# HAL library for GPIO
add_definitions(-DUSE_HAL_DRIVER -DSTM32F446xx)

############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################

################## PROJECT SETUP ######################################
project(${PROJECT_NAME})
enable_language(ASM)

if (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_EXECUTABLE})
  add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
  add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
elseif (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_STATIC_LIBRARY})
  add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
endif()

add_compile_definitions (${PROJECT_DEFINES})
include_directories (${INCLUDE_DIRS})

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall") # -Werror")
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

# Add a post-build command to generate a binary file from the ELF output
add_custom_command(TARGET motorcontrol POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:motorcontrol> motorcontrol.bin
    COMMAND ls -l motorcontrol.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Converting ELF to BIN and getting size")

add_custom_target(
    bin ALL  # 'ALL' means this target is part of the default build
    DEPENDS ${CMAKE_BINARY_DIR}/motorcontrol.bin  # Ensure the bin is created/updated
)

add_custom_target(
    flash
    COMMAND st-flash write motorcontrol.bin 0x08000000
    COMMAND st-flash reset
    DEPENDS ${CMAKE_BINARY_DIR}/motorcontrol.bin  # Ensure bin is created before flashing
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Flashing the firmware to the device"
)

add_custom_target(info
    COMMAND st-info --probe
    COMMENT "Getting info from the device using st-info")

add_custom_target(
    unlock
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/memory.sh --unlock
    COMMENT "Unlocking flash memory on the device"
)

add_custom_target(
    lock
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/memory.sh --lock
    COMMENT "Locking flash memory on the device"
)

add_custom_target(
    reset
    COMMAND st-flash reset
    COMMENT "Resetting device"
)

add_custom_target(
    debug-server
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/run-debug-server.sh
    COMMENT "Starting debug server"
)

add_custom_target(
    debug
    COMMAND gdb-multiarch $<TARGET_FILE:motorcontrol> -ex "target extended-remote localhost:3333"
    COMMENT "Starting GDB"
)
