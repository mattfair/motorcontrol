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
include(${CMAKE_SOURCE_DIR}/cmake/utils.cmake)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

#find_package(lcov REQUIRED)
#find_package(genhtml REQUIRED)
find_package(nnvg REQUIRED)

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "motorcontrol")
set (PROJECT_TYPE             "exe")

set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


file(GLOB PROJECT_SOURCES "${CMAKE_SOURCE_DIR}/src/Core/Src/*.c")
list(APPEND PROJECT_SOURCES "${CMAKE_SOURCE_DIR}/src/Core/Startup/startup_stm32f446retx.s")
file(GLOB_RECURSE STM32_DRIVER_SOURCES "${CMAKE_SOURCE_DIR}/src/Drivers/*.c")
list(APPEND PROJECT_SOURCES ${STM32_DRIVER_SOURCES})
list(REMOVE_ITEM PROJECT_SOURCES "${CMAKE_SOURCE_DIR}/src/Core/Src/main.c")

file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/src/Core/Inc/*.h")
file(GLOB_RECURSE STM32_DRIVER_HEADER_FILES "${CMAKE_SOURCE_DIR}/src/Drivers/*.h")
list(APPEND HEADER_FILES ${STM32_DRIVER_HEADER_FILES})

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

add_compile_definitions (${PROJECT_DEFINES})

# Define build options
option(BUILD_STM32F4 "Build for HAL F4" ON)
option(BUILD_TESTS "Build native tests" OFF)

# HAL library for GPIO
add_definitions(-DUSE_HAL_DRIVER)

if(BUILD_STM32F4)
  add_definitions(-DSTM32F4 -DSTM32F446xx)
endif()

### DDSL ###
SET(NUNAVUT_SUBMODULES_ROOT "${CMAKE_SOURCE_DIR}/dsdl")
set(NUNAVUT_GENERATED_ROOT ${CMAKE_BINARY_DIR}/generated)
set(NUNAVUT_LANG_STANDARD "c11")
set(NUNAVUT_LANG "c")

list(APPEND INCLUDE_DIRS "${NUNAVUT_GENERATED_ROOT}/${NUNAVUT_LANG}/default")

if(DEFINED ENV{NUNAVUT_LANG})
     message(STATUS "Getting NUNAVUT_LANG from the environment ($ENV{NUNAVUT_LANG})")
     set(NUNAVUT_LANG "$ENV{NUNAVUT_LANG}" CACHE STRING "The Nunavut output language.")
else()
     set(NUNAVUT_LANG "unspecified" CACHE STRING "The Nunavut output language.")
endif()

if(NOT DEFINED NUNAVUT_TARGET_ENDIANNESS)
     set(NUNAVUT_TARGET_ENDIANNESS "any" CACHE STRING "The endianess for the target architecture.")
endif()

if(NOT DEFINED NUNAVUT_SER_ASSERT)
     set(NUNAVUT_SER_ASSERT ON CACHE BOOL "Enable or disable serialization asserts in generated code.")
endif()

if(NOT DEFINED NUNAVUT_SER_FP_DISABLE)
     set(NUNAVUT_SER_FP_DISABLE OFF CACHE BOOL "Enable or disable floating point support in generated support code.")
endif()

if(NOT DEFINED NUNAVUT_OVR_VAR_ARRAY_ENABLE)
     set(NUNAVUT_OVR_VAR_ARRAY_ENABLE OFF CACHE BOOL "Enable or disable override variable array capacity in generated support code.")
endif()

#
# Generate serialization support headers
#
create_dsdl_target(nunavut-support
                   ${NUNAVUT_LANG}
                   "${NUNAVUT_LANG_STANDARD}"
                   ${NUNAVUT_GENERATED_ROOT}/${NUNAVUT_LANG}/default
                   ""
                   OFF
                   ${NUNAVUT_SER_ASSERT}
                   ${NUNAVUT_SER_FP_DISABLE}
                   ${NUNAVUT_OVR_VAR_ARRAY_ENABLE}
                   ON
                   "${NUNAVUT_TARGET_ENDIANNESS}"
                   "only")

#
# Generate types for the Cyphal v1 public_regulated_data_types set.
#
create_dsdl_target(dsdl-regulated
                   ${NUNAVUT_LANG}
                   "${NUNAVUT_LANG_STANDARD}"
                   ${NUNAVUT_GENERATED_ROOT}/${NUNAVUT_LANG}/default
                   ${NUNAVUT_SUBMODULES_ROOT}/public_regulated_data_types/uavcan
                   OFF
                   ${NUNAVUT_SER_ASSERT}
                   ${NUNAVUT_SER_FP_DISABLE}
                   ${NUNAVUT_OVR_VAR_ARRAY_ENABLE}
                   ON
                   "${NUNAVUT_TARGET_ENDIANNESS}"
                   "never")

add_dependencies(dsdl-regulated nunavut-support)

# Generate types for the non-standard regulated identifiers public_regulated_data_types set.
#
create_dsdl_target(dsdl-non-standard
                   ${NUNAVUT_LANG}
                   "${NUNAVUT_LANG_STANDARD}"
                   ${NUNAVUT_GENERATED_ROOT}/${NUNAVUT_LANG}/default
                   ${NUNAVUT_SUBMODULES_ROOT}/public_regulated_data_types/reg
                   OFF
                   ${NUNAVUT_SER_ASSERT}
                   ${NUNAVUT_SER_FP_DISABLE}
                   ${NUNAVUT_OVR_VAR_ARRAY_ENABLE}
                   ON
                   "${NUNAVUT_TARGET_ENDIANNESS}"
                   "never"
                   ${NUNAVUT_SUBMODULES_ROOT}/public_regulated_data_types/uavcan)

add_dependencies(dsdl-non-standard nunavut-support)
add_dependencies(dsdl-non-standard nunavut-regulated)

#
# Generate types for custom mattfair types
#
create_dsdl_target(dsdl-mattfair
                   ${NUNAVUT_LANG}
                   "${NUNAVUT_LANG_STANDARD}"
                   ${NUNAVUT_GENERATED_ROOT}/${NUNAVUT_LANG}/default
                   ${NUNAVUT_SUBMODULES_ROOT}/mattfair
                   OFF
                   ${NUNAVUT_SER_ASSERT}
                   ${NUNAVUT_SER_FP_DISABLE}
                   ${NUNAVUT_OVR_VAR_ARRAY_ENABLE}
                   ON
                   "${NUNAVUT_TARGET_ENDIANNESS}"
                   "never"
                   ${NUNAVUT_SUBMODULES_ROOT}/public_regulated_data_types/uavcan
                   ${NUNAVUT_SUBMODULES_ROOT}/public_regulated_data_types/reg
                  )


add_dependencies(dsdl-mattfair nunavut-regulated)
add_dependencies(dsdl-mattfair nunavut-non-standard)

set(DSDL_LIBRARIES dsdl-regulated dsdl-non-standard dsdl-mattfair nunavut-support)

add_definitions(-DNUNAVUT_ASSERT=assert)

# Forward the revision information to the compiler so that we could expose it at runtime. This is entirely optional.
execute_process(
        COMMAND git rev-parse --short=16 HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE vcs_revision_id
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "vcs_revision_id: ${vcs_revision_id}")
add_definitions(
        -DVERSION_MAJOR=1
        -DVERSION_MINOR=0
        -DVCS_REVISION_ID=0x${vcs_revision_id}ULL
        -DNODE_NAME="com.mattfair.udral.servo"
)

################## PROJECT SETUP ######################################
project(${PROJECT_NAME})

if(BUILD_TESTS)
    include(${CMAKE_SOURCE_DIR}/cmake/utils.cmake)
    apply_flag_set(${CMAKE_SOURCE_DIR}/cmake/compiler_flag_sets/native.cmake "" "" "")

    # Compile at 32-bit
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")

    set(IO_SRC 
      #"${CMAKE_SOURCE_DIR}/src/IO/register.c"
      "${CMAKE_SOURCE_DIR}/src/IO/flash.c"
    )
    list(APPEND INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/src")
    list(APPEND INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/tests/mocks/")

    add_library(io ${IO_SRC})
    target_include_directories(io PRIVATE ${INCLUDE_DIRS})
    add_dependencies(io dsdl-regulated dsdl-non-standard dsdl-mattfair nunavut-support)

    # TODO: work on removing these
    add_compile_options(
      #    -Wno-c++17-attribute-extensions
    #    -Wno-error=float-conversion
    #    -Wno-error=conversion
    #    -Wno-error=unused-parameter
    #    -Wno-error=double-promotion
    #    -Wno-error=float-equal
    #    -Wno-error=builtin-declaration-mismatch
    #    -Wno-error=format
    #    -Wno-error=missing-declarations
        -Wno-error=sign-conversion
    #    -Wno-error=implicit-function-declaration
    #    -Wno-error=comment
    #    -Wno-error=unused-variable
    #    -Wno-error=switch
        -Wno-error=switch-enum
    #    -Wno-error=pedantic
    #    -Wno-error=int-conversion
    #    -Wno-error=sign-compare
    #    -Wno-error=pointer-sign
         -Wno-error=old-style-cast
         -Wno-error=missing-declarations
    )

    #find_package(gtest REQUIRED)
    #add_subdirectory(${GOOGLETEST_SUBMODULE})
    #SET(GTEST_LIBRARIES gtest gmock)

    set(TESTS OFF CACHE BOOL "Switch off CppUTest Test build")
    add_subdirectory(${CMAKE_SOURCE_DIR}/submodules/cpputest)
    set(CPPUTest_COMPILE_FLAGS -Wno-error=old-style-cast -Wno-error=sign-promo)
    target_compile_options(CppUTest PRIVATE ${CPPUTest_COMPILE_FLAGS})
    target_compile_options(CppUTestExt PRIVATE ${CPPUTest_COMPILE_FLAGS})

    set(CPPUTest_LIBRARIES CppUTest CppUTestExt)
    set(CPPUTest_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/submodules/cpputest/include ${CMAKE_SOURCE_DIR}/submodules/cpputest/include/Platforms/Gcc)

    # Native test target
    enable_testing()

    file(GLOB_RECURSE IO_TEST_SOURCES "tests/IO/*_test.cpp")
    list(REMOVE_ITEM IO_TEST_SOURCES "${CMAKE_SOURCE_DIR}/tests/IO/register_test.cpp")

    set(IO_MOCKS_SOURCES 
      "${CMAKE_SOURCE_DIR}/tests/mocks/IO/mock_flash.cpp"
    )

    add_executable(io_test_runner 
      ${IO_TEST_SOURCES}
      ${IO_MOCKS_SOURCES}
      "${CMAKE_SOURCE_DIR}/tests/main.cpp"
    )
    target_include_directories(io_test_runner PRIVATE ${INCLUDE_DIRS} ${CPPUTest_INCLUDE_DIRS})
    target_link_libraries(io_test_runner PRIVATE ${CPPUTest_LIBRARIES} ${DSDL_LIBRARIES} io)
    add_dependencies(io_test_runner ${CPPUTest_LIBRARIES} ${DSDL_LIBRARIES} io)
    target_compile_options(io_test_runner PRIVATE ${CPPUTest_COMPILE_FLAGS})

    add_test(NAME io COMMAND io_test_runner)

    # Custom target to run all tests with ctest --output-on-failure
    add_custom_target(run_tests
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
        DEPENDS io_test_runner
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running tests with verbose output on failure"
    )
else()
    # Cross compile for ARM

    include_directories (${INCLUDE_DIRS})
    enable_language(ASM)
    set(CMAKE_SIZE_UTIL arm-none-eabi-size)

    set (LINKER_SCRIPT            "${CMAKE_SOURCE_DIR}/STM32F446RETX_FLASH.ld")
    set (MCPU                     "-mcpu=Cortex-M4")
    set (MFPU                 "-mfpu=fpv4-sp-d16")
    set (MFLOAT_ABI               "-mfloat-abi=hard")
    set (RUNTIME_LIBRARY          "--specs=nano.specs")
    set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")

    set (CMAKE_EXECUTABLE_SUFFIX ".elf")
    set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
    set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall") # -Werror")
    set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
    set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

    # Set up ARM toolchain
    set(CMAKE_SYSTEM_PROCESSOR "arm" CACHE STRING "")
    set(CMAKE_SYSTEM_NAME "Generic" CACHE STRING "")
    set(TOOLCHAIN_PREFIX "arm-none-eabi-")

    # Skip link step during toolchain validation
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

    # Specify toolchain
    set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
    set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
    set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
    set(CMAKE_AR           "${TOOLCHAIN_PREFIX}ar")
    set(CMAKE_LINKER       "${TOOLCHAIN_PREFIX}ld")
    set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy")
    set(CMAKE_RANLIB       "${TOOLCHAIN_PREFIX}ranlib")
    set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size")
    set(CMAKE_STRIP        "${TOOLCHAIN_PREFIX}ld")

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

    # arm target
    add_executable(arm_motorcontrol ${PROJECT_SOURCES} "${CMAKE_SOURCE_DIR}/src/Core/Src/main.c")
    target_include_directories(arm_motorcontrol PRIVATE ${INCLUDE_DIRS})
    set_target_properties(arm_motorcontrol PROPERTIES
        C_COMPILER arm-none-eabi-gcc
        CXX_COMPILER arm-none-eabi-g++
    )
    target_compile_options(arm_motorcontrol PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall)
    add_dependencies(arm_motorcontrol dsdl-regulated)
    add_dependencies(arm_motorcontrol dsdl-non-standard)
    add_dependencies(arm_motorcontrol dsdl-mattfair)
    add_dependencies(arm_motorcontrol nunavut-support)

    # Add a post-build command to generate a binary file from the ELF output
    add_custom_command(TARGET arm_motorcontrol POST_BUILD
        COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:arm_motorcontrol> arm_motorcontrol.bin
        COMMAND ls -l arm_motorcontrol.bin
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Converting ELF to BIN and getting size")

    add_custom_target(
      bin ALL  # 'ALL' means this target is part of the default build
      DEPENDS ${CMAKE_BINARY_DIR}/arm_motorcontrol.bin  # Ensure the bin is created/updated
    )

    add_custom_target(
        flash
        COMMAND st-flash write arm_motorcontrol.bin 0x08000000
        COMMAND st-flash reset
        DEPENDS ${CMAKE_BINARY_DIR}/arm_motorcontrol.bin  # Ensure bin is created before flashing
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
        COMMAND gdb-multiarch $<TARGET_FILE:arm_motorcontrol> -ex "target extended-remote localhost:3333"
        COMMENT "Starting GDB"
    )
endif()
