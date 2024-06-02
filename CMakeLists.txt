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
add_definitions(
  -DUSE_HAL_DRIVER
)

if(BUILD_STM32F4)
  add_definitions(-DSTM32F4 -DSTM32F446xx)
endif()

SET(GENERATED_ROOT ${CMAKE_BINARY_DIR}/generated)

### DDSL ###
SET(NUNAVUT_SUBMODULES_ROOT "${CMAKE_SOURCE_DIR}/dsdl")
set(NUNAVUT_GENERATED_ROOT ${GENERATED_ROOT}/dsdl)
set(NUNAVUT_LANG_STANDARD "c11")
set(NUNAVUT_LANG "c")

list(APPEND INCLUDE_DIRS "${NUNAVUT_GENERATED_ROOT}/${NUNAVUT_LANG}/default")
list(APPEND INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/src")

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
    apply_flag_set(${CMAKE_SOURCE_DIR}/cmake/compiler_flag_sets/native_w_cov.cmake "" "" "")

    # Compile at 32-bit
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")

    set(IO_SRC 
      #"${CMAKE_SOURCE_DIR}/src/IO/register.c"
      "${CMAKE_SOURCE_DIR}/src/IO/flash.c"
    )

    set(VERIFICATIONS_BINARY_DIR "${CMAKE_BINARY_DIR}/verifications")
    file(MAKE_DIRECTORY ${VERIFICATIONS_BINARY_DIR})
    set(UNITY_EXTRA_COMPILE_DEFS
      UNITY_OUTPUT_COLOR
      UNITY_POINTER_WIDTH=32
    )
    find_package(unity REQUIRED)
    find_package(cmock REQUIRED)
    find_package(lcov REQUIRED)
    find_package(genhtml REQUIRED)

    include(${CMAKE_SOURCE_DIR}/cmake/testing/unity.cmake)
    include(${CMAKE_SOURCE_DIR}/cmake/testing/cmock.cmake)

    generate_cmock_config("Stm32F4xx_HAL_Driver" "mock_" "stm32f4xx_hal.h;stm32f4xx_hal_def.h;stm32f4xx_hal_conf.h;stm32f4xx_hal_rcc_ex.h;stm32f4xx_hal_dma.h")
    create_mocks( 
      "Stm32F4xx_HAL_Driver"
      "${CMAKE_SOURCE_DIR}/src/Drivers/STM32F4xx_HAL_Driver/Inc/"
      "MOCK_HAL_DRIVER_DIR"
      "MOCK_HAL_DRIVER_SRC"
    )

    set(USE_ONLY_HAL_DRIVER_SRC 
      "stm32f4xx_hal.c"
      "stm32f4xx_hal_flash.c"
      "stm32f4xx_hal_flash_ex.c"
    )

    # List to hold filtered full paths
    set(FILTERED_MOCK_HAL_DRIVER_SRC)

    # Loop through the items in MOCK_HAL_DRIVER_SRC
    foreach(FILE ${MOCK_HAL_DRIVER_SRC})
        # Get the filename from the path
        get_filename_component(FILENAME ${FILE} NAME)
        
        # Check if the filename ends with any of the specified substrings
        foreach(SUBSTR ${USE_ONLY_HAL_DRIVER_SRC})
            if(${FILENAME} MATCHES "${SUBSTR}$")
                list(APPEND FILTERED_MOCK_HAL_DRIVER_SRC ${FILE})
            endif()
        endforeach()
    endforeach()

    # Output the filtered list of full paths
    message(STATUS "Filtered mock HAL driver sources: ${FILTERED_MOCK_HAL_DRIVER_SRC}")

    add_library(mock_hal_driver ${FILTERED_MOCK_HAL_DRIVER_SRC})
    target_include_directories(mock_hal_driver PRIVATE 
      ${MOCK_HAL_DRIVER_DIR}
      ${CMOCK_INCLUDE_DIRS} 
      "${CMAKE_SOURCE_DIR}/src/Drivers/STM32F4xx_HAL_Driver/Inc"
      "${CMAKE_SOURCE_DIR}/src/Core/Inc"
      "${CMAKE_SOURCE_DIR}/src/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
      "${CMAKE_SOURCE_DIR}/src/Drivers/CMSIS/Include/"
    )
    target_link_libraries(mock_hal_driver PRIVATE cmock_core)
    target_compile_options(mock_hal_driver PRIVATE
                          "-Wno-error"  # Third-party code is not our responsibility.
                          "-Wno-switch-enum"
                          "-Wno-missing-declarations"
                          "-Wno-sign-conversion"
                          "-Wno-double-promotion"
                          "-Wno-float-equal"
                          "-Wno-conversion"
                          )
    target_compile_definitions(mock_hal_driver PRIVATE STM32F446xx)

    add_library(io ${IO_SRC})
    target_include_directories(io PRIVATE ${INCLUDE_DIRS})
    add_dependencies(io dsdl-regulated dsdl-non-standard dsdl-mattfair nunavut-support)
    target_compile_options(io PRIVATE
      -Wno-error=missing-declarations
      -Wno-error=unused-variable
    )

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

    # Native test target
    enable_testing()

    set(TEST_LIBRARIES cmock_core unity_core mock_hal_driver)
    set(TEST_INCLUDE_DIRS 
      ${CPPUTest_INCLUDE_DIRS} 
      ${CMOCK_INCLUDE_DIRS} 
      ${MOCK_HAL_DRIVER_DIR}
    )

    #file(GLOB_RECURSE MOCK_SOURCES "tests/mocks/IO/*.cpp")
    #$add_library(mocks ${MOCK_SOURCES})
    #target_include_directories(mocks PRIVATE ${INCLUDE_DIRS} ${CPPUTest_INCLUDE_DIRS})

    #file(GLOB_RECURSE IO_TEST_SOURCES "tests/IO/*_test.cpp")
    #list(REMOVE_ITEM IO_TEST_SOURCES "${CMAKE_SOURCE_DIR}/tests/IO/register_test.cpp")

    set(ALL_TESTS "")
    set(ALL_TESTS_WITH_LCOV "")
    set(ALL_TEST_COVERAGE "")

    create_test(mock_hal_test tests/mocks/hal_test.c mock_hal_driver)
    target_include_directories(mock_hal_test PRIVATE 
      ${INCLUDE_DIRS} 
      ${TEST_INCLUDE_DIRS}
    )

    create_test(io_flash_test tests/IO/flash_test.c io mock_hal_driver)
    target_include_directories(io_flash_test PRIVATE 
      ${INCLUDE_DIRS} 
      ${TEST_INCLUDE_DIRS}
    )
    set(NATIVE_TEST_NAME io_flash_test)
    define_native_test_run_with_lcov(${NATIVE_TEST_NAME} ${VERIFICATIONS_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/\\*)
    define_natve_test_coverage(${NATIVE_TEST_NAME} ${VERIFICATIONS_BINARY_DIR})
    list(APPEND ALL_TESTS "run_${NATIVE_TEST_NAME}")
    list(APPEND ALL_TESTS_WITH_LCOV "run_${NATIVE_TEST_NAME}_with_lcov")
    list(APPEND ALL_TEST_COVERAGE "--add-tracefile")
    list(APPEND ALL_TEST_COVERAGE "${VERIFICATIONS_BINARY_DIR}/coverage.${NATIVE_TEST_NAME}.filtered.info")
    set(ALL_TESTS ${ALL_TESTS} PARENT_SCOPE)
    set(ALL_TESTS_WITH_LCOV ${ALL_TESTS_WITH_LCOV} PARENT_SCOPE)
    set(ALL_TEST_COVERAGE ${ALL_TEST_COVERAGE} PARENT_SCOPE)

    # Custom target to run all tests with ctest --output-on-failure
    add_custom_target(run_tests
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
        DEPENDS mock_hal_test
        DEPENDS io_flash_test
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running tests with verbose output on failure"
    )

    get_filename_component(NUNAVUT_PROJECT_ROOT "${CMAKE_SOURCE_DIR}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
    
    add_custom_target(
        lcov_zero
        ${LCOV}
              ${GOV_TOOL_ARG}
              --zerocounters
              --directory ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Resetting coverage counters."
    )


    add_custom_command(
        OUTPUT ${VERIFICATIONS_BINARY_DIR}/coverage.all.info
        COMMAND
              ${LCOV}
                  ${GOV_TOOL_ARG}
                  --rc lcov_branch_coverage=1
                  ${ALL_TEST_COVERAGE}
                  --output-file ${VERIFICATIONS_BINARY_DIR}/coverage.all.info
        DEPENDS ${ALL_TESTS_WITH_LCOV}
    )

    add_custom_command(
        OUTPUT ${VERIFICATIONS_BINARY_DIR}/coverage.info
        COMMAND
              ${LCOV}
                  ${GOV_TOOL_ARG}
                  --rc lcov_branch_coverage=1
                  --extract ${VERIFICATIONS_BINARY_DIR}/coverage.all.info
                            ${PROJECT_ROOT}/\\*
                  --output-file ${VERIFICATIONS_BINARY_DIR}/coverage.info
        DEPENDS ${VERIFICATIONS_BINARY_DIR}/coverage.all.info
    )

    add_custom_target(
        cov_info
        DEPENDS ${VERIFICATIONS_BINARY_DIR}/coverage.info
    )

    add_custom_target(
        cov_all
        ${GENHTML} --title "${PROJECT_NAME} native test coverage"
              --output-directory ${VERIFICATIONS_BINARY_DIR}/coverage/all
              --demangle-cpp
              --sort
              --num-spaces 4
              --function-coverage
              --branch-coverage
              --legend
              --highlight
              --show-details
              ${VERIFICATIONS_BINARY_DIR}/coverage.info
        DEPENDS ${VERIFICATIONS_BINARY_DIR}/coverage.info
        COMMENT "Build and run all tests and generate an overall html coverage report."
    )

else()
    # Cross compile for ARM
    apply_flag_set(${CMAKE_SOURCE_DIR}/cmake/compiler_flag_sets/native.cmake "" "" "")

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
