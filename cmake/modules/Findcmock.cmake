#
# Framework : CMock
# Homepage: http://www.throwtheswitch.org/cmock/
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#

set(CMOCK_SUBMODULE "${CMAKE_SOURCE_DIR}/submodules/cmock")

if(EXISTS "${CMOCK_SUBMODULE}/src")
    set(CMOCK_FOUND ON)
else()
    message(FATAL_ERROR "Couldn't find ${CMOCK_SUBMODULE}/src. Did you forget to git submodule --init?")
endif()

include_directories(
    ${CMOCK_SUBMODULE}/src
)

add_library(cmock_core STATIC EXCLUDE_FROM_ALL
            ${CMOCK_SUBMODULE}/src/cmock.c
)

target_include_directories(cmock_core PRIVATE
                           ${CMOCK_SUBMODULE}/src
)

target_compile_options(unity_core PUBLIC
                        "-Wno-float-equal"
                        "-Wno-double-promotion"
                        "-Wno-switch-enum"
                        "-Wno-conversion"
                        "-DCMOCK_INCLUDE_DOUBLE"
                        "${CMOCK_EXTRA_COMPILE_CFLAGS}"
                       )
target_compile_definitions(cmock_core PUBLIC
                           ${CMOCK_EXTRA_COMPILE_DEFS}
                          )
target_link_libraries(cmock_core PRIVATE unity_core)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(cmock
    REQUIRED_VARS CMOCK_FOUND
)
