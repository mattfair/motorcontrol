#
# Framework : Unity
# Homepage: http://www.throwtheswitch.org/unity/
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#

set(UNITY_SUBMODULE "${CMAKE_SOURCE_DIR}/submodules/cmock/vendor/unity")

if(EXISTS "${UNITY_SUBMODULE}/src")
    set(UNITY_FOUND ON)
else()
    message(FATAL_ERROR "Couldn't find ${UNITY_SUBMODULE}/src. Did you forget to git submodule --init?")
endif()

include_directories(
    ${UNITY_SUBMODULE}/src
)

add_library(unity_core STATIC EXCLUDE_FROM_ALL
            ${UNITY_SUBMODULE}/src/unity.c
)

target_include_directories(unity_core PRIVATE
                           ${UNITY_SUBMODULE}/src
)

target_compile_options(unity_core PUBLIC
                        "-Wno-float-equal"
                        "-Wno-double-promotion"
                        "-Wno-switch-enum"
                        "-Wno-conversion"
                        "-Wno-missing-declarations"
                        "-DUNITY_INCLUDE_DOUBLE"
                        "-DUNITY_USE_COMMAND_LINE_ARGS"
                        "${UNITY_EXTRA_COMPILE_CFLAGS}"
                       )
target_compile_definitions(unity_core PUBLIC
                           ${UNITY_EXTRA_COMPILE_DEFS}
                          )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(unity
    REQUIRED_VARS UNITY_FOUND
)
