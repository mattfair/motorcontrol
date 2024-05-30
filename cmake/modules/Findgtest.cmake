#
# Framework : Googletest
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#

set(GOOGLETEST_SUBMODULE "${CMAKE_SOURCE_DIR}/submodules/googletest")

if(EXISTS "${GOOGLETEST_SUBMODULE}/googletest")
    set(GTEST_FOUND ON)
else()
    message(FATAL_ERROR "Couldn't find Googletest. Did you forget to git submodule update --init?")
endif()

include_directories(
    SYSTEM ${GOOGLETEST_SUBMODULE}/googletest/include
    SYSTEM ${GOOGLETEST_SUBMODULE}/googlemock/include
)

add_library(my_gmock_main STATIC EXCLUDE_FROM_ALL
            ${GOOGLETEST_SUBMODULE}/googletest/src/gtest-all.cc
            ${GOOGLETEST_SUBMODULE}/googlemock/src/gmock-all.cc
            ${GOOGLETEST_SUBMODULE}/googlemock/src/gmock_main.cc
)

target_include_directories(my_gmock_main PRIVATE
            SYSTEM ${GOOGLETEST_SUBMODULE}/googletest
            SYSTEM ${GOOGLETEST_SUBMODULE}/googlemock
)

target_compile_options(my_gmock_main PRIVATE
                       "-Wno-error"  # Third-party code is not our responsibility.
                       "-Wno-switch-enum"
                       "-Wno-zero-as-null-pointer-constant"
                       "-Wno-missing-declarations"
                       "-Wno-sign-conversion"
                       "-Wno-double-promotion"
                       "-Wno-float-equal"
                       "-Wno-conversion"
                       "-DGTEST_HAS_PTHREAD=0"
                       "${NUNAVUT_VERIFICATION_EXTRA_COMPILE_CFLAGS}"
                       )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(gtest
    REQUIRED_VARS GTEST_FOUND
)
