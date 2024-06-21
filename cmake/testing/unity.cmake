function(create_test test_name test_src)
  set(TEST_RUNNERS_DIR ${CMAKE_BINARY_DIR}/generated/test_runners)
  file(MAKE_DIRECTORY ${TEST_RUNNERS_DIR})

  # ensure that the CMock configuration file is present
  set(CONFIG_CONTENT "
:unity:
  :enforce_strict_ordering: true
  :use_param_tests: true
  :cmdline_args: true
")
  set (CONFIG_FILE ${TEST_RUNNERS_DIR}/unity_config.yml)
  file(WRITE ${CONFIG_FILE} "${CONFIG_CONTENT}")

  get_filename_component( test_src_absolute ${test_src} REALPATH )
  set(TEST_RUNNER ${TEST_RUNNERS_DIR}/${test_name}_runner.c)
  add_custom_command    (
    OUTPUT ${TEST_RUNNER}
    COMMAND
    ruby ${CMAKE_SOURCE_DIR}/submodules/cmock/vendor/unity/auto/generate_test_runner.rb
    ${CONFIG_FILE}
    ${test_src_absolute} ${TEST_RUNNER}
    DEPENDS ${test_src})
  add_executable        (${test_name} ${test_src} ${TEST_RUNNER})

  # make sure we don't get warnings
  target_compile_options(${test_name} PRIVATE
    -Wno-missing-declarations
  )

  target_link_libraries (${test_name} unity_core)
  foreach(dep ${ARGN})
    target_link_libraries(${test_name} ${dep})
  endforeach()
  add_test              (${test_name} ${test_name})
endfunction()
