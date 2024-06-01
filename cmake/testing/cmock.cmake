function(create_mocks NAME PATH OUTPUT_PATH OUTPUT_SOURCE_LIST)
    set(mockdir ${CMAKE_BINARY_DIR}/generated/mocks/${NAME})
    set(mock_config ${CMAKE_BINARY_DIR}/${NAME}_cmock_config.yml)

    message(STATUS "CMock config file: ${mock_config}")
    message(STATUS "Mock directory: ${mockdir}")
    message(STATUS "Header absolute path: ${PATH}")
    message(STATUS "Output variable NAME: ${OUTPUT_VAR}")

    if(NOT EXISTS ${mock_config})
        message(FATAL_ERROR "CMock config file does not exist: ${mock_config}")
    endif()

    file(MAKE_DIRECTORY ${mockdir})

    SET(sources)

    file(GLOB HEADER_FILES "${PATH}/*.h")
    foreach(header_file IN LISTS HEADER_FILES)
      get_filename_component(filename_we ${header_file} NAME_WE)
      set(c_file ${mockdir}/mock_${filename_we}.c)
      add_custom_command(
          OUTPUT ${c_file}
          COMMAND ruby ${CMAKE_SOURCE_DIR}/submodules/cmock/lib/cmock.rb -o${mock_config} ${header_file}
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
          DEPENDS ${PATH}
          COMMENT "Generating mock for ${header_file}..."
      )

      list(APPEND sources ${c_file})
    endforeach()

    set(${OUTPUT_PATH} ${mockdir} PARENT_SCOPE)
    message(STATUS "Set ${OUTPUT_PATH} to ${mockdir}")

    set(${OUTPUT_SOURCE_LIST} ${sources} PARENT_SCOPE)
    message(STATUS "Set ${OUTPUT_SOURCE_LIST} to ${sources}")
endfunction()


# Function to generate the CMock configuration file
function(generate_cmock_config NAME MOCK_PREFIX INCLUDES)
    # Define the content of the CMock configuration file
    set(CMOCK_CONFIG_CONTENT "
:cmock:
  :mock_prefix: ${MOCK_PREFIX}
  :mock_path: ${CMAKE_BINARY_DIR}/generated/mocks/${NAME}
  :enforce_strict_ordering: true
  :plugins:
    - ignore
")
    # Check if INCLUDES is not empty
    if(INCLUDES)
        # Convert the list of includes to a YAML list format with correct indentation
        string(REPLACE ";" "\n    - " INCLUDES_YAML "${INCLUDES}")
        set(INCLUDES_YAML "    - ${INCLUDES_YAML}")

        # Append the includes to the CMock configuration content
        set(CMOCK_CONFIG_CONTENT "${CMOCK_CONFIG_CONTENT}
  :includes:
${INCLUDES_YAML}
")
    endif()

    # Set the path for the CMock configuration file in the build directory
    set(CMOCK_CONFIG_FILE ${CMAKE_BINARY_DIR}/${NAME}_cmock_config.yml)

    # Generate the CMock configuration file in the build directory
    file(WRITE ${CMOCK_CONFIG_FILE} "${CMOCK_CONFIG_CONTENT}")

    # Add a message to indicate that the file has been generated
    message(STATUS "Generated CMock configuration file at ${CMOCK_CONFIG_FILE}")
endfunction()
