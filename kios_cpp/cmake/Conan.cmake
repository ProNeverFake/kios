if(NOT EXISTS "${PROJECT_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD  "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake"
        "${PROJECT_BINARY_DIR}/conan.cmake"
        STATUS DOWNLOAD_STATUS)
    # Separate the returned status code, and error message.
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    # Check if download was successful.
    if(${STATUS_CODE} EQUAL 0)
      message(STATUS "Download completed successfully!")
    else()
      # Exit CMake if the download failed, printing the error message.
      message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
    endif()

endif()
include(${PROJECT_BINARY_DIR}/conan.cmake)

conan_check(REQUIRED)
conan_cmake_run(
    CONANFILE
    conanfile.txt
    BASIC_SETUP
#    CONAN_COMMAND
#    ${CONAN_CMD}
    CMAKE_TARGETS
    BUILD
    missing
    )

#include(${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
#conan_basic_setup()

