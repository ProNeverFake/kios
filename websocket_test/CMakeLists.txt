cmake_minimum_required(VERSION 3.2)
project(communication_test)

# pthread issues
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_subdirectory(division)
set(SOURCE_FILES main.cpp)

add_executable(communication_test ${SOURCE_FILES})

target_link_libraries(communication_test 
    PRIVATE
        Threads::Threads
    )

install(TARGETS 
    communication_test
    DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR}
    )
