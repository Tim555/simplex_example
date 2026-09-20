cmake_minimum_required(VERSION 3.20)

project(SimplexExample LANGUAGES CXX)

option(ENABLE_CLANG_TIDY "Run clang-tidy during C++ compilation" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

set(PROJECT_SOURCES
    src/main.cpp
)

set(PROJECT_HEADERS
)

set(SIMPLEX_SOURCES
    src/lp/solver.cpp
    src/lp/linear_program.cpp
    src/lp/lp_parser.cpp
)

set(SIMPLEX_HEADERS
    src/lp/solver.hpp
)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)

add_executable(simplex_example
    ${PROJECT_SOURCES}
    ${PROJECT_HEADERS}
)

target_include_directories(simplex_example PRIVATE src)

include(FetchContent)

FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen googletest yaml-cpp)

enable_testing()

add_library(simplex
    ${SIMPLEX_SOURCES}
    ${SIMPLEX_HEADERS}
)

target_include_directories(simplex PUBLIC src)
target_link_libraries(simplex PUBLIC Eigen3::Eigen)
target_link_libraries(simplex PUBLIC yaml-cpp::yaml-cpp)
target_link_libraries(simplex_example PRIVATE simplex)

add_executable(simplex_example_test
    test/linear_program_test.cpp
    test/simplex_test.cpp
    test/lp_parser_test.cpp
    ${PROJECT_HEADERS}
)

target_compile_definitions(simplex_example_test PRIVATE SIMPLEX_EXAMPLE_TESTING)
target_include_directories(simplex_example_test PRIVATE src)
target_link_libraries(simplex_example_test PRIVATE GTest::gtest_main simplex)

include(GoogleTest)
gtest_discover_tests(simplex_example_test
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
if(CLANG_FORMAT_EXECUTABLE)
    add_custom_target(format
        COMMAND ${CLANG_FORMAT_EXECUTABLE} -i
            ${PROJECT_SOURCES}
            ${PROJECT_HEADERS}
            ${SIMPLEX_SOURCES}
            ${SIMPLEX_HEADERS}
            test/linear_program_test.cpp
            test/simplex_test.cpp
            test/lp_parser_test.cpp
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Formatting C++ sources"
    )
endif()

if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(CLANG_TIDY_COMMAND
        ${CLANG_TIDY_EXECUTABLE}
        --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy
    )
    set_target_properties(simplex simplex_example simplex_example_test
        PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_COMMAND}"
    )
endif()
