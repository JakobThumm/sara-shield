cmake_minimum_required(VERSION 3.0.2)
project(safety_shield)

## Compile as C++17
add_compile_options(-std=c++17)

# Add an option to enable Address Sanitizer
option(ENABLE_ASAN "Enable Address Sanitizer" OFF)

if (ENABLE_ASAN)
    message("Address Sanitizer enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
endif ()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Add an option for enabling logging
option(ENABLE_LOGGING "Enable logging using spdlog" ON)

## Add an option for using header config
option(USE_HEADER_CONFIG "Use header file for config without YAML dependency" OFF)

## Find catkin macros and libraries
#find_package(Eigen3 3.4 REQUIRED NO_MODULE)
INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
# Find Python
# IF YOU DON'T WANT TO USE CONDA, DISABLE THESE THREE LINES
#set(PYTHON_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python")
#set(PYTHON_INCLUDE_DIR "$ENV{CONDA_PREFIX}/include/python3.8")
#set(PYTHON_LIBRARY "$ENV{CONDA_PREFIX}/lib/libpython3.8.so")
#set(PYTHON_LIBRARY_DEBUG "$ENV{CONDA_PREFIX}/lib/libpython3.8.so")
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

include_directories(
        include
)

## Find Python
set (PYBIND11_PYTHON_VERSION "3.8")
set(PYTHON_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python")
set(PYTHON_INCLUDE_DIR "$ENV{CONDA_PREFIX}/include/python3.8")
set(PYTHON_LIBRARY "$ENV{CONDA_PREFIX}/lib/libpython3.8.dylib")
set(PYTHON_LIBRARY_DEBUG "$ENV{CONDA_PREFIX}/lib/libpython3.8.dylib")

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/safety_shield.cc
        src/long_term_traj.cc
        src/motion.cc
        src/path.cc
        src/robot_reach.cc
        src/human_reach.cc
        src/measurement_handler.cc
        src/kalman_filter.cc
        src/verify_iso.cc
        
)

# Add external subdirectories
add_subdirectory(external/pybind11)
add_subdirectory(external/SaRA/ReachLib)
add_subdirectory(external/LongTermPlanner)

# Conditionally include YAML dependency if USE_HEADER_CONFIG is OFF
if (USE_HEADER_CONFIG)
    add_definitions(-DUSE_HEADER_CONFIG)
else()
    add_subdirectory(external/yaml-cpp)
    target_include_directories(${PROJECT_NAME}
        PUBLIC external/yaml-cpp/include
    )
    target_link_directories(${PROJECT_NAME}
        PUBLIC external/yaml-cpp/src
    )
    target_link_libraries(${PROJECT_NAME} yaml-cpp)
endif()

# Conditionally include spdlog if logging is enabled
if (ENABLE_LOGGING)
    add_definitions(-DENABLE_LOGGING)
    add_subdirectory(external/spdlog)
    target_include_directories(${PROJECT_NAME}
        PUBLIC external/spdlog/include
    )
    target_link_directories(${PROJECT_NAME}
        PUBLIC external/spdlog/src
    )
    set(LOGGING_LIB spdlog)
else()
    set(LOGGING_LIB "")
endif()

target_include_directories(${PROJECT_NAME}
        PUBLIC external/pybind11/include
        PUBLIC external/SaRA/ReachLib/include
        PUBLIC external/LongTermPlanner/include
)

target_link_directories(${PROJECT_NAME}
        PUBLIC external/SaRA/ReachLib/src
        PUBLIC external/LongTermPlanner/src
)

target_link_libraries(${PROJECT_NAME} SaRA long_term_planner ${LOGGING_LIB})

install(TARGETS ${PROJECT_NAME} DESTINATION /usr/lib)

add_executable(debug src/debug.cc)
target_link_libraries(debug ${PROJECT_NAME} long_term_planner ${LOGGING_LIB})
#############
## GTest ##
#############
enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(long_term_traj_tests tests/src/long_term_traj_tests.cc)
target_include_directories(long_term_traj_tests PUBLIC tests/include)
target_link_libraries(long_term_traj_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(human_reach_tests tests/src/human_reach_tests.cc)
target_include_directories(human_reach_tests PUBLIC tests/include)
target_link_libraries(human_reach_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(motion_tests tests/src/motion_tests.cc)
target_include_directories(motion_tests PUBLIC tests/include)
target_link_libraries(motion_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(path_tests tests/src/path_tests.cc)
target_include_directories(path_tests PUBLIC tests/include)
target_link_libraries(path_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(robot_reach_tests tests/src/robot_reach_tests.cc)
target_include_directories(robot_reach_tests PUBLIC tests/include)
target_link_libraries(robot_reach_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(verify_iso_tests tests/src/verify_iso_tests.cc)
target_include_directories(verify_iso_tests PUBLIC tests/include)
target_link_libraries(verify_iso_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(safety_shield_tests tests/src/safety_shield_tests.cc)
target_include_directories(safety_shield_tests PUBLIC tests/include)
target_link_libraries(safety_shield_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
gtest_discover_tests(long_term_traj_tests)
gtest_discover_tests(human_reach_tests)
gtest_discover_tests(motion_tests)
gtest_discover_tests(path_tests)
gtest_discover_tests(robot_reach_tests)
gtest_discover_tests(verify_iso_tests)
gtest_discover_tests(safety_shield_tests)

#catkin_add_gtest(verify_iso_test tests/capsule_tests.cc tests/verify_iso_fixture.h)
#target_link_libraries(verify_iso_test ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#catkin_add_gtest(long_term_traj_tests tests/long_term_traj_tests.cc tests/long_term_traj_fixture.h)
#target_link_libraries(long_term_traj_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

##############
## Bindings ##
##############
pybind11_add_module(safety_shield_py MODULE bindings/safety_shield_bindings.cpp)
target_link_libraries(safety_shield_py PRIVATE ${PROJECT_NAME} long_term_planner ${LOGGING_LIB})
