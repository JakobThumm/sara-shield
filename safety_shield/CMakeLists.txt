cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0057 NEW)

project(safety_shield)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(ENABLE_COVERAGE)
  # set compiler flags
  set(CMAKE_CXX_FLAGS "-O0 -coverage -fno-elide-constructors -fno-default-inline -fprofile-arcs")

  # find required tools
  find_program(LCOV lcov REQUIRED)
  find_program(GENHTML genhtml REQUIRED)

  # add coverage target
  add_custom_target(coverage
    # gather data
    COMMAND ${LCOV} --directory . --capture --output-file coverage.info --exclude '/usr/*' --exclude '*external/*' --exclude '*tests/*'
    # generate report
    COMMAND ${GENHTML} --demangle-cpp -o coverage coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

## Find catkin macros and libraries
#find_package(Eigen3 3.4 REQUIRED NO_MODULE)
INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
# Find Python
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)

if(BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
    find_package(pybind11 QUIET)

    if(Python3_FOUND AND pybind11_FOUND)
        message(STATUS "Building Python bindings")
        pybind11_add_module(safety_shield_py MODULE bindings/safety_shield_bindings.cpp)
        target_link_libraries(safety_shield_py PRIVATE ${PROJECT_NAME} long_term_planner spdlog yaml-cpp Python3::Module)
        set_target_properties(safety_shield_py PROPERTIES
            INSTALL_RPATH "$ORIGIN/../lib"
            BUILD_WITH_INSTALL_RPATH TRUE)
        install(TARGETS safety_shield_py DESTINATION ${SKBUILD_PROJECT_NAME})
        install(FILES __init__.py DESTINATION ${SKBUILD_PROJECT_NAME})
    else()
        message(WARNING "Python or pybind11 not found. Skipping Python bindings.")
    endif()
endif()

include_directories(
        include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/safety_shield.cc
        src/long_term_traj.cc
        src/motion.cc
        src/path.cc
        src/robot_reach.cc
        src/human_reach.cc
        src/verify_iso.cc
        src/kalman_filter.cc
        src/measurement_handler.cc
        src/trajectory_utils.cc
        src/verification_utils.cc
        src/config_utils.cc
        #src/advanced_verify_iso.cc
        )

# Add external subdirectories
# add_subdirectory(external/pybind11)
add_subdirectory(external/SaRA/ReachLib)
add_subdirectory(external/LongTermPlanner)
add_subdirectory(external/spdlog)
add_subdirectory(external/yaml-cpp)

target_include_directories(${PROJECT_NAME}
        # PUBLIC external/pybind11/include
        PUBLIC external/SaRA/ReachLib/include
        PUBLIC external/LongTermPlanner/include
        PUBLIC external/spdlog/include
        PUBLIC external/yaml-cpp/include
        )

target_link_directories(${PROJECT_NAME}
        PUBLIC external/SaRA/ReachLib/src
        PUBLIC external/LongTermPlanner/src
        PUBLIC external/spdlog/src
        PUBLIC external/yaml-cpp/src
        )

target_link_libraries(${PROJECT_NAME} SaRA long_term_planner spdlog yaml-cpp)

if(BUILD_PYTHON_BINDINGS)
  # Python module is already installed via the pybind11 target above
else()
  include(GNUInstallDirs)
  install(TARGETS ${PROJECT_NAME}
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()


add_executable(debug src/debug.cc)
target_link_libraries(debug ${PROJECT_NAME} long_term_planner spdlog yaml-cpp)

add_executable(self_constrained_collision_debug src/self_constrained_collision_debug.cc)
target_link_libraries(self_constrained_collision_debug ${PROJECT_NAME} long_term_planner spdlog yaml-cpp)

add_executable(unconstrained_collision_debug src/unconstrained_collision_debug.cc)
target_link_libraries(unconstrained_collision_debug ${PROJECT_NAME} long_term_planner spdlog yaml-cpp)

add_executable(hrgym_scenario_debug src/hrgym_scenario_debug.cc)
target_link_libraries(hrgym_scenario_debug ${PROJECT_NAME} long_term_planner spdlog yaml-cpp)

add_executable(debug_real_world_setup src/debug_real_world_setup.cc)
target_link_libraries(debug_real_world_setup ${PROJECT_NAME} long_term_planner spdlog yaml-cpp)

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
add_executable(trajectory_utils_tests tests/src/trajectory_utils_tests.cc)
target_include_directories(trajectory_utils_tests PUBLIC tests/include)
target_link_libraries(trajectory_utils_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
add_executable(verification_utils_tests tests/src/verification_utils_tests.cc)
target_include_directories(verification_utils_tests PUBLIC tests/include)
target_link_libraries(verification_utils_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
gtest_discover_tests(long_term_traj_tests)
gtest_discover_tests(human_reach_tests)
gtest_discover_tests(motion_tests)
gtest_discover_tests(path_tests)
gtest_discover_tests(robot_reach_tests)
gtest_discover_tests(verify_iso_tests)
gtest_discover_tests(safety_shield_tests)
gtest_discover_tests(trajectory_utils_tests)
gtest_discover_tests(verification_utils_tests)

configure_file(${CMAKE_SOURCE_DIR}/CTestCustom.cmake ${CMAKE_BINARY_DIR})

#catkin_add_gtest(verify_iso_test tests/capsule_tests.cc tests/verify_iso_fixture.h)
#target_link_libraries(verify_iso_test ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#catkin_add_gtest(long_term_traj_tests tests/long_term_traj_tests.cc tests/long_term_traj_fixture.h)
#target_link_libraries(long_term_traj_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

