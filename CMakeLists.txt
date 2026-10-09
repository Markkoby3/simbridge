cmake_minimum_required(VERSION 3.16)
project(SimBridge VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SIMBRIDGE_BUILD_TESTS "Build the GoogleTest suite" ON)
option(SIMBRIDGE_SANITIZE "Build with AddressSanitizer and UBSan" OFF)
set(SIMBRIDGE_WITH_DDS AUTO CACHE STRING "Build the Cyclone DDS transport: AUTO, ON or OFF")
set_property(CACHE SIMBRIDGE_WITH_DDS PROPERTY STRINGS AUTO ON OFF)

find_package(Threads REQUIRED)

set(SIMBRIDGE_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
if(SIMBRIDGE_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

set(SIMBRIDGE_PLUGIN_DIR ${CMAKE_BINARY_DIR}/plugins)

# Core library: bus, scenario parser, plugin loader, backend adapters, engine, UDP bridge.
add_library(simbridge_core STATIC
  src/scenario.cpp
  src/plugin_registry.cpp
  src/kinematic_backend.cpp
  src/legacy_block_sim.cpp
  src/legacy_blocks_adapter.cpp
  src/engine.cpp
  src/wire.cpp
  src/udp_bridge.cpp)
target_include_directories(simbridge_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(simbridge_core PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
target_compile_options(simbridge_core PRIVATE ${SIMBRIDGE_WARNINGS})
set_target_properties(simbridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Plugins are loadable modules that depend only on the public headers.
function(simbridge_add_plugin name)
  add_library(${name} MODULE ${ARGN})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(${name} PRIVATE ${SIMBRIDGE_WARNINGS})
  set_target_properties(${name} PROPERTIES
    PREFIX ""
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    LIBRARY_OUTPUT_DIRECTORY ${SIMBRIDGE_PLUGIN_DIR})
endfunction()

# Optional DDS transport (Eclipse Cyclone DDS). AUTO builds it when Cyclone DDS is installed.
set(SIMBRIDGE_HAS_DDS OFF)
if(NOT SIMBRIDGE_WITH_DDS STREQUAL "OFF")
  if(SIMBRIDGE_WITH_DDS STREQUAL "ON")
    find_package(CycloneDDS REQUIRED)
  else()
    find_package(CycloneDDS QUIET)
  endif()
  if(CycloneDDS_FOUND)
    set(SIMBRIDGE_HAS_DDS ON)
    idlc_generate(TARGET simbridge_dds_types FILES idl/simbridge_types.idl DEFAULT_EXTENSIBILITY final)
    add_library(simbridge_dds STATIC src/dds_bridge.cpp)
    target_link_libraries(simbridge_dds PUBLIC simbridge_core CycloneDDS::ddsc PRIVATE simbridge_dds_types)
    # Expose the generated type header so tools and tests can speak the same DDS types.
    target_include_directories(simbridge_dds PUBLIC $<TARGET_PROPERTY:simbridge_dds_types,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_options(simbridge_dds PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${SIMBRIDGE_WARNINGS}>)
    set_target_properties(simbridge_dds PROPERTIES POSITION_INDEPENDENT_CODE ON)
  endif()
endif()
message(STATUS "SimBridge DDS transport: ${SIMBRIDGE_HAS_DDS}")

simbridge_add_plugin(waypoint_follower plugins/waypoint_follower/waypoint_follower.cpp)
simbridge_add_plugin(radar_sensor plugins/radar_sensor/radar_sensor.cpp)

foreach(app simbridge_run simbridge_bench)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} PRIVATE simbridge_core)
  target_compile_options(${app} PRIVATE ${SIMBRIDGE_WARNINGS})
  target_compile_definitions(${app} PRIVATE SIMBRIDGE_DEFAULT_PLUGIN_DIR="${SIMBRIDGE_PLUGIN_DIR}")
  add_dependencies(${app} waypoint_follower radar_sensor)
endforeach()
if(SIMBRIDGE_HAS_DDS)
  target_link_libraries(simbridge_run PRIVATE simbridge_dds)
  target_compile_definitions(simbridge_run PRIVATE SIMBRIDGE_HAS_DDS=1)

  # Example autonomy process: shares only the IDL types and topic names with the simulation.
  add_executable(simbridge_autonomy apps/simbridge_autonomy.cpp)
  target_link_libraries(simbridge_autonomy PRIVATE simbridge_dds)
  target_compile_options(simbridge_autonomy PRIVATE ${SIMBRIDGE_WARNINGS})
endif()

if(SIMBRIDGE_BUILD_TESTS)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_library(bad_abi_plugin MODULE tests/plugins/bad_abi_plugin.cpp)
  target_include_directories(bad_abi_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set_target_properties(bad_abi_plugin PROPERTIES
    PREFIX "" CXX_VISIBILITY_PRESET hidden
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test_plugins)

  add_executable(simbridge_tests
    tests/test_message_bus.cpp
    tests/test_scenario.cpp
    tests/test_plugin_registry.cpp
    tests/test_backends.cpp
    tests/test_wire.cpp
    tests/test_engine.cpp
    tests/test_udp_bridge.cpp)
  target_link_libraries(simbridge_tests PRIVATE simbridge_core GTest::gtest_main)
  target_compile_definitions(simbridge_tests PRIVATE
    SIMBRIDGE_PLUGIN_DIR="${SIMBRIDGE_PLUGIN_DIR}"
    SIMBRIDGE_TEST_PLUGIN_DIR="${CMAKE_BINARY_DIR}/test_plugins"
    SIMBRIDGE_SCENARIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scenarios")
  add_dependencies(simbridge_tests waypoint_follower radar_sensor bad_abi_plugin)

  if(SIMBRIDGE_HAS_DDS)
    target_sources(simbridge_tests PRIVATE tests/test_dds_bridge.cpp)
    target_link_libraries(simbridge_tests PRIVATE simbridge_dds)
  endif()

  include(GoogleTest)
  gtest_discover_tests(simbridge_tests)
endif()
