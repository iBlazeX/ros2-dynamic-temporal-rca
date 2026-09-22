cmake_minimum_required(VERSION 3.22)
project(diagnostic_tui CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json 3.9 REQUIRED)   # Ubuntu 24.04: nlohmann-json3-dev 3.11.3

# ---------------------------------------------------------------- FTXUI
# No Ubuntu package for FTXUI exists, so the library is fetched at configure time
# from a PINNED commit (tag v5.0.0) and linked statically into the executable.
# Set -DFTXUI_SOURCE_DIR=<path> to build from a local checkout instead (offline).
set(FTXUI_PINNED_TAG "v5.0.0")
set(FTXUI_PINNED_COMMIT "cdf28903a7781f97ba94d30b79c3a4b0c97ccce7")
set(FTXUI_SOURCE_DIR "" CACHE PATH "Optional local FTXUI ${FTXUI_PINNED_TAG} checkout")

set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(FTXUI_QUIET ON CACHE BOOL "" FORCE)

include(FetchContent)
if(FTXUI_SOURCE_DIR)
  FetchContent_Declare(ftxui SOURCE_DIR "${FTXUI_SOURCE_DIR}")
else()
  FetchContent_Declare(ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI
    GIT_TAG        ${FTXUI_PINNED_COMMIT}
    GIT_SHALLOW    OFF)
endif()
FetchContent_MakeAvailable(ftxui)

# ---------------------------------------------------------------- library (ROS-free, testable)
add_library(dashboard_state STATIC
  src/dashboard_state.cpp)
target_include_directories(dashboard_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(dashboard_state PUBLIC nlohmann_json::nlohmann_json)

add_library(dashboard_render STATIC
  src/render.cpp)
target_link_libraries(dashboard_render PUBLIC dashboard_state ftxui::dom ftxui::screen)

# ---------------------------------------------------------------- executable
add_executable(rca_tui src/rca_tui_main.cpp)
target_link_libraries(rca_tui PRIVATE dashboard_render ftxui::component
  rclcpp::rclcpp ${std_msgs_TARGETS})

install(TARGETS rca_tui DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------- tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dashboard_state test/test_dashboard_state.cpp)
  target_link_libraries(test_dashboard_state dashboard_render)
endif()

ament_package()
