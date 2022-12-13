# find_package(fmt CONFIG)
# find_package(spdlog CONFIG)
# find_package(CLI11 CONFIG REQUIRED)

# Generic test that uses conan libs
add_executable(intro main.cpp)
target_link_libraries(intro  PRIVATE project_options project_warnings)
target_link_system_libraries(
  intro
  PRIVATE
  fmt::fmt
  CLI11::CLI11
  ${catkin_LIBRARIES}
)


target_include_directories(intro PRIVATE "${CMAKE_BINARY_DIR}/configured_files/include")

