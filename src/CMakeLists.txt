# find_package(fmt CONFIG)
# find_package(spdlog CONFIG)
# find_package(CLI11 CONFIG REQUIRED)

# Generic test that uses conan libs
add_executable(intro main.cpp)
target_link_libraries(intro  PRIVATE project_options project_warnings  ${catkin_LIBRARIES}
)
target_link_system_libraries(
  intro
  PRIVATE
  fmt::fmt
  CLI11::CLI11
)
target_include_directories(intro PRIVATE "${CMAKE_BINARY_DIR}/configured_files/include")


add_executable(point_cloud_extractor point_cloud_extractor.cpp)
target_link_libraries(point_cloud_extractor PRIVATE
  project_options
  project_warnings
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_system_libraries(
  point_cloud_extractor
  PRIVATE
  fmt::fmt
  CLI11::CLI11
)
target_include_directories(point_cloud_extractor 
  PRIVATE
  "${CMAKE_BINARY_DIR}/configured_files/include" 
  ${PCL_INCLUDE_DIRS}
)


add_executable(navfix_to_csv navfix_to_csv.cpp)
target_link_libraries(navfix_to_csv PRIVATE
  project_options
  project_warnings
  ${catkin_LIBRARIES}
)
target_link_system_libraries(
  navfix_to_csv
  PRIVATE
  fmt::fmt
  CLI11::CLI11
)
target_include_directories(navfix_to_csv 
  PRIVATE
  "${CMAKE_BINARY_DIR}/configured_files/include" 
)