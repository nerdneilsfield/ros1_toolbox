
CPMAddPackage("gh:fmtlib/fmt#7.1.3")
CPMAddPackage("gh:CLIUtils/CLI11@2.3.1")
# CPMAddPackage("gh:gabime/spdlog@1.8.2")

# CPMAddPackage("gh:catchorg/Catch2@3.2.0")

find_package(PCL REQUIRED)
message(STATUS "PCL_VERSION: ${PCL_VERSION}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")