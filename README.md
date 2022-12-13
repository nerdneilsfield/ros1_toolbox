# cpp-project-template

A very simple modern cpp project template

Base on:

- cmake template: [aminya/project_options](https://github.com/aminya/project_options)
- dependencies: [CPM](https://github.com/cpm-cmake/CPM.cmake)

## Add dependencies

You need to specify dependencies for your project in `Dependencies.cmake` file.

1. Use `find_package` to find the package
1. Use `CPMAddPackage` to add the dependencies

Noice that link with third party dependencies use:

```cmake
target_link_system_libraries(
  intro
  PRIVATE
  fmt::fmt
  spdlog::spdlog
  CLI11::CLI11
)

```

## Use `make` to build and test your project

```bash

# make project in debug mode
make debug

# make project in release mode
make release

# test project in debug mode
make test

# test project in release mode
make test-release

# install project
make install

# install project in debug mode
make install-debug

# pack project in release mode
make pack

# pack project in debug mode
make pack-debug

# format the code
make format

```
