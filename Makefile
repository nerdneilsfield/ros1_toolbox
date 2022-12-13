debug: config-debug
	cmake --build build/debug -j $(nproc)
release: config-release
	cmake --build build/release -j $(nproc)

config-debug:
	cmake -B build/debug -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=$HOME/.local -DFEATURE_TESTS=ON -S .

config-release:
	cmake -B build/release -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local -S .

test::
	cd build/debug && ctest

test-release::
	cd build/release && ctest

install::
	cmake --install build/release

install-debug::
	cmake --install build/debug

pack::
	cd build/release && cpack

pack-debug::
	cd build/debug && cpack

format:
	find src -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	find test -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	find fuzz_test -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	find configured_files -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	# find src -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	# find test -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	# find fuzz_test -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	# find configured_files -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	# cmake-format -c .cmake-format.yaml -i CMakeLists.txt

clean:
	rm -rf build/debug

clean-release:
	rm -rf build/release