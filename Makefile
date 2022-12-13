build:
	catkin build --no-status --no-notify --no-color --summarize ros1_toolbox

format:
	find src -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	# find test -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	# find fuzz_test -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	find configured_files -regex '.*\.\(cpp\|h\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
	find src -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	# find test -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	# find fuzz_test -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	find configured_files -regex '.*\.\(cmake\)' -exec cmake-format -c .cmake-format.yaml -i {} \;
	cmake-format -c .cmake-format.yaml -i CMakeLists.txt

clean:
	catkin clean --yes ros1_toolbox

intro:
	cd ../.. && bash -c "source devel/setup.bash; rosrun ros1_toolbox intro"

help:
	@echo "build: Build the package"
	@echo "format: Format the code"
	@echo "clean: Clean the package"
	@echo "help: Show this help"