# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds

# Include any dependencies generated for this target.
include test/CMakeFiles/test-sizes.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/test-sizes.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/test-sizes.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/test-sizes.dir/flags.make

test/CMakeFiles/test-sizes.dir/test_sizes.cpp.o: test/CMakeFiles/test-sizes.dir/flags.make
test/CMakeFiles/test-sizes.dir/test_sizes.cpp.o: /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_sizes.cpp
test/CMakeFiles/test-sizes.dir/test_sizes.cpp.o: test/CMakeFiles/test-sizes.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/test-sizes.dir/test_sizes.cpp.o"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/test-sizes.dir/test_sizes.cpp.o -MF CMakeFiles/test-sizes.dir/test_sizes.cpp.o.d -o CMakeFiles/test-sizes.dir/test_sizes.cpp.o -c /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_sizes.cpp

test/CMakeFiles/test-sizes.dir/test_sizes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test-sizes.dir/test_sizes.cpp.i"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_sizes.cpp > CMakeFiles/test-sizes.dir/test_sizes.cpp.i

test/CMakeFiles/test-sizes.dir/test_sizes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test-sizes.dir/test_sizes.cpp.s"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_sizes.cpp -o CMakeFiles/test-sizes.dir/test_sizes.cpp.s

test/CMakeFiles/test-sizes.dir/test_utils.cpp.o: test/CMakeFiles/test-sizes.dir/flags.make
test/CMakeFiles/test-sizes.dir/test_utils.cpp.o: /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_utils.cpp
test/CMakeFiles/test-sizes.dir/test_utils.cpp.o: test/CMakeFiles/test-sizes.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/test-sizes.dir/test_utils.cpp.o"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/test-sizes.dir/test_utils.cpp.o -MF CMakeFiles/test-sizes.dir/test_utils.cpp.o.d -o CMakeFiles/test-sizes.dir/test_utils.cpp.o -c /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_utils.cpp

test/CMakeFiles/test-sizes.dir/test_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test-sizes.dir/test_utils.cpp.i"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_utils.cpp > CMakeFiles/test-sizes.dir/test_utils.cpp.i

test/CMakeFiles/test-sizes.dir/test_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test-sizes.dir/test_utils.cpp.s"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test/test_utils.cpp -o CMakeFiles/test-sizes.dir/test_utils.cpp.s

# Object files for target test-sizes
test__sizes_OBJECTS = \
"CMakeFiles/test-sizes.dir/test_sizes.cpp.o" \
"CMakeFiles/test-sizes.dir/test_utils.cpp.o"

# External object files for target test-sizes
test__sizes_EXTERNAL_OBJECTS =

test/test-sizes: test/CMakeFiles/test-sizes.dir/test_sizes.cpp.o
test/test-sizes: test/CMakeFiles/test-sizes.dir/test_utils.cpp.o
test/test-sizes: test/CMakeFiles/test-sizes.dir/build.make
test/test-sizes: gtest/libgtest_main.a
test/test-sizes: gtest/libgtest.a
test/test-sizes: /home/user/robo/sb-robot/microros_ws/install/microcdr/lib/libmicrocdr.a
test/test-sizes: /home/user/robo/sb-robot/microros_ws/install/microxrcedds_client/lib/libmicroxrcedds_client.a
test/test-sizes: librmw_microxrcedds.so
test/test-sizes: /opt/ros/humble/lib/librmw.so
test/test-sizes: /home/user/robo/sb-robot/microros_ws/install/microxrcedds_client/lib/libmicroxrcedds_client.a
test/test-sizes: /home/user/robo/sb-robot/microros_ws/install/microcdr/lib/libmicrocdr.a
test/test-sizes: /home/user/robo/sb-robot/microros_ws/install/rosidl_typesupport_microxrcedds_c/lib/librosidl_typesupport_microxrcedds_c.a
test/test-sizes: /opt/ros/humble/lib/librosidl_runtime_c.so
test/test-sizes: /opt/ros/humble/lib/librcutils.so
test/test-sizes: test/CMakeFiles/test-sizes.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test-sizes"
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-sizes.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/test-sizes.dir/build: test/test-sizes
.PHONY : test/CMakeFiles/test-sizes.dir/build

test/CMakeFiles/test-sizes.dir/clean:
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test && $(CMAKE_COMMAND) -P CMakeFiles/test-sizes.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/test-sizes.dir/clean

test/CMakeFiles/test-sizes.dir/depend:
	cd /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c /home/user/robo/sb-robot/microros_ws/src/uros/rmw_microxrcedds/rmw_microxrcedds_c/test /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test /home/user/robo/sb-robot/microros_ws/build/rmw_microxrcedds/test/CMakeFiles/test-sizes.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/test-sizes.dir/depend

