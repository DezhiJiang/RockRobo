# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/lzy/clion-2018.1.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/lzy/clion-2018.1.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lzy/RockroboBridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lzy/RockroboBridge/cmake-build-debug

# Include any dependencies generated for this target.
include gmock/gtest/CMakeFiles/gtest.dir/depend.make

# Include the progress variables for this target.
include gmock/gtest/CMakeFiles/gtest.dir/progress.make

# Include the compile flags for this target's objects.
include gmock/gtest/CMakeFiles/gtest.dir/flags.make

gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: gmock/gtest/CMakeFiles/gtest.dir/flags.make
gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: /usr/src/gtest/src/gtest-all.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lzy/RockroboBridge/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o"
	cd /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtest.dir/src/gtest-all.cc.o -c /usr/src/gtest/src/gtest-all.cc

gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest.dir/src/gtest-all.cc.i"
	cd /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/src/gtest/src/gtest-all.cc > CMakeFiles/gtest.dir/src/gtest-all.cc.i

gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest.dir/src/gtest-all.cc.s"
	cd /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/src/gtest/src/gtest-all.cc -o CMakeFiles/gtest.dir/src/gtest-all.cc.s

gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires:

.PHONY : gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires

gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides: gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires
	$(MAKE) -f gmock/gtest/CMakeFiles/gtest.dir/build.make gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides.build
.PHONY : gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides

gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides.build: gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o


# Object files for target gtest
gtest_OBJECTS = \
"CMakeFiles/gtest.dir/src/gtest-all.cc.o"

# External object files for target gtest
gtest_EXTERNAL_OBJECTS =

gmock/gtest/libgtest.a: gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o
gmock/gtest/libgtest.a: gmock/gtest/CMakeFiles/gtest.dir/build.make
gmock/gtest/libgtest.a: gmock/gtest/CMakeFiles/gtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lzy/RockroboBridge/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgtest.a"
	cd /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest && $(CMAKE_COMMAND) -P CMakeFiles/gtest.dir/cmake_clean_target.cmake
	cd /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gmock/gtest/CMakeFiles/gtest.dir/build: gmock/gtest/libgtest.a

.PHONY : gmock/gtest/CMakeFiles/gtest.dir/build

gmock/gtest/CMakeFiles/gtest.dir/requires: gmock/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires

.PHONY : gmock/gtest/CMakeFiles/gtest.dir/requires

gmock/gtest/CMakeFiles/gtest.dir/clean:
	cd /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest && $(CMAKE_COMMAND) -P CMakeFiles/gtest.dir/cmake_clean.cmake
.PHONY : gmock/gtest/CMakeFiles/gtest.dir/clean

gmock/gtest/CMakeFiles/gtest.dir/depend:
	cd /home/lzy/RockroboBridge/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lzy/RockroboBridge /usr/src/gtest /home/lzy/RockroboBridge/cmake-build-debug /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest /home/lzy/RockroboBridge/cmake-build-debug/gmock/gtest/CMakeFiles/gtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gmock/gtest/CMakeFiles/gtest.dir/depend

