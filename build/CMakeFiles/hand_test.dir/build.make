# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/galaxy/Desktop/hand_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/galaxy/Desktop/hand_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/hand_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hand_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hand_test.dir/flags.make

CMakeFiles/hand_test.dir/hand_test.c.o: CMakeFiles/hand_test.dir/flags.make
CMakeFiles/hand_test.dir/hand_test.c.o: ../hand_test.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/galaxy/Desktop/hand_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/hand_test.dir/hand_test.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/hand_test.dir/hand_test.c.o   -c /home/galaxy/Desktop/hand_sdk/hand_test.c

CMakeFiles/hand_test.dir/hand_test.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/hand_test.dir/hand_test.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/galaxy/Desktop/hand_sdk/hand_test.c > CMakeFiles/hand_test.dir/hand_test.c.i

CMakeFiles/hand_test.dir/hand_test.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/hand_test.dir/hand_test.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/galaxy/Desktop/hand_sdk/hand_test.c -o CMakeFiles/hand_test.dir/hand_test.c.s

# Object files for target hand_test
hand_test_OBJECTS = \
"CMakeFiles/hand_test.dir/hand_test.c.o"

# External object files for target hand_test
hand_test_EXTERNAL_OBJECTS =

hand_test: CMakeFiles/hand_test.dir/hand_test.c.o
hand_test: CMakeFiles/hand_test.dir/build.make
hand_test: libhand_sdkLib.so
hand_test: /usr/lib/x86_64-linux-gnu/libprotobuf-c.so
hand_test: /usr/lib/x86_64-linux-gnu/libprotobuf.so
hand_test: CMakeFiles/hand_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/galaxy/Desktop/hand_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable hand_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hand_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hand_test.dir/build: hand_test

.PHONY : CMakeFiles/hand_test.dir/build

CMakeFiles/hand_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hand_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hand_test.dir/clean

CMakeFiles/hand_test.dir/depend:
	cd /home/galaxy/Desktop/hand_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/galaxy/Desktop/hand_sdk /home/galaxy/Desktop/hand_sdk /home/galaxy/Desktop/hand_sdk/build /home/galaxy/Desktop/hand_sdk/build /home/galaxy/Desktop/hand_sdk/build/CMakeFiles/hand_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hand_test.dir/depend

