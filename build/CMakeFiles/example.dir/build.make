# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kijin/workspace/example1_sp_hi_fd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kijin/workspace/example1_sp_hi_fd/build

# Include any dependencies generated for this target.
include CMakeFiles/example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example.dir/flags.make

CMakeFiles/example.dir/src/main.o: CMakeFiles/example.dir/flags.make
CMakeFiles/example.dir/src/main.o: ../src/main.cpp
CMakeFiles/example.dir/src/main.o: ../manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/example.dir/src/main.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/kijin/workspace/example1_sp_hi_fd/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/example.dir/src/main.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/example.dir/src/main.o -c /home/kijin/workspace/example1_sp_hi_fd/src/main.cpp

CMakeFiles/example.dir/src/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example.dir/src/main.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/kijin/workspace/example1_sp_hi_fd/src/main.cpp > CMakeFiles/example.dir/src/main.i

CMakeFiles/example.dir/src/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example.dir/src/main.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/kijin/workspace/example1_sp_hi_fd/src/main.cpp -o CMakeFiles/example.dir/src/main.s

CMakeFiles/example.dir/src/main.o.requires:
.PHONY : CMakeFiles/example.dir/src/main.o.requires

CMakeFiles/example.dir/src/main.o.provides: CMakeFiles/example.dir/src/main.o.requires
	$(MAKE) -f CMakeFiles/example.dir/build.make CMakeFiles/example.dir/src/main.o.provides.build
.PHONY : CMakeFiles/example.dir/src/main.o.provides

CMakeFiles/example.dir/src/main.o.provides.build: CMakeFiles/example.dir/src/main.o

# Object files for target example
example_OBJECTS = \
"CMakeFiles/example.dir/src/main.o"

# External object files for target example
example_EXTERNAL_OBJECTS =

../bin/example: CMakeFiles/example.dir/src/main.o
../bin/example: CMakeFiles/example.dir/build.make
../bin/example: CMakeFiles/example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example.dir/build: ../bin/example
.PHONY : CMakeFiles/example.dir/build

CMakeFiles/example.dir/requires: CMakeFiles/example.dir/src/main.o.requires
.PHONY : CMakeFiles/example.dir/requires

CMakeFiles/example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example.dir/clean

CMakeFiles/example.dir/depend:
	cd /home/kijin/workspace/example1_sp_hi_fd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kijin/workspace/example1_sp_hi_fd /home/kijin/workspace/example1_sp_hi_fd /home/kijin/workspace/example1_sp_hi_fd/build /home/kijin/workspace/example1_sp_hi_fd/build /home/kijin/workspace/example1_sp_hi_fd/build/CMakeFiles/example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example.dir/depend

