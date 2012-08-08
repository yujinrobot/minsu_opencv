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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /opt/kobuki/minsu/minsu_opencv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/kobuki/minsu/minsu_opencv

# Include any dependencies generated for this target.
include src/CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/test.dir/flags.make

src/CMakeFiles/test.dir/test.o: src/CMakeFiles/test.dir/flags.make
src/CMakeFiles/test.dir/test.o: src/test.cpp
src/CMakeFiles/test.dir/test.o: manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/roslang/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/roscpp/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/rospy/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/roslib/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/share/message_filters/manifest.xml
src/CMakeFiles/test.dir/test.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kobuki/minsu/minsu_opencv/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/test.dir/test.o"
	cd /opt/kobuki/minsu/minsu_opencv/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/test.dir/test.o -c /opt/kobuki/minsu/minsu_opencv/src/test.cpp

src/CMakeFiles/test.dir/test.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/test.i"
	cd /opt/kobuki/minsu/minsu_opencv/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /opt/kobuki/minsu/minsu_opencv/src/test.cpp > CMakeFiles/test.dir/test.i

src/CMakeFiles/test.dir/test.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/test.s"
	cd /opt/kobuki/minsu/minsu_opencv/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /opt/kobuki/minsu/minsu_opencv/src/test.cpp -o CMakeFiles/test.dir/test.s

src/CMakeFiles/test.dir/test.o.requires:
.PHONY : src/CMakeFiles/test.dir/test.o.requires

src/CMakeFiles/test.dir/test.o.provides: src/CMakeFiles/test.dir/test.o.requires
	$(MAKE) -f src/CMakeFiles/test.dir/build.make src/CMakeFiles/test.dir/test.o.provides.build
.PHONY : src/CMakeFiles/test.dir/test.o.provides

src/CMakeFiles/test.dir/test.o.provides.build: src/CMakeFiles/test.dir/test.o

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

bin/test: src/CMakeFiles/test.dir/test.o
bin/test: src/CMakeFiles/test.dir/build.make
bin/test: src/CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/test"
	cd /opt/kobuki/minsu/minsu_opencv/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/test.dir/build: bin/test
.PHONY : src/CMakeFiles/test.dir/build

src/CMakeFiles/test.dir/requires: src/CMakeFiles/test.dir/test.o.requires
.PHONY : src/CMakeFiles/test.dir/requires

src/CMakeFiles/test.dir/clean:
	cd /opt/kobuki/minsu/minsu_opencv/src && $(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/test.dir/clean

src/CMakeFiles/test.dir/depend:
	cd /opt/kobuki/minsu/minsu_opencv && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/kobuki/minsu/minsu_opencv /opt/kobuki/minsu/minsu_opencv/src /opt/kobuki/minsu/minsu_opencv /opt/kobuki/minsu/minsu_opencv/src /opt/kobuki/minsu/minsu_opencv/src/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/test.dir/depend

