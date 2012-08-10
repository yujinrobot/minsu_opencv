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
CMAKE_SOURCE_DIR = /opt/kobuki/minsu/temp/minsu_opencv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/kobuki/minsu/temp/minsu_opencv/build

# Include any dependencies generated for this target.
include src/CMakeFiles/tracking_Redball_openCV.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/tracking_Redball_openCV.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/tracking_Redball_openCV.dir/flags.make

src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: src/CMakeFiles/tracking_Redball_openCV.dir/flags.make
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: ../src/tracking_Redball_openCV.cpp
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: ../manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/roslang/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/roscpp/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/rospy/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/roslib/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/share/message_filters/manifest.xml
src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/kobuki/minsu/temp/minsu_opencv/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o"
	cd /opt/kobuki/minsu/temp/minsu_opencv/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o -c /opt/kobuki/minsu/temp/minsu_opencv/src/tracking_Redball_openCV.cpp

src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.i"
	cd /opt/kobuki/minsu/temp/minsu_opencv/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /opt/kobuki/minsu/temp/minsu_opencv/src/tracking_Redball_openCV.cpp > CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.i

src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.s"
	cd /opt/kobuki/minsu/temp/minsu_opencv/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /opt/kobuki/minsu/temp/minsu_opencv/src/tracking_Redball_openCV.cpp -o CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.s

src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.requires:
.PHONY : src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.requires

src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.provides: src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.requires
	$(MAKE) -f src/CMakeFiles/tracking_Redball_openCV.dir/build.make src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.provides.build
.PHONY : src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.provides

src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.provides.build: src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o

# Object files for target tracking_Redball_openCV
tracking_Redball_openCV_OBJECTS = \
"CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o"

# External object files for target tracking_Redball_openCV
tracking_Redball_openCV_EXTERNAL_OBJECTS =

../bin/tracking_Redball_openCV: src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o
../bin/tracking_Redball_openCV: src/CMakeFiles/tracking_Redball_openCV.dir/build.make
../bin/tracking_Redball_openCV: src/CMakeFiles/tracking_Redball_openCV.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/tracking_Redball_openCV"
	cd /opt/kobuki/minsu/temp/minsu_opencv/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking_Redball_openCV.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/tracking_Redball_openCV.dir/build: ../bin/tracking_Redball_openCV
.PHONY : src/CMakeFiles/tracking_Redball_openCV.dir/build

src/CMakeFiles/tracking_Redball_openCV.dir/requires: src/CMakeFiles/tracking_Redball_openCV.dir/tracking_Redball_openCV.o.requires
.PHONY : src/CMakeFiles/tracking_Redball_openCV.dir/requires

src/CMakeFiles/tracking_Redball_openCV.dir/clean:
	cd /opt/kobuki/minsu/temp/minsu_opencv/build/src && $(CMAKE_COMMAND) -P CMakeFiles/tracking_Redball_openCV.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/tracking_Redball_openCV.dir/clean

src/CMakeFiles/tracking_Redball_openCV.dir/depend:
	cd /opt/kobuki/minsu/temp/minsu_opencv/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/kobuki/minsu/temp/minsu_opencv /opt/kobuki/minsu/temp/minsu_opencv/src /opt/kobuki/minsu/temp/minsu_opencv/build /opt/kobuki/minsu/temp/minsu_opencv/build/src /opt/kobuki/minsu/temp/minsu_opencv/build/src/CMakeFiles/tracking_Redball_openCV.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/tracking_Redball_openCV.dir/depend

