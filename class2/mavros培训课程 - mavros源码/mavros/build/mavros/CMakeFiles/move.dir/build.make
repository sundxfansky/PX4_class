# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/nvidia/mavros/src/mavros/mavros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/mavros/build/mavros

# Include any dependencies generated for this target.
include CMakeFiles/move.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/move.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/move.dir/flags.make

CMakeFiles/move.dir/src/move.cpp.o: CMakeFiles/move.dir/flags.make
CMakeFiles/move.dir/src/move.cpp.o: /home/nvidia/mavros/src/mavros/mavros/src/move.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/mavros/build/mavros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/move.dir/src/move.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/move.dir/src/move.cpp.o -c /home/nvidia/mavros/src/mavros/mavros/src/move.cpp

CMakeFiles/move.dir/src/move.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/move.dir/src/move.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/mavros/src/mavros/mavros/src/move.cpp > CMakeFiles/move.dir/src/move.cpp.i

CMakeFiles/move.dir/src/move.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/move.dir/src/move.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/mavros/src/mavros/mavros/src/move.cpp -o CMakeFiles/move.dir/src/move.cpp.s

CMakeFiles/move.dir/src/move.cpp.o.requires:

.PHONY : CMakeFiles/move.dir/src/move.cpp.o.requires

CMakeFiles/move.dir/src/move.cpp.o.provides: CMakeFiles/move.dir/src/move.cpp.o.requires
	$(MAKE) -f CMakeFiles/move.dir/build.make CMakeFiles/move.dir/src/move.cpp.o.provides.build
.PHONY : CMakeFiles/move.dir/src/move.cpp.o.provides

CMakeFiles/move.dir/src/move.cpp.o.provides.build: CMakeFiles/move.dir/src/move.cpp.o


# Object files for target move
move_OBJECTS = \
"CMakeFiles/move.dir/src/move.cpp.o"

# External object files for target move
move_EXTERNAL_OBJECTS =

/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: CMakeFiles/move.dir/src/move.cpp.o
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: CMakeFiles/move.dir/build.make
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /home/nvidia/mavros/devel/.private/mavros/lib/libmavros.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libclass_loader.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/libPocoFoundation.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libdl.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libroslib.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librospack.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libtinyxml.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libtf2_ros.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libactionlib.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libmessage_filters.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libroscpp.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_signals.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libtf2.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /home/nvidia/mavros/devel/.private/libmavconn/lib/libmavconn.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole_bridge.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libeigen_conversions.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librostime.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libcpp_common.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libclass_loader.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/libPocoFoundation.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libdl.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libroslib.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librospack.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libtinyxml.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libtf2_ros.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libactionlib.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libmessage_filters.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libroscpp.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_signals.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libtf2.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /home/nvidia/mavros/devel/.private/libmavconn/lib/libmavconn.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole_bridge.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libeigen_conversions.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/librostime.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /opt/ros/kinetic/lib/libcpp_common.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: /usr/lib/aarch64-linux-gnu/libGeographic.so
/home/nvidia/mavros/devel/.private/mavros/lib/mavros/move: CMakeFiles/move.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/mavros/build/mavros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/nvidia/mavros/devel/.private/mavros/lib/mavros/move"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/move.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/move.dir/build: /home/nvidia/mavros/devel/.private/mavros/lib/mavros/move

.PHONY : CMakeFiles/move.dir/build

CMakeFiles/move.dir/requires: CMakeFiles/move.dir/src/move.cpp.o.requires

.PHONY : CMakeFiles/move.dir/requires

CMakeFiles/move.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/move.dir/cmake_clean.cmake
.PHONY : CMakeFiles/move.dir/clean

CMakeFiles/move.dir/depend:
	cd /home/nvidia/mavros/build/mavros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/mavros/src/mavros/mavros /home/nvidia/mavros/src/mavros/mavros /home/nvidia/mavros/build/mavros /home/nvidia/mavros/build/mavros /home/nvidia/mavros/build/mavros/CMakeFiles/move.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/move.dir/depend

