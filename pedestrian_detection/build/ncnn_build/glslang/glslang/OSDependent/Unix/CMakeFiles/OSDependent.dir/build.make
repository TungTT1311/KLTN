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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jetson/Desktop/person_detection/pedestrian_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetson/Desktop/person_detection/pedestrian_detection/build

# Include any dependencies generated for this target.
include ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/depend.make

# Include the progress variables for this target.
include ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/progress.make

# Include the compile flags for this target's objects.
include ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/flags.make

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/flags.make
ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o: /home/jetson/Desktop/person_detection/ncnn/glslang/glslang/OSDependent/Unix/ossource.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OSDependent.dir/ossource.cpp.o -c /home/jetson/Desktop/person_detection/ncnn/glslang/glslang/OSDependent/Unix/ossource.cpp

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OSDependent.dir/ossource.cpp.i"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/ncnn/glslang/glslang/OSDependent/Unix/ossource.cpp > CMakeFiles/OSDependent.dir/ossource.cpp.i

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OSDependent.dir/ossource.cpp.s"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/ncnn/glslang/glslang/OSDependent/Unix/ossource.cpp -o CMakeFiles/OSDependent.dir/ossource.cpp.s

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.requires:

.PHONY : ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.requires

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.provides: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.requires
	$(MAKE) -f ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/build.make ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.provides.build
.PHONY : ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.provides

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.provides.build: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o


# Object files for target OSDependent
OSDependent_OBJECTS = \
"CMakeFiles/OSDependent.dir/ossource.cpp.o"

# External object files for target OSDependent
OSDependent_EXTERNAL_OBJECTS =

ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o
ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/build.make
ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libOSDependent.a"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix && $(CMAKE_COMMAND) -P CMakeFiles/OSDependent.dir/cmake_clean_target.cmake
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OSDependent.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/build: ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a

.PHONY : ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/build

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/requires: ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/ossource.cpp.o.requires

.PHONY : ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/requires

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/clean:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix && $(CMAKE_COMMAND) -P CMakeFiles/OSDependent.dir/cmake_clean.cmake
.PHONY : ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/clean

ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/depend:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/Desktop/person_detection/pedestrian_detection /home/jetson/Desktop/person_detection/ncnn/glslang/glslang/OSDependent/Unix /home/jetson/Desktop/person_detection/pedestrian_detection/build /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ncnn_build/glslang/glslang/OSDependent/Unix/CMakeFiles/OSDependent.dir/depend
