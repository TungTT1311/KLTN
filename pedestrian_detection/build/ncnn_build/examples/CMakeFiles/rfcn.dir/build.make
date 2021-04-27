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
include ncnn_build/examples/CMakeFiles/rfcn.dir/depend.make

# Include the progress variables for this target.
include ncnn_build/examples/CMakeFiles/rfcn.dir/progress.make

# Include the compile flags for this target's objects.
include ncnn_build/examples/CMakeFiles/rfcn.dir/flags.make

ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o: ncnn_build/examples/CMakeFiles/rfcn.dir/flags.make
ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o: /home/jetson/Desktop/person_detection/ncnn/examples/rfcn.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rfcn.dir/rfcn.cpp.o -c /home/jetson/Desktop/person_detection/ncnn/examples/rfcn.cpp

ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rfcn.dir/rfcn.cpp.i"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/ncnn/examples/rfcn.cpp > CMakeFiles/rfcn.dir/rfcn.cpp.i

ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rfcn.dir/rfcn.cpp.s"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/ncnn/examples/rfcn.cpp -o CMakeFiles/rfcn.dir/rfcn.cpp.s

ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.requires:

.PHONY : ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.requires

ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.provides: ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.requires
	$(MAKE) -f ncnn_build/examples/CMakeFiles/rfcn.dir/build.make ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.provides.build
.PHONY : ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.provides

ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.provides.build: ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o


# Object files for target rfcn
rfcn_OBJECTS = \
"CMakeFiles/rfcn.dir/rfcn.cpp.o"

# External object files for target rfcn
rfcn_EXTERNAL_OBJECTS =

ncnn_build/examples/rfcn: ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o
ncnn_build/examples/rfcn: ncnn_build/examples/CMakeFiles/rfcn.dir/build.make
ncnn_build/examples/rfcn: ncnn_build/src/libncnn.a
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.1.1
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.1.1
ncnn_build/examples/rfcn: /usr/lib/gcc/aarch64-linux-gnu/7/libgomp.so
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libpthread.so
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libvulkan.so
ncnn_build/examples/rfcn: ncnn_build/glslang/glslang/libglslang.a
ncnn_build/examples/rfcn: ncnn_build/glslang/SPIRV/libSPIRV.a
ncnn_build/examples/rfcn: ncnn_build/glslang/glslang/libMachineIndependent.a
ncnn_build/examples/rfcn: ncnn_build/glslang/OGLCompilersDLL/libOGLCompiler.a
ncnn_build/examples/rfcn: ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a
ncnn_build/examples/rfcn: ncnn_build/glslang/glslang/libGenericCodeGen.a
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1
ncnn_build/examples/rfcn: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1
ncnn_build/examples/rfcn: ncnn_build/examples/CMakeFiles/rfcn.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rfcn"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rfcn.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ncnn_build/examples/CMakeFiles/rfcn.dir/build: ncnn_build/examples/rfcn

.PHONY : ncnn_build/examples/CMakeFiles/rfcn.dir/build

ncnn_build/examples/CMakeFiles/rfcn.dir/requires: ncnn_build/examples/CMakeFiles/rfcn.dir/rfcn.cpp.o.requires

.PHONY : ncnn_build/examples/CMakeFiles/rfcn.dir/requires

ncnn_build/examples/CMakeFiles/rfcn.dir/clean:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && $(CMAKE_COMMAND) -P CMakeFiles/rfcn.dir/cmake_clean.cmake
.PHONY : ncnn_build/examples/CMakeFiles/rfcn.dir/clean

ncnn_build/examples/CMakeFiles/rfcn.dir/depend:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/Desktop/person_detection/pedestrian_detection /home/jetson/Desktop/person_detection/ncnn/examples /home/jetson/Desktop/person_detection/pedestrian_detection/build /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples/CMakeFiles/rfcn.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ncnn_build/examples/CMakeFiles/rfcn.dir/depend

