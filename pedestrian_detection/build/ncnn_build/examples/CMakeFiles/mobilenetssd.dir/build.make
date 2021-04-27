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
include ncnn_build/examples/CMakeFiles/mobilenetssd.dir/depend.make

# Include the progress variables for this target.
include ncnn_build/examples/CMakeFiles/mobilenetssd.dir/progress.make

# Include the compile flags for this target's objects.
include ncnn_build/examples/CMakeFiles/mobilenetssd.dir/flags.make

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/flags.make
ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o: /home/jetson/Desktop/person_detection/ncnn/examples/mobilenetssd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o -c /home/jetson/Desktop/person_detection/ncnn/examples/mobilenetssd.cpp

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.i"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jetson/Desktop/person_detection/ncnn/examples/mobilenetssd.cpp > CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.i

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.s"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jetson/Desktop/person_detection/ncnn/examples/mobilenetssd.cpp -o CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.s

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.requires:

.PHONY : ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.requires

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.provides: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.requires
	$(MAKE) -f ncnn_build/examples/CMakeFiles/mobilenetssd.dir/build.make ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.provides.build
.PHONY : ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.provides

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.provides.build: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o


# Object files for target mobilenetssd
mobilenetssd_OBJECTS = \
"CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o"

# External object files for target mobilenetssd
mobilenetssd_EXTERNAL_OBJECTS =

ncnn_build/examples/mobilenetssd: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o
ncnn_build/examples/mobilenetssd: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/build.make
ncnn_build/examples/mobilenetssd: ncnn_build/src/libncnn.a
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.1.1
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.1.1
ncnn_build/examples/mobilenetssd: /usr/lib/gcc/aarch64-linux-gnu/7/libgomp.so
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libpthread.so
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libvulkan.so
ncnn_build/examples/mobilenetssd: ncnn_build/glslang/glslang/libglslang.a
ncnn_build/examples/mobilenetssd: ncnn_build/glslang/SPIRV/libSPIRV.a
ncnn_build/examples/mobilenetssd: ncnn_build/glslang/glslang/libMachineIndependent.a
ncnn_build/examples/mobilenetssd: ncnn_build/glslang/OGLCompilersDLL/libOGLCompiler.a
ncnn_build/examples/mobilenetssd: ncnn_build/glslang/glslang/OSDependent/Unix/libOSDependent.a
ncnn_build/examples/mobilenetssd: ncnn_build/glslang/glslang/libGenericCodeGen.a
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1
ncnn_build/examples/mobilenetssd: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1
ncnn_build/examples/mobilenetssd: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jetson/Desktop/person_detection/pedestrian_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mobilenetssd"
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mobilenetssd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ncnn_build/examples/CMakeFiles/mobilenetssd.dir/build: ncnn_build/examples/mobilenetssd

.PHONY : ncnn_build/examples/CMakeFiles/mobilenetssd.dir/build

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/requires: ncnn_build/examples/CMakeFiles/mobilenetssd.dir/mobilenetssd.cpp.o.requires

.PHONY : ncnn_build/examples/CMakeFiles/mobilenetssd.dir/requires

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/clean:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples && $(CMAKE_COMMAND) -P CMakeFiles/mobilenetssd.dir/cmake_clean.cmake
.PHONY : ncnn_build/examples/CMakeFiles/mobilenetssd.dir/clean

ncnn_build/examples/CMakeFiles/mobilenetssd.dir/depend:
	cd /home/jetson/Desktop/person_detection/pedestrian_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/Desktop/person_detection/pedestrian_detection /home/jetson/Desktop/person_detection/ncnn/examples /home/jetson/Desktop/person_detection/pedestrian_detection/build /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples /home/jetson/Desktop/person_detection/pedestrian_detection/build/ncnn_build/examples/CMakeFiles/mobilenetssd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ncnn_build/examples/CMakeFiles/mobilenetssd.dir/depend

