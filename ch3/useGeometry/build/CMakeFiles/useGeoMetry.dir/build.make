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
CMAKE_SOURCE_DIR = /home/wang/Desktop/slambook/ch3/useGeometry

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wang/Desktop/slambook/ch3/useGeometry/build

# Include any dependencies generated for this target.
include CMakeFiles/useGeoMetry.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/useGeoMetry.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/useGeoMetry.dir/flags.make

CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o: CMakeFiles/useGeoMetry.dir/flags.make
CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o: ../useGeometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wang/Desktop/slambook/ch3/useGeometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o -c /home/wang/Desktop/slambook/ch3/useGeometry/useGeometry.cpp

CMakeFiles/useGeoMetry.dir/useGeometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/useGeoMetry.dir/useGeometry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/Desktop/slambook/ch3/useGeometry/useGeometry.cpp > CMakeFiles/useGeoMetry.dir/useGeometry.cpp.i

CMakeFiles/useGeoMetry.dir/useGeometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/useGeoMetry.dir/useGeometry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/Desktop/slambook/ch3/useGeometry/useGeometry.cpp -o CMakeFiles/useGeoMetry.dir/useGeometry.cpp.s

CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.requires:

.PHONY : CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.requires

CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.provides: CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.requires
	$(MAKE) -f CMakeFiles/useGeoMetry.dir/build.make CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.provides.build
.PHONY : CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.provides

CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.provides.build: CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o


# Object files for target useGeoMetry
useGeoMetry_OBJECTS = \
"CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o"

# External object files for target useGeoMetry
useGeoMetry_EXTERNAL_OBJECTS =

useGeoMetry: CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o
useGeoMetry: CMakeFiles/useGeoMetry.dir/build.make
useGeoMetry: CMakeFiles/useGeoMetry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wang/Desktop/slambook/ch3/useGeometry/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable useGeoMetry"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/useGeoMetry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/useGeoMetry.dir/build: useGeoMetry

.PHONY : CMakeFiles/useGeoMetry.dir/build

CMakeFiles/useGeoMetry.dir/requires: CMakeFiles/useGeoMetry.dir/useGeometry.cpp.o.requires

.PHONY : CMakeFiles/useGeoMetry.dir/requires

CMakeFiles/useGeoMetry.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/useGeoMetry.dir/cmake_clean.cmake
.PHONY : CMakeFiles/useGeoMetry.dir/clean

CMakeFiles/useGeoMetry.dir/depend:
	cd /home/wang/Desktop/slambook/ch3/useGeometry/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/Desktop/slambook/ch3/useGeometry /home/wang/Desktop/slambook/ch3/useGeometry /home/wang/Desktop/slambook/ch3/useGeometry/build /home/wang/Desktop/slambook/ch3/useGeometry/build /home/wang/Desktop/slambook/ch3/useGeometry/build/CMakeFiles/useGeoMetry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/useGeoMetry.dir/depend
