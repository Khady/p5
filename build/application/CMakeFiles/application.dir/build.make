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
CMAKE_COMMAND = /usr/local/Cellar/cmake/2.8.12.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/2.8.12.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/local/Cellar/cmake/2.8.12.2/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/louisroche/Code/graph/p5/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/louisroche/Code/graph/p5/build

# Include any dependencies generated for this target.
include application/CMakeFiles/application.dir/depend.make

# Include the progress variables for this target.
include application/CMakeFiles/application.dir/progress.make

# Include the compile flags for this target's objects.
include application/CMakeFiles/application.dir/flags.make

application/CMakeFiles/application.dir/application.cpp.o: application/CMakeFiles/application.dir/flags.make
application/CMakeFiles/application.dir/application.cpp.o: /Users/louisroche/Code/graph/p5/src/application/application.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/louisroche/Code/graph/p5/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object application/CMakeFiles/application.dir/application.cpp.o"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/application.dir/application.cpp.o -c /Users/louisroche/Code/graph/p5/src/application/application.cpp

application/CMakeFiles/application.dir/application.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/application.dir/application.cpp.i"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/louisroche/Code/graph/p5/src/application/application.cpp > CMakeFiles/application.dir/application.cpp.i

application/CMakeFiles/application.dir/application.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/application.dir/application.cpp.s"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/louisroche/Code/graph/p5/src/application/application.cpp -o CMakeFiles/application.dir/application.cpp.s

application/CMakeFiles/application.dir/application.cpp.o.requires:
.PHONY : application/CMakeFiles/application.dir/application.cpp.o.requires

application/CMakeFiles/application.dir/application.cpp.o.provides: application/CMakeFiles/application.dir/application.cpp.o.requires
	$(MAKE) -f application/CMakeFiles/application.dir/build.make application/CMakeFiles/application.dir/application.cpp.o.provides.build
.PHONY : application/CMakeFiles/application.dir/application.cpp.o.provides

application/CMakeFiles/application.dir/application.cpp.o.provides.build: application/CMakeFiles/application.dir/application.cpp.o

application/CMakeFiles/application.dir/camera_roam.cpp.o: application/CMakeFiles/application.dir/flags.make
application/CMakeFiles/application.dir/camera_roam.cpp.o: /Users/louisroche/Code/graph/p5/src/application/camera_roam.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/louisroche/Code/graph/p5/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object application/CMakeFiles/application.dir/camera_roam.cpp.o"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/application.dir/camera_roam.cpp.o -c /Users/louisroche/Code/graph/p5/src/application/camera_roam.cpp

application/CMakeFiles/application.dir/camera_roam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/application.dir/camera_roam.cpp.i"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/louisroche/Code/graph/p5/src/application/camera_roam.cpp > CMakeFiles/application.dir/camera_roam.cpp.i

application/CMakeFiles/application.dir/camera_roam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/application.dir/camera_roam.cpp.s"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/louisroche/Code/graph/p5/src/application/camera_roam.cpp -o CMakeFiles/application.dir/camera_roam.cpp.s

application/CMakeFiles/application.dir/camera_roam.cpp.o.requires:
.PHONY : application/CMakeFiles/application.dir/camera_roam.cpp.o.requires

application/CMakeFiles/application.dir/camera_roam.cpp.o.provides: application/CMakeFiles/application.dir/camera_roam.cpp.o.requires
	$(MAKE) -f application/CMakeFiles/application.dir/build.make application/CMakeFiles/application.dir/camera_roam.cpp.o.provides.build
.PHONY : application/CMakeFiles/application.dir/camera_roam.cpp.o.provides

application/CMakeFiles/application.dir/camera_roam.cpp.o.provides.build: application/CMakeFiles/application.dir/camera_roam.cpp.o

application/CMakeFiles/application.dir/imageio.cpp.o: application/CMakeFiles/application.dir/flags.make
application/CMakeFiles/application.dir/imageio.cpp.o: /Users/louisroche/Code/graph/p5/src/application/imageio.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/louisroche/Code/graph/p5/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object application/CMakeFiles/application.dir/imageio.cpp.o"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/application.dir/imageio.cpp.o -c /Users/louisroche/Code/graph/p5/src/application/imageio.cpp

application/CMakeFiles/application.dir/imageio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/application.dir/imageio.cpp.i"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/louisroche/Code/graph/p5/src/application/imageio.cpp > CMakeFiles/application.dir/imageio.cpp.i

application/CMakeFiles/application.dir/imageio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/application.dir/imageio.cpp.s"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/louisroche/Code/graph/p5/src/application/imageio.cpp -o CMakeFiles/application.dir/imageio.cpp.s

application/CMakeFiles/application.dir/imageio.cpp.o.requires:
.PHONY : application/CMakeFiles/application.dir/imageio.cpp.o.requires

application/CMakeFiles/application.dir/imageio.cpp.o.provides: application/CMakeFiles/application.dir/imageio.cpp.o.requires
	$(MAKE) -f application/CMakeFiles/application.dir/build.make application/CMakeFiles/application.dir/imageio.cpp.o.provides.build
.PHONY : application/CMakeFiles/application.dir/imageio.cpp.o.provides

application/CMakeFiles/application.dir/imageio.cpp.o.provides.build: application/CMakeFiles/application.dir/imageio.cpp.o

application/CMakeFiles/application.dir/scene_loader.cpp.o: application/CMakeFiles/application.dir/flags.make
application/CMakeFiles/application.dir/scene_loader.cpp.o: /Users/louisroche/Code/graph/p5/src/application/scene_loader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/louisroche/Code/graph/p5/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object application/CMakeFiles/application.dir/scene_loader.cpp.o"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/application.dir/scene_loader.cpp.o -c /Users/louisroche/Code/graph/p5/src/application/scene_loader.cpp

application/CMakeFiles/application.dir/scene_loader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/application.dir/scene_loader.cpp.i"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/louisroche/Code/graph/p5/src/application/scene_loader.cpp > CMakeFiles/application.dir/scene_loader.cpp.i

application/CMakeFiles/application.dir/scene_loader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/application.dir/scene_loader.cpp.s"
	cd /Users/louisroche/Code/graph/p5/build/application && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/louisroche/Code/graph/p5/src/application/scene_loader.cpp -o CMakeFiles/application.dir/scene_loader.cpp.s

application/CMakeFiles/application.dir/scene_loader.cpp.o.requires:
.PHONY : application/CMakeFiles/application.dir/scene_loader.cpp.o.requires

application/CMakeFiles/application.dir/scene_loader.cpp.o.provides: application/CMakeFiles/application.dir/scene_loader.cpp.o.requires
	$(MAKE) -f application/CMakeFiles/application.dir/build.make application/CMakeFiles/application.dir/scene_loader.cpp.o.provides.build
.PHONY : application/CMakeFiles/application.dir/scene_loader.cpp.o.provides

application/CMakeFiles/application.dir/scene_loader.cpp.o.provides.build: application/CMakeFiles/application.dir/scene_loader.cpp.o

# Object files for target application
application_OBJECTS = \
"CMakeFiles/application.dir/application.cpp.o" \
"CMakeFiles/application.dir/camera_roam.cpp.o" \
"CMakeFiles/application.dir/imageio.cpp.o" \
"CMakeFiles/application.dir/scene_loader.cpp.o"

# External object files for target application
application_EXTERNAL_OBJECTS =

application/libapplication.a: application/CMakeFiles/application.dir/application.cpp.o
application/libapplication.a: application/CMakeFiles/application.dir/camera_roam.cpp.o
application/libapplication.a: application/CMakeFiles/application.dir/imageio.cpp.o
application/libapplication.a: application/CMakeFiles/application.dir/scene_loader.cpp.o
application/libapplication.a: application/CMakeFiles/application.dir/build.make
application/libapplication.a: application/CMakeFiles/application.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libapplication.a"
	cd /Users/louisroche/Code/graph/p5/build/application && $(CMAKE_COMMAND) -P CMakeFiles/application.dir/cmake_clean_target.cmake
	cd /Users/louisroche/Code/graph/p5/build/application && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/application.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
application/CMakeFiles/application.dir/build: application/libapplication.a
.PHONY : application/CMakeFiles/application.dir/build

application/CMakeFiles/application.dir/requires: application/CMakeFiles/application.dir/application.cpp.o.requires
application/CMakeFiles/application.dir/requires: application/CMakeFiles/application.dir/camera_roam.cpp.o.requires
application/CMakeFiles/application.dir/requires: application/CMakeFiles/application.dir/imageio.cpp.o.requires
application/CMakeFiles/application.dir/requires: application/CMakeFiles/application.dir/scene_loader.cpp.o.requires
.PHONY : application/CMakeFiles/application.dir/requires

application/CMakeFiles/application.dir/clean:
	cd /Users/louisroche/Code/graph/p5/build/application && $(CMAKE_COMMAND) -P CMakeFiles/application.dir/cmake_clean.cmake
.PHONY : application/CMakeFiles/application.dir/clean

application/CMakeFiles/application.dir/depend:
	cd /Users/louisroche/Code/graph/p5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/louisroche/Code/graph/p5/src /Users/louisroche/Code/graph/p5/src/application /Users/louisroche/Code/graph/p5/build /Users/louisroche/Code/graph/p5/build/application /Users/louisroche/Code/graph/p5/build/application/CMakeFiles/application.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : application/CMakeFiles/application.dir/depend

