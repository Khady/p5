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
include SDLmain/CMakeFiles/SDLmain.dir/depend.make

# Include the progress variables for this target.
include SDLmain/CMakeFiles/SDLmain.dir/progress.make

# Include the compile flags for this target's objects.
include SDLmain/CMakeFiles/SDLmain.dir/flags.make

SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o: SDLmain/CMakeFiles/SDLmain.dir/flags.make
SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o: /Users/louisroche/Code/graph/p5/src/SDLmain/SDLmain.m
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/louisroche/Code/graph/p5/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o"
	cd /Users/louisroche/Code/graph/p5/build/SDLmain && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/SDLmain.dir/SDLmain.m.o -c /Users/louisroche/Code/graph/p5/src/SDLmain/SDLmain.m

SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SDLmain.dir/SDLmain.m.i"
	cd /Users/louisroche/Code/graph/p5/build/SDLmain && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/louisroche/Code/graph/p5/src/SDLmain/SDLmain.m > CMakeFiles/SDLmain.dir/SDLmain.m.i

SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SDLmain.dir/SDLmain.m.s"
	cd /Users/louisroche/Code/graph/p5/build/SDLmain && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/louisroche/Code/graph/p5/src/SDLmain/SDLmain.m -o CMakeFiles/SDLmain.dir/SDLmain.m.s

SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.requires:
.PHONY : SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.requires

SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.provides: SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.requires
	$(MAKE) -f SDLmain/CMakeFiles/SDLmain.dir/build.make SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.provides.build
.PHONY : SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.provides

SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.provides.build: SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o

# Object files for target SDLmain
SDLmain_OBJECTS = \
"CMakeFiles/SDLmain.dir/SDLmain.m.o"

# External object files for target SDLmain
SDLmain_EXTERNAL_OBJECTS =

SDLmain/libSDLmain.a: SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o
SDLmain/libSDLmain.a: SDLmain/CMakeFiles/SDLmain.dir/build.make
SDLmain/libSDLmain.a: SDLmain/CMakeFiles/SDLmain.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libSDLmain.a"
	cd /Users/louisroche/Code/graph/p5/build/SDLmain && $(CMAKE_COMMAND) -P CMakeFiles/SDLmain.dir/cmake_clean_target.cmake
	cd /Users/louisroche/Code/graph/p5/build/SDLmain && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SDLmain.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
SDLmain/CMakeFiles/SDLmain.dir/build: SDLmain/libSDLmain.a
.PHONY : SDLmain/CMakeFiles/SDLmain.dir/build

SDLmain/CMakeFiles/SDLmain.dir/requires: SDLmain/CMakeFiles/SDLmain.dir/SDLmain.m.o.requires
.PHONY : SDLmain/CMakeFiles/SDLmain.dir/requires

SDLmain/CMakeFiles/SDLmain.dir/clean:
	cd /Users/louisroche/Code/graph/p5/build/SDLmain && $(CMAKE_COMMAND) -P CMakeFiles/SDLmain.dir/cmake_clean.cmake
.PHONY : SDLmain/CMakeFiles/SDLmain.dir/clean

SDLmain/CMakeFiles/SDLmain.dir/depend:
	cd /Users/louisroche/Code/graph/p5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/louisroche/Code/graph/p5/src /Users/louisroche/Code/graph/p5/src/SDLmain /Users/louisroche/Code/graph/p5/build /Users/louisroche/Code/graph/p5/build/SDLmain /Users/louisroche/Code/graph/p5/build/SDLmain/CMakeFiles/SDLmain.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : SDLmain/CMakeFiles/SDLmain.dir/depend

