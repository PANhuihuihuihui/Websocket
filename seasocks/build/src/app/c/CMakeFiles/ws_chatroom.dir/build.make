# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/pi/Desktop/seasocks

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/seasocks/build

# Include any dependencies generated for this target.
include src/app/c/CMakeFiles/ws_chatroom.dir/depend.make

# Include the progress variables for this target.
include src/app/c/CMakeFiles/ws_chatroom.dir/progress.make

# Include the compile flags for this target's objects.
include src/app/c/CMakeFiles/ws_chatroom.dir/flags.make

src/app/c/CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.o: src/app/c/CMakeFiles/ws_chatroom.dir/flags.make
src/app/c/CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.o: ../src/app/c/ws_chatroom.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/app/c/CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.o"
	cd /home/pi/Desktop/seasocks/build/src/app/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.o -c /home/pi/Desktop/seasocks/src/app/c/ws_chatroom.cpp

src/app/c/CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.i"
	cd /home/pi/Desktop/seasocks/build/src/app/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/seasocks/src/app/c/ws_chatroom.cpp > CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.i

src/app/c/CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.s"
	cd /home/pi/Desktop/seasocks/build/src/app/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/seasocks/src/app/c/ws_chatroom.cpp -o CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.s

# Object files for target ws_chatroom
ws_chatroom_OBJECTS = \
"CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.o"

# External object files for target ws_chatroom
ws_chatroom_EXTERNAL_OBJECTS =

src/app/c/ws_chatroom: src/app/c/CMakeFiles/ws_chatroom.dir/ws_chatroom.cpp.o
src/app/c/ws_chatroom: src/app/c/CMakeFiles/ws_chatroom.dir/build.make
src/app/c/ws_chatroom: src/main/c/libseasocks.a
src/app/c/ws_chatroom: /usr/lib/arm-linux-gnueabihf/libz.so
src/app/c/ws_chatroom: src/app/c/CMakeFiles/ws_chatroom.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ws_chatroom"
	cd /home/pi/Desktop/seasocks/build/src/app/c && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ws_chatroom.dir/link.txt --verbose=$(VERBOSE)
	cd /home/pi/Desktop/seasocks/build/src/app/c && /usr/bin/cmake -E copy_directory /home/pi/Desktop/seasocks/src/ws_chatroom_web /home/pi/Desktop/seasocks/build/src/app/c/src/ws_chatroom_web

# Rule to build all files generated by this target.
src/app/c/CMakeFiles/ws_chatroom.dir/build: src/app/c/ws_chatroom

.PHONY : src/app/c/CMakeFiles/ws_chatroom.dir/build

src/app/c/CMakeFiles/ws_chatroom.dir/clean:
	cd /home/pi/Desktop/seasocks/build/src/app/c && $(CMAKE_COMMAND) -P CMakeFiles/ws_chatroom.dir/cmake_clean.cmake
.PHONY : src/app/c/CMakeFiles/ws_chatroom.dir/clean

src/app/c/CMakeFiles/ws_chatroom.dir/depend:
	cd /home/pi/Desktop/seasocks/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/seasocks /home/pi/Desktop/seasocks/src/app/c /home/pi/Desktop/seasocks/build /home/pi/Desktop/seasocks/build/src/app/c /home/pi/Desktop/seasocks/build/src/app/c/CMakeFiles/ws_chatroom.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/app/c/CMakeFiles/ws_chatroom.dir/depend

