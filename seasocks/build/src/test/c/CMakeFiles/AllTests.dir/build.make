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
CMAKE_SOURCE_DIR = /home/pi/Desktop/Websocket/seasocks

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/Websocket/seasocks/build

# Include any dependencies generated for this target.
include src/test/c/CMakeFiles/AllTests.dir/depend.make

# Include the progress variables for this target.
include src/test/c/CMakeFiles/AllTests.dir/progress.make

# Include the compile flags for this target's objects.
include src/test/c/CMakeFiles/AllTests.dir/flags.make

src/test/c/CMakeFiles/AllTests.dir/test_main.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/test_main.cpp.o: ../src/test/c/test_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/test_main.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/test_main.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/test_main.cpp

src/test/c/CMakeFiles/AllTests.dir/test_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/test_main.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/test_main.cpp > CMakeFiles/AllTests.dir/test_main.cpp.i

src/test/c/CMakeFiles/AllTests.dir/test_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/test_main.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/test_main.cpp -o CMakeFiles/AllTests.dir/test_main.cpp.s

src/test/c/CMakeFiles/AllTests.dir/ConnectionTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/ConnectionTests.cpp.o: ../src/test/c/ConnectionTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/ConnectionTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/ConnectionTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/ConnectionTests.cpp

src/test/c/CMakeFiles/AllTests.dir/ConnectionTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/ConnectionTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/ConnectionTests.cpp > CMakeFiles/AllTests.dir/ConnectionTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/ConnectionTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/ConnectionTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/ConnectionTests.cpp -o CMakeFiles/AllTests.dir/ConnectionTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/CrackedUriTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/CrackedUriTests.cpp.o: ../src/test/c/CrackedUriTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/CrackedUriTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/CrackedUriTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/CrackedUriTests.cpp

src/test/c/CMakeFiles/AllTests.dir/CrackedUriTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/CrackedUriTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/CrackedUriTests.cpp > CMakeFiles/AllTests.dir/CrackedUriTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/CrackedUriTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/CrackedUriTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/CrackedUriTests.cpp -o CMakeFiles/AllTests.dir/CrackedUriTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/HeaderMapTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/HeaderMapTests.cpp.o: ../src/test/c/HeaderMapTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/HeaderMapTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/HeaderMapTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/HeaderMapTests.cpp

src/test/c/CMakeFiles/AllTests.dir/HeaderMapTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/HeaderMapTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/HeaderMapTests.cpp > CMakeFiles/AllTests.dir/HeaderMapTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/HeaderMapTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/HeaderMapTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/HeaderMapTests.cpp -o CMakeFiles/AllTests.dir/HeaderMapTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/HtmlTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/HtmlTests.cpp.o: ../src/test/c/HtmlTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/HtmlTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/HtmlTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/HtmlTests.cpp

src/test/c/CMakeFiles/AllTests.dir/HtmlTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/HtmlTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/HtmlTests.cpp > CMakeFiles/AllTests.dir/HtmlTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/HtmlTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/HtmlTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/HtmlTests.cpp -o CMakeFiles/AllTests.dir/HtmlTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/HybiTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/HybiTests.cpp.o: ../src/test/c/HybiTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/HybiTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/HybiTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/HybiTests.cpp

src/test/c/CMakeFiles/AllTests.dir/HybiTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/HybiTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/HybiTests.cpp > CMakeFiles/AllTests.dir/HybiTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/HybiTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/HybiTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/HybiTests.cpp -o CMakeFiles/AllTests.dir/HybiTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/JsonTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/JsonTests.cpp.o: ../src/test/c/JsonTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/JsonTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/JsonTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/JsonTests.cpp

src/test/c/CMakeFiles/AllTests.dir/JsonTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/JsonTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/JsonTests.cpp > CMakeFiles/AllTests.dir/JsonTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/JsonTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/JsonTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/JsonTests.cpp -o CMakeFiles/AllTests.dir/JsonTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/ServerTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/ServerTests.cpp.o: ../src/test/c/ServerTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/ServerTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/ServerTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/ServerTests.cpp

src/test/c/CMakeFiles/AllTests.dir/ServerTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/ServerTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/ServerTests.cpp > CMakeFiles/AllTests.dir/ServerTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/ServerTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/ServerTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/ServerTests.cpp -o CMakeFiles/AllTests.dir/ServerTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/ToStringTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/ToStringTests.cpp.o: ../src/test/c/ToStringTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/ToStringTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/ToStringTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/ToStringTests.cpp

src/test/c/CMakeFiles/AllTests.dir/ToStringTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/ToStringTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/ToStringTests.cpp > CMakeFiles/AllTests.dir/ToStringTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/ToStringTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/ToStringTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/ToStringTests.cpp -o CMakeFiles/AllTests.dir/ToStringTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.o: ../src/test/c/EmbeddedContentTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/EmbeddedContentTests.cpp

src/test/c/CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/EmbeddedContentTests.cpp > CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/EmbeddedContentTests.cpp -o CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.o: ../src/test/c/ResponseBuilderTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/ResponseBuilderTests.cpp

src/test/c/CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/ResponseBuilderTests.cpp > CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/ResponseBuilderTests.cpp -o CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/ResponseTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/ResponseTests.cpp.o: ../src/test/c/ResponseTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/ResponseTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/ResponseTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/ResponseTests.cpp

src/test/c/CMakeFiles/AllTests.dir/ResponseTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/ResponseTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/ResponseTests.cpp > CMakeFiles/AllTests.dir/ResponseTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/ResponseTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/ResponseTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/ResponseTests.cpp -o CMakeFiles/AllTests.dir/ResponseTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/StringUtilTests.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/StringUtilTests.cpp.o: ../src/test/c/StringUtilTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/StringUtilTests.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/StringUtilTests.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/StringUtilTests.cpp

src/test/c/CMakeFiles/AllTests.dir/StringUtilTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/StringUtilTests.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/StringUtilTests.cpp > CMakeFiles/AllTests.dir/StringUtilTests.cpp.i

src/test/c/CMakeFiles/AllTests.dir/StringUtilTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/StringUtilTests.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/StringUtilTests.cpp -o CMakeFiles/AllTests.dir/StringUtilTests.cpp.s

src/test/c/CMakeFiles/AllTests.dir/RequestTest.cpp.o: src/test/c/CMakeFiles/AllTests.dir/flags.make
src/test/c/CMakeFiles/AllTests.dir/RequestTest.cpp.o: ../src/test/c/RequestTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object src/test/c/CMakeFiles/AllTests.dir/RequestTest.cpp.o"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AllTests.dir/RequestTest.cpp.o -c /home/pi/Desktop/Websocket/seasocks/src/test/c/RequestTest.cpp

src/test/c/CMakeFiles/AllTests.dir/RequestTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AllTests.dir/RequestTest.cpp.i"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/Websocket/seasocks/src/test/c/RequestTest.cpp > CMakeFiles/AllTests.dir/RequestTest.cpp.i

src/test/c/CMakeFiles/AllTests.dir/RequestTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AllTests.dir/RequestTest.cpp.s"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/Websocket/seasocks/src/test/c/RequestTest.cpp -o CMakeFiles/AllTests.dir/RequestTest.cpp.s

# Object files for target AllTests
AllTests_OBJECTS = \
"CMakeFiles/AllTests.dir/test_main.cpp.o" \
"CMakeFiles/AllTests.dir/ConnectionTests.cpp.o" \
"CMakeFiles/AllTests.dir/CrackedUriTests.cpp.o" \
"CMakeFiles/AllTests.dir/HeaderMapTests.cpp.o" \
"CMakeFiles/AllTests.dir/HtmlTests.cpp.o" \
"CMakeFiles/AllTests.dir/HybiTests.cpp.o" \
"CMakeFiles/AllTests.dir/JsonTests.cpp.o" \
"CMakeFiles/AllTests.dir/ServerTests.cpp.o" \
"CMakeFiles/AllTests.dir/ToStringTests.cpp.o" \
"CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.o" \
"CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.o" \
"CMakeFiles/AllTests.dir/ResponseTests.cpp.o" \
"CMakeFiles/AllTests.dir/StringUtilTests.cpp.o" \
"CMakeFiles/AllTests.dir/RequestTest.cpp.o"

# External object files for target AllTests
AllTests_EXTERNAL_OBJECTS =

src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/test_main.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/ConnectionTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/CrackedUriTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/HeaderMapTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/HtmlTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/HybiTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/JsonTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/ServerTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/ToStringTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/EmbeddedContentTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/ResponseBuilderTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/ResponseTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/StringUtilTests.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/RequestTest.cpp.o
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/build.make
src/test/c/AllTests: src/main/c/libseasocks.a
src/test/c/AllTests: /usr/lib/arm-linux-gnueabihf/libz.so
src/test/c/AllTests: src/test/c/CMakeFiles/AllTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/Websocket/seasocks/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX executable AllTests"
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AllTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/test/c/CMakeFiles/AllTests.dir/build: src/test/c/AllTests

.PHONY : src/test/c/CMakeFiles/AllTests.dir/build

src/test/c/CMakeFiles/AllTests.dir/clean:
	cd /home/pi/Desktop/Websocket/seasocks/build/src/test/c && $(CMAKE_COMMAND) -P CMakeFiles/AllTests.dir/cmake_clean.cmake
.PHONY : src/test/c/CMakeFiles/AllTests.dir/clean

src/test/c/CMakeFiles/AllTests.dir/depend:
	cd /home/pi/Desktop/Websocket/seasocks/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/Websocket/seasocks /home/pi/Desktop/Websocket/seasocks/src/test/c /home/pi/Desktop/Websocket/seasocks/build /home/pi/Desktop/Websocket/seasocks/build/src/test/c /home/pi/Desktop/Websocket/seasocks/build/src/test/c/CMakeFiles/AllTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/test/c/CMakeFiles/AllTests.dir/depend

