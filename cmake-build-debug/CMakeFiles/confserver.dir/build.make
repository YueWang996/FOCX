# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2021.2.3\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2021.2.3\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\AllFiles\programming\MachineDog\FocX_v_0_5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\AllFiles\programming\MachineDog\FocX_v_0_5\cmake-build-debug

# Utility rule file for confserver.

# Include any custom commands dependencies for this target.
include CMakeFiles/confserver.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/confserver.dir/progress.make

CMakeFiles/confserver:
	python C:/Users/Justin/esp/esp-idf/tools/kconfig_new/prepare_kconfig_files.py --env-file C:/AllFiles/programming/MachineDog/FocX_v_0_5/cmake-build-debug/config.env
	python C:/Users/Justin/esp/esp-idf/tools/kconfig_new/confserver.py --env-file C:/AllFiles/programming/MachineDog/FocX_v_0_5/cmake-build-debug/config.env --kconfig C:/Users/Justin/esp/esp-idf/Kconfig --sdkconfig-rename C:/Users/Justin/esp/esp-idf/sdkconfig.rename --config C:/AllFiles/programming/MachineDog/FocX_v_0_5/sdkconfig

confserver: CMakeFiles/confserver
confserver: CMakeFiles/confserver.dir/build.make
.PHONY : confserver

# Rule to build all files generated by this target.
CMakeFiles/confserver.dir/build: confserver
.PHONY : CMakeFiles/confserver.dir/build

CMakeFiles/confserver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\confserver.dir\cmake_clean.cmake
.PHONY : CMakeFiles/confserver.dir/clean

CMakeFiles/confserver.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\AllFiles\programming\MachineDog\FocX_v_0_5 C:\AllFiles\programming\MachineDog\FocX_v_0_5 C:\AllFiles\programming\MachineDog\FocX_v_0_5\cmake-build-debug C:\AllFiles\programming\MachineDog\FocX_v_0_5\cmake-build-debug C:\AllFiles\programming\MachineDog\FocX_v_0_5\cmake-build-debug\CMakeFiles\confserver.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/confserver.dir/depend

