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
CMAKE_SOURCE_DIR = C:\AllFiles\programming\MachineDog\FocX_v_0_5_5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\AllFiles\programming\MachineDog\FocX_v_0_5_5\cmake-build-debug

# Utility rule file for menuconfig.

# Include any custom commands dependencies for this target.
include CMakeFiles/menuconfig.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/menuconfig.dir/progress.make

CMakeFiles/menuconfig:
	python C:/Users/Justin/esp/esp-idf/tools/kconfig_new/prepare_kconfig_files.py --env-file C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/cmake-build-debug/config.env
	python C:/Users/Justin/esp/esp-idf/tools/kconfig_new/confgen.py --kconfig C:/Users/Justin/esp/esp-idf/Kconfig --sdkconfig-rename C:/Users/Justin/esp/esp-idf/sdkconfig.rename --config C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/sdkconfig --env-file C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/cmake-build-debug/config.env --env IDF_TARGET=esp32 --env IDF_ENV_FPGA= --dont-write-deprecated --output config C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/sdkconfig
	python C:/Users/Justin/esp/esp-idf/tools/check_term.py
	"C:\Program Files\JetBrains\CLion 2021.2.3\bin\cmake\win\bin\cmake.exe" -E env COMPONENT_KCONFIGS_SOURCE_FILE=C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/cmake-build-debug/kconfigs.in COMPONENT_KCONFIGS_PROJBUILD_SOURCE_FILE=C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/cmake-build-debug/kconfigs_projbuild.in IDF_CMAKE=y KCONFIG_CONFIG=C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/sdkconfig IDF_TARGET=esp32 IDF_ENV_FPGA= python -m menuconfig C:/Users/Justin/esp/esp-idf/Kconfig
	python C:/Users/Justin/esp/esp-idf/tools/kconfig_new/confgen.py --kconfig C:/Users/Justin/esp/esp-idf/Kconfig --sdkconfig-rename C:/Users/Justin/esp/esp-idf/sdkconfig.rename --config C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/sdkconfig --env-file C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/cmake-build-debug/config.env --env IDF_TARGET=esp32 --env IDF_ENV_FPGA= --output config C:/AllFiles/programming/MachineDog/FocX_v_0_5_5/sdkconfig

menuconfig: CMakeFiles/menuconfig
menuconfig: CMakeFiles/menuconfig.dir/build.make
.PHONY : menuconfig

# Rule to build all files generated by this target.
CMakeFiles/menuconfig.dir/build: menuconfig
.PHONY : CMakeFiles/menuconfig.dir/build

CMakeFiles/menuconfig.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\menuconfig.dir\cmake_clean.cmake
.PHONY : CMakeFiles/menuconfig.dir/clean

CMakeFiles/menuconfig.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\AllFiles\programming\MachineDog\FocX_v_0_5_5 C:\AllFiles\programming\MachineDog\FocX_v_0_5_5 C:\AllFiles\programming\MachineDog\FocX_v_0_5_5\cmake-build-debug C:\AllFiles\programming\MachineDog\FocX_v_0_5_5\cmake-build-debug C:\AllFiles\programming\MachineDog\FocX_v_0_5_5\cmake-build-debug\CMakeFiles\menuconfig.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/menuconfig.dir/depend

