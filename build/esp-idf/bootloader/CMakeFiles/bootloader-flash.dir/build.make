# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ubuntu/Work/ANIOT/Particiones

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Work/ANIOT/Particiones/build

# Utility rule file for bootloader-flash.

# Include the progress variables for this target.
include esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/progress.make

esp-idf/bootloader/CMakeFiles/bootloader-flash:
	cd /home/ubuntu/esp/esp-idf/components/bootloader && /usr/bin/cmake -D IDF_PATH="/home/ubuntu/esp/esp-idf" -D ESPTOOLPY="/home/ubuntu/.espressif/python_env/idf4.1_py3.8_env/bin/python /home/ubuntu/esp/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32" -D ESPTOOL_ARGS="write_flash @flash_bootloader_args" -D WORKING_DIRECTORY="/home/ubuntu/Work/ANIOT/Particiones/build" -P /home/ubuntu/esp/esp-idf/components/esptool_py/run_esptool.cmake

bootloader-flash: esp-idf/bootloader/CMakeFiles/bootloader-flash
bootloader-flash: esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/build.make

.PHONY : bootloader-flash

# Rule to build all files generated by this target.
esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/build: bootloader-flash

.PHONY : esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/build

esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/clean:
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/bootloader && $(CMAKE_COMMAND) -P CMakeFiles/bootloader-flash.dir/cmake_clean.cmake
.PHONY : esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/clean

esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/depend:
	cd /home/ubuntu/Work/ANIOT/Particiones/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Work/ANIOT/Particiones /home/ubuntu/esp/esp-idf/components/bootloader /home/ubuntu/Work/ANIOT/Particiones/build /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/bootloader /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/bootloader/CMakeFiles/bootloader-flash.dir/depend

