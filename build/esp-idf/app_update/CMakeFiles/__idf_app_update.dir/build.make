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

# Include any dependencies generated for this target.
include esp-idf/app_update/CMakeFiles/__idf_app_update.dir/depend.make

# Include the progress variables for this target.
include esp-idf/app_update/CMakeFiles/__idf_app_update.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/app_update/CMakeFiles/__idf_app_update.dir/flags.make

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.obj: esp-idf/app_update/CMakeFiles/__idf_app_update.dir/flags.make
esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.obj: /home/ubuntu/esp/esp-idf/components/app_update/esp_ota_ops.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.obj"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && /home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.obj   -c /home/ubuntu/esp/esp-idf/components/app_update/esp_ota_ops.c

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.i"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && /home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ubuntu/esp/esp-idf/components/app_update/esp_ota_ops.c > CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.i

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.s"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && /home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ubuntu/esp/esp-idf/components/app_update/esp_ota_ops.c -o CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.s

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_app_desc.c.obj: esp-idf/app_update/CMakeFiles/__idf_app_update.dir/flags.make
esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_app_desc.c.obj: /home/ubuntu/esp/esp-idf/components/app_update/esp_app_desc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_app_desc.c.obj"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && /home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) -D PROJECT_NAME=\"Particiones\" -DPROJECT_VER=\"1\" $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/__idf_app_update.dir/esp_app_desc.c.obj   -c /home/ubuntu/esp/esp-idf/components/app_update/esp_app_desc.c

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_app_desc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_app_update.dir/esp_app_desc.c.i"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && /home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) -D PROJECT_NAME=\"Particiones\" -DPROJECT_VER=\"1\" $(C_INCLUDES) $(C_FLAGS) -E /home/ubuntu/esp/esp-idf/components/app_update/esp_app_desc.c > CMakeFiles/__idf_app_update.dir/esp_app_desc.c.i

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_app_desc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_app_update.dir/esp_app_desc.c.s"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && /home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) -D PROJECT_NAME=\"Particiones\" -DPROJECT_VER=\"1\" $(C_INCLUDES) $(C_FLAGS) -S /home/ubuntu/esp/esp-idf/components/app_update/esp_app_desc.c -o CMakeFiles/__idf_app_update.dir/esp_app_desc.c.s

# Object files for target __idf_app_update
__idf_app_update_OBJECTS = \
"CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.obj" \
"CMakeFiles/__idf_app_update.dir/esp_app_desc.c.obj"

# External object files for target __idf_app_update
__idf_app_update_EXTERNAL_OBJECTS =

esp-idf/app_update/libapp_update.a: esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_ota_ops.c.obj
esp-idf/app_update/libapp_update.a: esp-idf/app_update/CMakeFiles/__idf_app_update.dir/esp_app_desc.c.obj
esp-idf/app_update/libapp_update.a: esp-idf/app_update/CMakeFiles/__idf_app_update.dir/build.make
esp-idf/app_update/libapp_update.a: esp-idf/app_update/CMakeFiles/__idf_app_update.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libapp_update.a"
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && $(CMAKE_COMMAND) -P CMakeFiles/__idf_app_update.dir/cmake_clean_target.cmake
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_app_update.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/app_update/CMakeFiles/__idf_app_update.dir/build: esp-idf/app_update/libapp_update.a

.PHONY : esp-idf/app_update/CMakeFiles/__idf_app_update.dir/build

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/clean:
	cd /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update && $(CMAKE_COMMAND) -P CMakeFiles/__idf_app_update.dir/cmake_clean.cmake
.PHONY : esp-idf/app_update/CMakeFiles/__idf_app_update.dir/clean

esp-idf/app_update/CMakeFiles/__idf_app_update.dir/depend:
	cd /home/ubuntu/Work/ANIOT/Particiones/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Work/ANIOT/Particiones /home/ubuntu/esp/esp-idf/components/app_update /home/ubuntu/Work/ANIOT/Particiones/build /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update /home/ubuntu/Work/ANIOT/Particiones/build/esp-idf/app_update/CMakeFiles/__idf_app_update.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/app_update/CMakeFiles/__idf_app_update.dir/depend

