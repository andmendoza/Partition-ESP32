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
include CMakeFiles/Particiones.elf.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Particiones.elf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Particiones.elf.dir/flags.make

project_elf_src.c:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating project_elf_src.c"
	/usr/bin/cmake -E touch /home/ubuntu/Work/ANIOT/Particiones/build/project_elf_src.c

CMakeFiles/Particiones.elf.dir/project_elf_src.c.obj: CMakeFiles/Particiones.elf.dir/flags.make
CMakeFiles/Particiones.elf.dir/project_elf_src.c.obj: project_elf_src.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/Particiones.elf.dir/project_elf_src.c.obj"
	/home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/Particiones.elf.dir/project_elf_src.c.obj   -c /home/ubuntu/Work/ANIOT/Particiones/build/project_elf_src.c

CMakeFiles/Particiones.elf.dir/project_elf_src.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/Particiones.elf.dir/project_elf_src.c.i"
	/home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ubuntu/Work/ANIOT/Particiones/build/project_elf_src.c > CMakeFiles/Particiones.elf.dir/project_elf_src.c.i

CMakeFiles/Particiones.elf.dir/project_elf_src.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/Particiones.elf.dir/project_elf_src.c.s"
	/home/ubuntu/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ubuntu/Work/ANIOT/Particiones/build/project_elf_src.c -o CMakeFiles/Particiones.elf.dir/project_elf_src.c.s

# Object files for target Particiones.elf
Particiones_elf_OBJECTS = \
"CMakeFiles/Particiones.elf.dir/project_elf_src.c.obj"

# External object files for target Particiones.elf
Particiones_elf_EXTERNAL_OBJECTS =

Particiones.elf: CMakeFiles/Particiones.elf.dir/project_elf_src.c.obj
Particiones.elf: CMakeFiles/Particiones.elf.dir/build.make
Particiones.elf: esp-idf/xtensa/libxtensa.a
Particiones.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
Particiones.elf: esp-idf/app_update/libapp_update.a
Particiones.elf: esp-idf/spi_flash/libspi_flash.a
Particiones.elf: esp-idf/bootloader_support/libbootloader_support.a
Particiones.elf: esp-idf/efuse/libefuse.a
Particiones.elf: esp-idf/driver/libdriver.a
Particiones.elf: esp-idf/nvs_flash/libnvs_flash.a
Particiones.elf: esp-idf/pthread/libpthread.a
Particiones.elf: esp-idf/espcoredump/libespcoredump.a
Particiones.elf: esp-idf/perfmon/libperfmon.a
Particiones.elf: esp-idf/esp32/libesp32.a
Particiones.elf: esp-idf/esp_common/libesp_common.a
Particiones.elf: esp-idf/esp_rom/libesp_rom.a
Particiones.elf: esp-idf/soc/libsoc.a
Particiones.elf: esp-idf/esp_eth/libesp_eth.a
Particiones.elf: esp-idf/tcpip_adapter/libtcpip_adapter.a
Particiones.elf: esp-idf/esp_netif/libesp_netif.a
Particiones.elf: esp-idf/esp_event/libesp_event.a
Particiones.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
Particiones.elf: esp-idf/esp_wifi/libesp_wifi.a
Particiones.elf: esp-idf/lwip/liblwip.a
Particiones.elf: esp-idf/log/liblog.a
Particiones.elf: esp-idf/heap/libheap.a
Particiones.elf: esp-idf/freertos/libfreertos.a
Particiones.elf: esp-idf/vfs/libvfs.a
Particiones.elf: esp-idf/newlib/libnewlib.a
Particiones.elf: esp-idf/cxx/libcxx.a
Particiones.elf: esp-idf/app_trace/libapp_trace.a
Particiones.elf: esp-idf/asio/libasio.a
Particiones.elf: esp-idf/cbor/libcbor.a
Particiones.elf: esp-idf/coap/libcoap.a
Particiones.elf: esp-idf/console/libconsole.a
Particiones.elf: esp-idf/nghttp/libnghttp.a
Particiones.elf: esp-idf/esp-tls/libesp-tls.a
Particiones.elf: esp-idf/esp_adc_cal/libesp_adc_cal.a
Particiones.elf: esp-idf/esp_gdbstub/libesp_gdbstub.a
Particiones.elf: esp-idf/tcp_transport/libtcp_transport.a
Particiones.elf: esp-idf/esp_http_client/libesp_http_client.a
Particiones.elf: esp-idf/esp_http_server/libesp_http_server.a
Particiones.elf: esp-idf/esp_https_ota/libesp_https_ota.a
Particiones.elf: esp-idf/protobuf-c/libprotobuf-c.a
Particiones.elf: esp-idf/protocomm/libprotocomm.a
Particiones.elf: esp-idf/mdns/libmdns.a
Particiones.elf: esp-idf/esp_local_ctrl/libesp_local_ctrl.a
Particiones.elf: esp-idf/sdmmc/libsdmmc.a
Particiones.elf: esp-idf/esp_serial_slave_link/libesp_serial_slave_link.a
Particiones.elf: esp-idf/esp_websocket_client/libesp_websocket_client.a
Particiones.elf: esp-idf/expat/libexpat.a
Particiones.elf: esp-idf/wear_levelling/libwear_levelling.a
Particiones.elf: esp-idf/fatfs/libfatfs.a
Particiones.elf: esp-idf/freemodbus/libfreemodbus.a
Particiones.elf: esp-idf/jsmn/libjsmn.a
Particiones.elf: esp-idf/json/libjson.a
Particiones.elf: esp-idf/libsodium/liblibsodium.a
Particiones.elf: esp-idf/mqtt/libmqtt.a
Particiones.elf: esp-idf/openssl/libopenssl.a
Particiones.elf: esp-idf/spiffs/libspiffs.a
Particiones.elf: esp-idf/ulp/libulp.a
Particiones.elf: esp-idf/unity/libunity.a
Particiones.elf: esp-idf/wifi_provisioning/libwifi_provisioning.a
Particiones.elf: esp-idf/sdmmc/libsdmmc.a
Particiones.elf: esp-idf/wear_levelling/libwear_levelling.a
Particiones.elf: esp-idf/protocomm/libprotocomm.a
Particiones.elf: esp-idf/protobuf-c/libprotobuf-c.a
Particiones.elf: esp-idf/mdns/libmdns.a
Particiones.elf: esp-idf/console/libconsole.a
Particiones.elf: esp-idf/json/libjson.a
Particiones.elf: esp-idf/xtensa/libxtensa.a
Particiones.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
Particiones.elf: esp-idf/app_update/libapp_update.a
Particiones.elf: esp-idf/spi_flash/libspi_flash.a
Particiones.elf: esp-idf/bootloader_support/libbootloader_support.a
Particiones.elf: esp-idf/efuse/libefuse.a
Particiones.elf: esp-idf/driver/libdriver.a
Particiones.elf: esp-idf/nvs_flash/libnvs_flash.a
Particiones.elf: esp-idf/pthread/libpthread.a
Particiones.elf: esp-idf/espcoredump/libespcoredump.a
Particiones.elf: esp-idf/perfmon/libperfmon.a
Particiones.elf: esp-idf/esp32/libesp32.a
Particiones.elf: esp-idf/esp_common/libesp_common.a
Particiones.elf: esp-idf/esp_rom/libesp_rom.a
Particiones.elf: esp-idf/soc/libsoc.a
Particiones.elf: esp-idf/esp_eth/libesp_eth.a
Particiones.elf: esp-idf/tcpip_adapter/libtcpip_adapter.a
Particiones.elf: esp-idf/esp_netif/libesp_netif.a
Particiones.elf: esp-idf/esp_event/libesp_event.a
Particiones.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
Particiones.elf: esp-idf/esp_wifi/libesp_wifi.a
Particiones.elf: esp-idf/lwip/liblwip.a
Particiones.elf: esp-idf/log/liblog.a
Particiones.elf: esp-idf/heap/libheap.a
Particiones.elf: esp-idf/freertos/libfreertos.a
Particiones.elf: esp-idf/vfs/libvfs.a
Particiones.elf: esp-idf/newlib/libnewlib.a
Particiones.elf: esp-idf/cxx/libcxx.a
Particiones.elf: esp-idf/app_trace/libapp_trace.a
Particiones.elf: esp-idf/nghttp/libnghttp.a
Particiones.elf: esp-idf/esp-tls/libesp-tls.a
Particiones.elf: esp-idf/tcp_transport/libtcp_transport.a
Particiones.elf: esp-idf/esp_http_client/libesp_http_client.a
Particiones.elf: esp-idf/esp_http_server/libesp_http_server.a
Particiones.elf: esp-idf/ulp/libulp.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcoexist.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcore.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libespnow.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libmesh.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libnet80211.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libpp.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/librtc.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libsmartconfig.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libphy.a
Particiones.elf: esp-idf/xtensa/libxtensa.a
Particiones.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
Particiones.elf: esp-idf/app_update/libapp_update.a
Particiones.elf: esp-idf/spi_flash/libspi_flash.a
Particiones.elf: esp-idf/bootloader_support/libbootloader_support.a
Particiones.elf: esp-idf/efuse/libefuse.a
Particiones.elf: esp-idf/driver/libdriver.a
Particiones.elf: esp-idf/nvs_flash/libnvs_flash.a
Particiones.elf: esp-idf/pthread/libpthread.a
Particiones.elf: esp-idf/espcoredump/libespcoredump.a
Particiones.elf: esp-idf/perfmon/libperfmon.a
Particiones.elf: esp-idf/esp32/libesp32.a
Particiones.elf: esp-idf/esp_common/libesp_common.a
Particiones.elf: esp-idf/esp_rom/libesp_rom.a
Particiones.elf: esp-idf/soc/libsoc.a
Particiones.elf: esp-idf/esp_eth/libesp_eth.a
Particiones.elf: esp-idf/tcpip_adapter/libtcpip_adapter.a
Particiones.elf: esp-idf/esp_netif/libesp_netif.a
Particiones.elf: esp-idf/esp_event/libesp_event.a
Particiones.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
Particiones.elf: esp-idf/esp_wifi/libesp_wifi.a
Particiones.elf: esp-idf/lwip/liblwip.a
Particiones.elf: esp-idf/log/liblog.a
Particiones.elf: esp-idf/heap/libheap.a
Particiones.elf: esp-idf/freertos/libfreertos.a
Particiones.elf: esp-idf/vfs/libvfs.a
Particiones.elf: esp-idf/newlib/libnewlib.a
Particiones.elf: esp-idf/cxx/libcxx.a
Particiones.elf: esp-idf/app_trace/libapp_trace.a
Particiones.elf: esp-idf/nghttp/libnghttp.a
Particiones.elf: esp-idf/esp-tls/libesp-tls.a
Particiones.elf: esp-idf/tcp_transport/libtcp_transport.a
Particiones.elf: esp-idf/esp_http_client/libesp_http_client.a
Particiones.elf: esp-idf/esp_http_server/libesp_http_server.a
Particiones.elf: esp-idf/ulp/libulp.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcoexist.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcore.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libespnow.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libmesh.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libnet80211.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libpp.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/librtc.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libsmartconfig.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libphy.a
Particiones.elf: esp-idf/xtensa/libxtensa.a
Particiones.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
Particiones.elf: esp-idf/app_update/libapp_update.a
Particiones.elf: esp-idf/spi_flash/libspi_flash.a
Particiones.elf: esp-idf/bootloader_support/libbootloader_support.a
Particiones.elf: esp-idf/efuse/libefuse.a
Particiones.elf: esp-idf/driver/libdriver.a
Particiones.elf: esp-idf/nvs_flash/libnvs_flash.a
Particiones.elf: esp-idf/pthread/libpthread.a
Particiones.elf: esp-idf/espcoredump/libespcoredump.a
Particiones.elf: esp-idf/perfmon/libperfmon.a
Particiones.elf: esp-idf/esp32/libesp32.a
Particiones.elf: esp-idf/esp_common/libesp_common.a
Particiones.elf: esp-idf/esp_rom/libesp_rom.a
Particiones.elf: esp-idf/soc/libsoc.a
Particiones.elf: esp-idf/esp_eth/libesp_eth.a
Particiones.elf: esp-idf/tcpip_adapter/libtcpip_adapter.a
Particiones.elf: esp-idf/esp_netif/libesp_netif.a
Particiones.elf: esp-idf/esp_event/libesp_event.a
Particiones.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
Particiones.elf: esp-idf/esp_wifi/libesp_wifi.a
Particiones.elf: esp-idf/lwip/liblwip.a
Particiones.elf: esp-idf/log/liblog.a
Particiones.elf: esp-idf/heap/libheap.a
Particiones.elf: esp-idf/freertos/libfreertos.a
Particiones.elf: esp-idf/vfs/libvfs.a
Particiones.elf: esp-idf/newlib/libnewlib.a
Particiones.elf: esp-idf/cxx/libcxx.a
Particiones.elf: esp-idf/app_trace/libapp_trace.a
Particiones.elf: esp-idf/nghttp/libnghttp.a
Particiones.elf: esp-idf/esp-tls/libesp-tls.a
Particiones.elf: esp-idf/tcp_transport/libtcp_transport.a
Particiones.elf: esp-idf/esp_http_client/libesp_http_client.a
Particiones.elf: esp-idf/esp_http_server/libesp_http_server.a
Particiones.elf: esp-idf/ulp/libulp.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcoexist.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcore.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libespnow.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libmesh.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libnet80211.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libpp.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/librtc.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libsmartconfig.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libphy.a
Particiones.elf: esp-idf/xtensa/libxtensa.a
Particiones.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
Particiones.elf: esp-idf/app_update/libapp_update.a
Particiones.elf: esp-idf/spi_flash/libspi_flash.a
Particiones.elf: esp-idf/bootloader_support/libbootloader_support.a
Particiones.elf: esp-idf/efuse/libefuse.a
Particiones.elf: esp-idf/driver/libdriver.a
Particiones.elf: esp-idf/nvs_flash/libnvs_flash.a
Particiones.elf: esp-idf/pthread/libpthread.a
Particiones.elf: esp-idf/espcoredump/libespcoredump.a
Particiones.elf: esp-idf/perfmon/libperfmon.a
Particiones.elf: esp-idf/esp32/libesp32.a
Particiones.elf: esp-idf/esp_common/libesp_common.a
Particiones.elf: esp-idf/esp_rom/libesp_rom.a
Particiones.elf: esp-idf/soc/libsoc.a
Particiones.elf: esp-idf/esp_eth/libesp_eth.a
Particiones.elf: esp-idf/tcpip_adapter/libtcpip_adapter.a
Particiones.elf: esp-idf/esp_netif/libesp_netif.a
Particiones.elf: esp-idf/esp_event/libesp_event.a
Particiones.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
Particiones.elf: esp-idf/esp_wifi/libesp_wifi.a
Particiones.elf: esp-idf/lwip/liblwip.a
Particiones.elf: esp-idf/log/liblog.a
Particiones.elf: esp-idf/heap/libheap.a
Particiones.elf: esp-idf/freertos/libfreertos.a
Particiones.elf: esp-idf/vfs/libvfs.a
Particiones.elf: esp-idf/newlib/libnewlib.a
Particiones.elf: esp-idf/cxx/libcxx.a
Particiones.elf: esp-idf/app_trace/libapp_trace.a
Particiones.elf: esp-idf/nghttp/libnghttp.a
Particiones.elf: esp-idf/esp-tls/libesp-tls.a
Particiones.elf: esp-idf/tcp_transport/libtcp_transport.a
Particiones.elf: esp-idf/esp_http_client/libesp_http_client.a
Particiones.elf: esp-idf/esp_http_server/libesp_http_server.a
Particiones.elf: esp-idf/ulp/libulp.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
Particiones.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcoexist.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libcore.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libespnow.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libmesh.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libnet80211.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libpp.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/librtc.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libsmartconfig.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_wifi/lib/esp32/libphy.a
Particiones.elf: /home/ubuntu/esp/esp-idf/components/xtensa/esp32/libhal.a
Particiones.elf: esp-idf/newlib/libnewlib.a
Particiones.elf: esp-idf/pthread/libpthread.a
Particiones.elf: esp-idf/cxx/libcxx.a
Particiones.elf: esp-idf/esp32/esp32_out.ld
Particiones.elf: esp-idf/esp32/ld/esp32.project.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp32/ld/esp32.peripherals.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_rom/esp32/ld/esp32.rom.newlib-time.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_rom/esp32/ld/esp32.rom.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_rom/esp32/ld/esp32.rom.libgcc.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_rom/esp32/ld/esp32.rom.newlib-data.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_rom/esp32/ld/esp32.rom.syscalls.ld
Particiones.elf: /home/ubuntu/esp/esp-idf/components/esp_rom/esp32/ld/esp32.rom.newlib-funcs.ld
Particiones.elf: CMakeFiles/Particiones.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Particiones.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Particiones.elf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Particiones.elf.dir/build: Particiones.elf

.PHONY : CMakeFiles/Particiones.elf.dir/build

CMakeFiles/Particiones.elf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Particiones.elf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Particiones.elf.dir/clean

CMakeFiles/Particiones.elf.dir/depend: project_elf_src.c
	cd /home/ubuntu/Work/ANIOT/Particiones/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Work/ANIOT/Particiones /home/ubuntu/Work/ANIOT/Particiones /home/ubuntu/Work/ANIOT/Particiones/build /home/ubuntu/Work/ANIOT/Particiones/build /home/ubuntu/Work/ANIOT/Particiones/build/CMakeFiles/Particiones.elf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Particiones.elf.dir/depend

