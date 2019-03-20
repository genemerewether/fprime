cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "mereweth@jpl.nasa.gov")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../../dspal/cmake_hexagon")
include(qurt_flags)
include(bundle)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-gnu-array-member-paren-init -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing") 

include_directories(
	../../dspal/include
	../../
	)

QURT_BUNDLE(APP_NAME dsp_relay
    DSP_SOURCES
	gpio_relay.c
	uart_relay.c
	spi_relay.c
    DSP_INCS
	./
    APPS_SOURCES
        test/ut/dsp_relay_main.c
    APPS_INCS
	./
    APPS_COMPILER arm-linux-gnueabihf-gcc
	)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
