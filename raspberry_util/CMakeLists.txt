cmake_minimum_required (VERSION 2.6)

########################
#set for toolchain
########################
set (tools_chain_path /usr/bin)
set (tools_chain_prefix arm-linux-gnueabihf)
set (CMAKE_C_COMPILER ${tools_chain_path}/${tools_chain_prefix}-gcc)

########################
#Common Variables
########################
project(raspberryutil C)

#set src
SET (SRC src/raspberry_util.c)

add_definitions (
 -DRASPBERRY_DEBUG_ENABLE
)

add_compile_options(
 -std=c99
 -mcpu=cortex-a53
 -mfpu=vfp
 -mfloat-abi=hard
 -Wall -O3 -g3
)

include_directories (./inc)

add_library(${PROJECT_NAME} SHARED ${SRC})

link_directories ( /usr/local/lib )

target_link_libraries(${PROJECT_NAME} bcm2835)

set_target_properties( ${PROJECT_NAME}
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)
