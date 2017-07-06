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
project(main C)

#set src
SET (SRC 
     ${CMAKE_SOURCE_DIR}/main.c 
     )

add_definitions (
)

add_compile_options(
 -std=gnu99
 -mcpu=cortex-a53
 -mfpu=vfp
 -mfloat-abi=hard
 -Wall -O3 -g3
)

include_directories ( 
 ${CMAKE_SOURCE_DIR}/bme280/inc
 ${CMAKE_SOURCE_DIR}/raspberry_util/inc
)

add_executable(${PROJECT_NAME} ${SRC})

set_target_properties( ${PROJECT_NAME}
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

link_directories(/usr/local/lib)

target_link_libraries(${PROJECT_NAME} raspberryutil bme280)
