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
project(bme280 C)

#set src
SET (SRC 
     ${CMAKE_SOURCE_DIR}/src/bme280_app.c 
     ${CMAKE_SOURCE_DIR}/src/bme280_drv.c 
     )

add_definitions (
# -DBME280_DRV_ENABLE_FLOAT 
# -DBME280_ENABLE_INT64 
# -DBME280_ENABLE_BUS_CONNECT_SPI_4W 
 -DBME280_ENABLE_BUS_CONNECT_SPI_3W 
# -DBME280_ENABLE_BUS_CONNECT_I2C_SDO_GND 
# -DBME280_ENABLE_BUS_CONNECT_I2C_SDO_VDD
  -DBME280_DEBUG_ENABLE -DBME280_DEBUG_KERNEL
)

add_compile_options(
 -std=c99
 -mcpu=cortex-a53
 -mfpu=vfp
 -mfloat-abi=hard
 -Wall -O3 -g3
)

include_directories ( 
 ${CMAKE_SOURCE_DIR}/inc
)

add_library(${PROJECT_NAME} STATIC ${SRC})

set_target_properties( ${PROJECT_NAME}
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)
