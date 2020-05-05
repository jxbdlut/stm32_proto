cmake_minimum_required(VERSION 3.7)
project(stm32_proto C CXX ASM)

message(STATUS "Building project: ${PROJECT_NAME}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F427IGTx_FLASH.ld)


# mcu
set(MCU "${CPU} -mthumb ${FPU} ${FLOAT-ABI}" )

# compile gcc flags
set(ASFLAGS "${MCU} ${OPT} -Wall -fdata-sections -ffunction-sections")
set(CFLAGS  "${MCU} ${OPT} -Wall -fdata-sections -ffunction-sections")

# set the build type
if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Debug)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Build type: Debug")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Build type: Release")
endif()

## Setup cross-compile toolchain 
set(CROSS_COMPILE_PREFIX arm-none-eabi)
include(${CMAKE_SOURCE_DIR}/Src/cmake/gcc-arm-none-eabi.cmake)
include(${CMAKE_SOURCE_DIR}/Src/target-def.cmake)

## auto-set variables from user input
set(MCU_FLAGS "${CPU} -mthumb ${FPU} ${FLOAT_ABI}")
set(LINK_FLAGS "${MCU_FLAGS} -Wl,--gc-sections")
set(EXTRA_LINK_FLAGS "-Wl,-Map=${PROJECT_NAME}.map,--cref,--no-warn-mismatch -specs=nano.specs -specs=nosys.specs")
include(${CMAKE_SOURCE_DIR}/Src/cmake/stm32-gcc-flags.cmake)

# print summary of configurations
message(STATUS "MCU_FAMILY: ${MCU_FAMILY}")
message(STATUS "MCU_NAME: ${MCU_NAME}")
message(STATUS "Specified C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Linker script: ${LINKER_SCRIPT}")
message(STATUS "Use LL library: ${USE_LL_LIB}")
message(STATUS "Use HAL library: ${USE_HAL_LIB}")
message(STATUS "Use Segger SystemView library: ${USE_SYSTEM_VIEW}")

## version information
set(VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set(VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
if(USE_SYSTEM_VIEW)
add_definitions("-DENABLE_SYSTEMVIEW")
endif()

## add project components
set(ELF_TARGET ${PROJECT_NAME}.elf)
set(PROJ_DRIVERS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Drivers)
set(PROJ_FREERTOS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party)

# project-specific modules
add_subdirectory(${PROJ_DRIVERS_PATH}/CMSIS ${CMAKE_BINARY_DIR}/Drivers/CMSIS/build)
add_subdirectory(${PROJ_DRIVERS_PATH}/STM32F4xx_HAL_Driver ${PROJ_DRIVERS_PATH}/STM32F4xx_HAL_Driver/build)
add_subdirectory(${PROJ_FREERTOS_PATH}/FreeRTOS ${PROJ_FREERTOS_PATH}/FreeRTOS/build)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto ${CMAKE_CURRENT_SOURCE_DIR}/proto/build)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/freertos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f427xx.s
)

add_executable(${ELF_TARGET} ${SOURCES})
target_compile_definitions(${ELF_TARGET} PUBLIC -D${MCU_FAMILY})
target_link_libraries(${ELF_TARGET} PUBLIC cmsis nanopb FreeRTOS stm32f4xx_hal -lc -lm -lnosys)

target_include_directories(${ELF_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>)
  target_link_libraries(${ELF_TARGET} PRIVATE -T${LINKER_SCRIPT} ${EXTRA_LINK_FLAGS})

## add post-build targets
include(${CMAKE_CURRENT_SOURCE_DIR}/Src/cmake/stm32-gcc-postbuild.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/stm32-gcc-jlink.cmake)