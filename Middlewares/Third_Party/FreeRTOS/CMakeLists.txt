# Add libraries
# aux_source_directory(Source FREERTOS_SOURCES)

file(GLOB_RECURSE FREERTOS_SOURCES Source/*.c)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/include
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS_V2
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F
)

add_library(FreeRTOS ${FREERTOS_SOURCES})
target_link_libraries(FreeRTOS PUBLIC cmsis)
target_include_directories(FreeRTOS INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS_V2>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F>)
