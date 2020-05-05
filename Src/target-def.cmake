# Target-specific flags
set(MCU_ARCH ARMCM4)
set(MCU_FAMILY STM32F427xx)
set(MCU_NAME STM32F427IGTx)

set(CPU "-mcpu=cortex-m4")
set(FPU "-mfpu=fpv4-sp-d16")
set(FLOAT_ABI "-mfloat-abi=hard")

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F427IGTx_FLASH.ld)

option(USE_LL_LIB "Enable LL library" ON)
option(USE_HAL_LIB "Enable HAL library" ON)
option(USE_SYSTEM_VIEW "Enable Segger SystemView library" OFF)