# CMSIS Build CMakeLists generated on 2024-02-14T18:41:26

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET G030_T12)
set(CPU Cortex-M0+)
set(PRJ_DIR "c:/Users/Robin Lu/Desktop/G030-T12/MDK-ARM")
set(OUT_DIR "C:/Users/Robin Lu/Desktop/G030-T12/MDK-ARM/out/G030_T12/G030T12")
set(INT_DIR "C:/Users/Robin Lu/Desktop/G030-T12/MDK-ARM/tmp/G030_T12/G030T12")
set(FPU NO_FPU)
set(BYTE_ORDER Little-endian)
set(OPTIMIZE none)
set(DEBUG on)
set(CC_FLAGS_GLOBAL "-std=c99 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(CXX_FLAGS_GLOBAL "-xc++ -std=c++03 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(LD_FLAGS_GLOBAL "--library_type=microlib --map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --lto --strict --summary_stderr --info summarysizes")
set(LD_SCRIPT "c:/Users/Robin Lu/Desktop/G030-T12/MDK-ARM/G030_T12_G030T12.sct")

set(DEFINES
  STM32G031xx
  _RTE_
  USE_FULL_LL_DRIVER
  HSE_VALUE=8000000
  HSE_STARTUP_TIMEOUT=100
  LSE_STARTUP_TIMEOUT=5000
  LSE_VALUE=32768
  EXTERNAL_CLOCK_VALUE=12288000
  HSI_VALUE=16000000
  LSI_VALUE=32000
  VDD_VALUE=3300
  PREFETCH_ENABLE=1
  INSTRUCTION_CACHE_ENABLE=1
  DATA_CACHE_ENABLE=1
  __MICROLIB
)

set(INC_PATHS
  "c:/Users/Robin Lu/Desktop/G030-T12/Inc"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Inc"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/CMSIS/Device/ST/STM32G0xx/Include"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/CMSIS/Include"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2"
  "c:/Users/Robin Lu/Desktop/G030-T12/MDK-ARM/RTE/_G030T12"
  "C:/Users/Robin Lu/AppData/Local/arm/packs/ARM/CMSIS/6.0.0/CMSIS/Core/Include"
  "C:/Users/Robin Lu/AppData/Local/arm/packs/Keil/STM32G0xx_DFP/1.5.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include"
)

set(AS_ARM_SRC_FILES
  "c:/Users/Robin Lu/Desktop/G030-T12/MDK-ARM/startup_stm32g031xx.s"
)

set(CC_SRC_FILES
  "C:/Users/Robin Lu/AppData/Local/arm/packs/Keil/ARM_Compiler/1.7.2/Source/retarget_io.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_adc.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_dma.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_exti.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_gpio.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_pwr.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_rcc.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_spi.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_tim.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_utils.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/SEGGER_RTT.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/SEGGER_RTT_printf.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/adc.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/dma.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/gpio.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/main.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/oled_driver.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/spi.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/stm32g0xx_it.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/system_stm32g0xx.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/tim.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/mui.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/mui_u8g2.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_arc.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_bitmap.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_box.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_buffer.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_button.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_circle.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_cleardisplay.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_d_memory.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_d_setup.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_font.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_fonts.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_hvline.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_input_value.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_intersection.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_kerning.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_line.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_ll_hvline.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_message.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_polygon.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_selection_list.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8g2_setup.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8log.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8log_u8g2.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8log_u8x8.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_8x8.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_byte.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_cad.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_capture.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_d_ssd1306_128x64_noname.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_debounce.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_display.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_fonts.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_gpio.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_input_value.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_message.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_selection_list.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_setup.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_string.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_u16toa.c"
  "c:/Users/Robin Lu/Desktop/G030-T12/Src/u8g2/u8x8_u8toa.c"
)

set(AS_FLAGS_c:/Users/Robin?Lu/Desktop/G030-T12/MDK-ARM/startup_stm32g031xx.s "-masm=auto")

# Toolchain config map

set(REGISTERED_TOOLCHAIN_ROOT "c:/Users/Robin Lu/.vcpkg/artifacts/2139c4c6/compilers.arm.armclang/6.21.0/bin")
set(REGISTERED_TOOLCHAIN_VERSION "6.21.0")
set(TOOLCHAIN_VERSION_MIN "6.21.0")
include ("C:/Users/Robin Lu/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.2.1/etc/AC6.6.18.0.cmake")
include ("C:/Users/Robin Lu/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.2.1/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES AS_ARM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_AS_ARM_FLAGS "${AS_ARM_CPU} ${AS_ARM_BYTE_ORDER} ${AS_ARM_DEFINES} ${AS_ARM_OPTIONS_FLAGS} ${AS_ARM_FLAGS}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_BYTE_ORDER} ${CC_DEFINES} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Local Flags

foreach(SRC ${AS_ARM_SRC_FILES})
  string(REPLACE " " "?" S ${SRC})
  if(DEFINED AS_FLAGS_${S})
    set(AS_FLAGS_LOCAL "${AS_FLAGS_${S}}")
  else()
    set(AS_FLAGS_LOCAL "${AS_FLAGS_GLOBAL}")
  endif()
  set_source_files_properties(${SRC} PROPERTIES COMPILE_FLAGS "${AS_FLAGS_LOCAL}")
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE AS_ARM)
endforeach()

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${AS_ARM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "G030_T12")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
