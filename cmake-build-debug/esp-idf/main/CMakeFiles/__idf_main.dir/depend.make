# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.20

esp-idf/main/CMakeFiles/__idf_main.dir/FOC/FOC.cpp.obj: \
 ../main/FOC/FOC.cpp \
 ../main/FOC/FOC.h \
 ../main/FOC/foc_utilities.h \
 ../main/conf.h \
 ../main/driver/MotorControlPWM.h \
 ../main/driver/SPI_CLASS.h \
 ../main/print_utils.h \
 ../main/sensor/Tle5012b/TLE5012B.h \
 C:/Users/Justin/esp/esp-idf/components/driver/esp32/include/driver/adc.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/adc_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/mcpwm.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/periph_ctrl.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_master.h \
 C:/Users/Justin/esp/esp-idf/components/esp_adc_cal/include/esp_adc_cal.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_assert.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_bit_defs.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_compiler.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_err.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_idf_version.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_private/crosscore_int.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_types.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/compare_set.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/cpu.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/spinlock.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp_rom_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_intr_alloc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_system.h \
 C:/Users/Justin/esp/esp-idf/components/esp_timer/include/esp_timer.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/FreeRTOS.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/deprecated_definitions.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/list.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/mpu_wrappers.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/portable.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/projdefs.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/task.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/FreeRTOSConfig.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portbenchmark.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro_priv.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_config.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/hal/esp32/include/hal/cpu_ll.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/adc_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_hal.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/gpio_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/mcpwm_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/spi_types.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/esp_heap_caps.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/multi_heap.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/assert.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/esp_newlib.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/sys/reent.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_sig_map.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/io_mux_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/mcpwm_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/periph_defs.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/rtc_io_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdio_slave_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdmmc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_caps.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/gpio_periph.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/soc_memory_layout.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/spi_periph.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-isa.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-matmap.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/extreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/specreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/system.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie-asm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/esp_attr.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xt_instr_macros.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/coreasm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/corebits.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/hal.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa-versions.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-core-state.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-frames.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime.h \
 config/sdkconfig.h
esp-idf/main/CMakeFiles/__idf_main.dir/FOC/foc_utilities.cpp.obj: \
 ../main/FOC/foc_utilities.cpp \
 ../main/FOC/foc_utilities.h \
 ../main/conf.h
esp-idf/main/CMakeFiles/__idf_main.dir/driver/MotorControlPWM.cpp.obj: \
 ../main/conf.h \
 ../main/driver/MotorControlPWM.cpp \
 ../main/driver/MotorControlPWM.h \
 C:/Users/Justin/esp/esp-idf/components/driver/esp32/include/driver/adc.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/adc_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/mcpwm.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/periph_ctrl.h \
 C:/Users/Justin/esp/esp-idf/components/esp_adc_cal/include/esp_adc_cal.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_assert.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_bit_defs.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_compiler.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_err.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_idf_version.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_private/crosscore_int.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_types.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/compare_set.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/cpu.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/spinlock.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp_rom_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_intr_alloc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_system.h \
 C:/Users/Justin/esp/esp-idf/components/esp_timer/include/esp_timer.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/FreeRTOS.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/deprecated_definitions.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/list.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/mpu_wrappers.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/portable.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/projdefs.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/task.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/FreeRTOSConfig.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portbenchmark.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro_priv.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_config.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/hal/esp32/include/hal/cpu_ll.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/adc_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_hal.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/gpio_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/mcpwm_types.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/esp_heap_caps.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/multi_heap.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/assert.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/esp_newlib.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/sys/reent.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_sig_map.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/io_mux_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/mcpwm_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/periph_defs.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/rtc_io_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_caps.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/gpio_periph.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/soc_memory_layout.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-isa.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-matmap.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/extreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/specreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/system.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie-asm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/esp_attr.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xt_instr_macros.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/coreasm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/corebits.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/hal.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa-versions.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-core-state.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-frames.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime.h \
 config/sdkconfig.h
esp-idf/main/CMakeFiles/__idf_main.dir/driver/SPI_CLASS.cpp.obj: \
 ../main/driver/SPI_CLASS.cpp \
 ../main/driver/SPI_CLASS.h \
 ../main/print_utils.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_master.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_assert.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_bit_defs.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_compiler.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_err.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_idf_version.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_private/crosscore_int.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_types.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/compare_set.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/cpu.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/spinlock.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp_rom_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_intr_alloc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_system.h \
 C:/Users/Justin/esp/esp-idf/components/esp_timer/include/esp_timer.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/FreeRTOS.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/deprecated_definitions.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/mpu_wrappers.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/portable.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/projdefs.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/FreeRTOSConfig.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portbenchmark.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro_priv.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_config.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/hal/esp32/include/hal/cpu_ll.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_hal.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/gpio_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/spi_types.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/esp_heap_caps.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/multi_heap.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/assert.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/esp_newlib.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/sys/reent.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_sig_map.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/io_mux_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/periph_defs.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/rtc_io_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdio_slave_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdmmc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_caps.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/gpio_periph.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/soc_memory_layout.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/spi_periph.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-isa.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-matmap.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/extreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/specreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/system.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie-asm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/esp_attr.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xt_instr_macros.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/coreasm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/corebits.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/hal.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa-versions.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-core-state.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-frames.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime.h \
 config/sdkconfig.h
esp-idf/main/CMakeFiles/__idf_main.dir/main.cpp.obj: \
 ../main/FOC/FOC.h \
 ../main/FOC/foc_utilities.h \
 ../main/conf.h \
 ../main/driver/MotorControlPWM.h \
 ../main/driver/SPI_CLASS.h \
 ../main/main.cpp \
 ../main/print_utils.h \
 ../main/sensor/Tle5012b/TLE5012B.h \
 C:/Users/Justin/esp/esp-idf/components/driver/esp32/include/driver/adc.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/adc_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/mcpwm.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/periph_ctrl.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_master.h \
 C:/Users/Justin/esp/esp-idf/components/esp_adc_cal/include/esp_adc_cal.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_assert.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_bit_defs.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_compiler.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_err.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_idf_version.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_private/crosscore_int.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_types.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/compare_set.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/cpu.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/spinlock.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp_rom_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_intr_alloc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_system.h \
 C:/Users/Justin/esp/esp-idf/components/esp_timer/include/esp_timer.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/FreeRTOS.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/deprecated_definitions.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/list.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/mpu_wrappers.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/portable.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/projdefs.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/task.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/FreeRTOSConfig.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portbenchmark.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro_priv.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_config.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/hal/esp32/include/hal/cpu_ll.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/adc_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_hal.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/gpio_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/mcpwm_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/spi_types.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/esp_heap_caps.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/multi_heap.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/assert.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/esp_newlib.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/sys/reent.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_sig_map.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/io_mux_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/mcpwm_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/periph_defs.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/rtc_io_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdio_slave_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdmmc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_caps.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/timer_group_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/timer_group_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/gpio_periph.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/soc_memory_layout.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/spi_periph.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-isa.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-matmap.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/extreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/specreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/system.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie-asm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/esp_attr.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xt_instr_macros.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/coreasm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/corebits.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/hal.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa-versions.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-core-state.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-frames.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime.h \
 config/sdkconfig.h
esp-idf/main/CMakeFiles/__idf_main.dir/sensor/Tle5012b/TLE5012B.cpp.obj: \
 ../main/driver/SPI_CLASS.h \
 ../main/print_utils.h \
 ../main/sensor/Tle5012b/TLE5012B.cpp \
 ../main/sensor/Tle5012b/TLE5012B.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_common.h \
 C:/Users/Justin/esp/esp-idf/components/driver/include/driver/spi_master.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_assert.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_bit_defs.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_compiler.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_err.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_idf_version.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_private/crosscore_int.h \
 C:/Users/Justin/esp/esp-idf/components/esp_common/include/esp_types.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/compare_set.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/cpu.h \
 C:/Users/Justin/esp/esp-idf/components/esp_hw_support/include/soc/spinlock.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32c3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/gpio.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s2/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/ets_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp32s3/rom/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_rom/include/esp_rom_sys.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_intr_alloc.h \
 C:/Users/Justin/esp/esp-idf/components/esp_system/include/esp_system.h \
 C:/Users/Justin/esp/esp-idf/components/esp_timer/include/esp_timer.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/FreeRTOS.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/deprecated_definitions.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/mpu_wrappers.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/portable.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/include/freertos/projdefs.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/FreeRTOSConfig.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portbenchmark.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro_priv.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_config.h \
 C:/Users/Justin/esp/esp-idf/components/freertos/port/xtensa/include/freertos/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/hal/esp32/include/hal/cpu_ll.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_hal.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/cpu_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/gpio_types.h \
 C:/Users/Justin/esp/esp-idf/components/hal/include/hal/spi_types.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/esp_heap_caps.h \
 C:/Users/Justin/esp/esp-idf/components/heap/include/multi_heap.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/assert.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/esp_newlib.h \
 C:/Users/Justin/esp/esp-idf/components/newlib/platform_include/sys/reent.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_sig_map.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/gpio_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/io_mux_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/periph_defs.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/rtc_io_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdio_slave_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/sdmmc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_caps.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/soc_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_pins.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_reg.h \
 C:/Users/Justin/esp/esp-idf/components/soc/esp32/include/soc/spi_struct.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/gpio_periph.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/lldesc.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/soc_memory_layout.h \
 C:/Users/Justin/esp/esp-idf/components/soc/include/soc/spi_periph.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-isa.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core-matmap.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/core.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/extreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/specreg.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/system.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie-asm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/esp32/include/xtensa/config/tie.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/esp_attr.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xt_instr_macros.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/coreasm.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/corebits.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/hal.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa-versions.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_api.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtensa_context.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-core-state.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime-frames.h \
 C:/Users/Justin/esp/esp-idf/components/xtensa/include/xtensa/xtruntime.h \
 config/sdkconfig.h
