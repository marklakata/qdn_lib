project(qdn_lib)

add_library(qdn_lib
  qdn_adc.cpp
  qdn_adc.h
  qdn_adc_ad7xxx.cpp
  qdn_adc_ad7xxx.h
  qdn_cpu.h
  qdn_dac_mcp4822.cpp
  qdn_dac_mcp4822.h
  qdn_dma.cpp
  qdn_dma.h
  qdn_ds24b33.cpp
  qdn_ds24b33.h
  qdn_fifo.cpp
  qdn_fifo.h
  qdn_flash.h
  qdn_flash_stm32f1xx.c
  qdn_gpio.cpp
  qdn_gpio.h
  qdn_onewire.cpp
  qdn_onewire.h
  qdn_parameter.cpp
  qdn_parameter.h
  qdn_rtc.c
  qdn_rtc.h
  qdn_serial.h
  qdn_serial_debug.h
  qdn_serial_stm32.cpp
  qdn_spi.h
  qdn_spi_stm32f10x.cpp
  qdn_stm32_hard_fault_handler.h
  qdn_stm32_hard_fault_handler_c.c
  qdn_stm32_hard_fault_handler_gnu.s
  qdn_stm32f10x.c
  qdn_stm32f10x.h
  qdn_stm32fxxx.c
  qdn_stm32fxxx.h
  qdn_timer.cpp
  qdn_timer.h
  qdn_util.cpp
  qdn_util.h
  qdn_xos.h
  qdn_xos_no_rtos.c
)

  #qdn_xos.c

  # qdn_flash_stm32f4xx.c
  # qdn_stm32f4xx.c

  # qdn_flash_stm32l1xx.c

  #qdn_interpolation.cpp
  #qdn_interpolation.h

  #qdn_flash_msp430f5xxx.c


target_include_directories(qdn_lib
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(qdn_lib
  PRIVATE 
    "${CMAKE_CURRENT_LIST_DIR}/../be3x00_sensor/Libraries/STM32F10x_StdPeriph_Driver/inc"
    "${CMAKE_CURRENT_LIST_DIR}/../be3x00_sensor/Libraries/CMSIS/Device/ST/STM32F10x/Include"
    "${CMAKE_CURRENT_LIST_DIR}/../be3x00_sensor/Libraries/CMSIS/Include"
    "${CMAKE_CURRENT_LIST_DIR}/../be3x00_sensor/Libraries/STM32_USB-FS-Device_Driver/inc"
)


target_link_libraries(qdn_lib
  Library
)
 
