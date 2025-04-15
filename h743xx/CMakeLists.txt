get_target_property(defs stm32cubemx INTERFACE_COMPILE_DEFINITIONS)
if("STM32H743xx" IN_LIST defs)
  target_sources(${PROJECT_NAME} PRIVATE
    fdcan.cpp
    spi.cpp
    tim.cpp
    uart.cpp
  )
endif()