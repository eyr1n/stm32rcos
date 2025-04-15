add_library(${PROJECT_NAME}_f303xx INTERFACE)

target_sources(${PROJECT_NAME}_f303xx INTERFACE
  bxcan.cpp
  spi.cpp
  tim.cpp
  uart.cpp
)

target_include_directories(${PROJECT_NAME}_f303xx INTERFACE
  include
)