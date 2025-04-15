add_library(${PROJECT_NAME}_f303xx INTERFACE)

add_library(${PROJECT_NAME}::f303xx ALIAS ${PROJECT_NAME}_f303xx)

target_sources(${PROJECT_NAME}_f303xx INTERFACE
  bxcan.cpp
  spi.cpp
  tim.cpp
  uart.cpp
)

target_include_directories(${PROJECT_NAME}_f303xx INTERFACE
  include
)