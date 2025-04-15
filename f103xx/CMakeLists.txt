add_library(${PROJECT_NAME}_f103xx INTERFACE)

add_library(${PROJECT_NAME}::f103xx ALIAS ${PROJECT_NAME}_f103xx)

target_sources(${PROJECT_NAME}_f103xx INTERFACE
  bxcan.cpp
  spi.cpp
  tim.cpp
  uart.cpp
)

target_include_directories(${PROJECT_NAME}_f103xx INTERFACE
  include
)