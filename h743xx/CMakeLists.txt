add_library(${PROJECT_NAME}_h743xx INTERFACE)

add_library(${PROJECT_NAME}::h743xx ALIAS ${PROJECT_NAME}_h743xx)

target_sources(${PROJECT_NAME}_h743xx INTERFACE
  fdcan.cpp
  spi.cpp
  tim.cpp
  uart.cpp
)

target_include_directories(${PROJECT_NAME}_h743xx INTERFACE
  include
)