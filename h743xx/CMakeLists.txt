cmake_minimum_required(VERSION 3.22)

project(stm32rcos_h743xx LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME} INTERFACE
  fdcan.cpp
  uart.cpp
)

target_include_directories(${PROJECT_NAME} INTERFACE
  include
)