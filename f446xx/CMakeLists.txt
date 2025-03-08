cmake_minimum_required(VERSION 3.22)

project(stm32rcos_f446xx LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME} INTERFACE
  bxcan.cpp
  uart.cpp
)