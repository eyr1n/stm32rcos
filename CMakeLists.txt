cmake_minimum_required(VERSION 3.22)

project(stm32rcos LANGUAGES CXX)

set(DEVICE_SPECIFIC_SOURCES
  ${CMAKE_CURRENT_BINARY_DIR}/device/bxcan.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/device/fdcan.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/device/i2c.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/device/spi.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/device/tim.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/device/uart.cpp
)

# Generate device-specific sources
find_package(Python REQUIRED COMPONENTS Interpreter)
add_custom_command(
  OUTPUT ${DEVICE_SPECIFIC_SOURCES}
  COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/device_specific_sources.py 
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST device
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/device_specific_sources.py
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_device DEPENDS ${DEVICE_SPECIFIC_SOURCES})

# stm32rcos
add_library(${PROJECT_NAME} INTERFACE)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_device)
target_sources(${PROJECT_NAME} INTERFACE
  ${DEVICE_SPECIFIC_SOURCES}
  src/stdout.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE
  cxx_std_23
)
target_include_directories(${PROJECT_NAME} INTERFACE
  include
)

# stm32rcos::printf_float
add_library(${PROJECT_NAME}_printf_float INTERFACE)
add_library(${PROJECT_NAME}::printf_float ALIAS ${PROJECT_NAME}_printf_float)
target_link_libraries(${PROJECT_NAME}_printf_float INTERFACE
  "-u _printf_float"
)

# Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_OUTPUT_LANGUAGE Japanese)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
  doxygen_add_docs(docs
    README.md
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()
