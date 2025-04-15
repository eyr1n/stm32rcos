cmake_minimum_required(VERSION 3.22)

project(stm32rcos LANGUAGES CXX)

set(DEVICE_SPECIFIC_SOURCES
  device/bxcan.cpp
  device/fdcan.cpp
  device/spi.cpp
  device/tim.cpp
  device/uart.cpp
)

# stm32rcos
add_library(${PROJECT_NAME} OBJECT
  ${DEVICE_SPECIFIC_SOURCES}
  src/peripheral/uart/uart.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)
target_link_libraries(${PROJECT_NAME} PUBLIC
  STM32_Drivers
  FreeRTOS
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

# stm32rcos::printf_float
add_library(${PROJECT_NAME}_printf_float INTERFACE)
add_library(${PROJECT_NAME}::printf_float ALIAS ${PROJECT_NAME}_printf_float)
target_link_libraries(${PROJECT_NAME}_printf_float INTERFACE
  "-u _printf_float"
)

# Generate device-specific sources
find_package(Python REQUIRED COMPONENTS Interpreter)
add_custom_command(
  OUTPUT ${DEVICE_SPECIFIC_SOURCES}
  COMMAND 
    ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/device_specific_sources.py 
      ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device device
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/device_specific_sources.py
  VERBATIM
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
