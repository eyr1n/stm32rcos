cmake_minimum_required(VERSION 3.22)

project(stm32rcos LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME} INTERFACE
  src/uart.cpp
)

target_include_directories(${PROJECT_NAME} INTERFACE
  include
)

add_library(${PROJECT_NAME}_printf_float INTERFACE)

add_library(${PROJECT_NAME}::printf_float ALIAS ${PROJECT_NAME}_printf_float)

target_link_libraries(${PROJECT_NAME}_printf_float INTERFACE
  "-u _printf_float"
)

add_subdirectory(f103xx)
add_subdirectory(f303xx)
add_subdirectory(f446xx)
add_subdirectory(h743xx)

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
