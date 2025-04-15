cmake_minimum_required(VERSION 3.22)

project(stm32rcos LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME} INTERFACE
  src/uart.cpp
)

target_include_directories(${PROJECT_NAME} INTERFACE
  include
)

add_subdirectory(f103xx)
add_subdirectory(f303xx)
add_subdirectory(f446xx)
add_subdirectory(h743xx)

add_library(stm32rcos::f103xx ALIAS ${PROJECT_NAME}_f103xx)
add_library(stm32rcos::f303xx ALIAS ${PROJECT_NAME}_f303xx)
add_library(stm32rcos::f446xx ALIAS ${PROJECT_NAME}_f446xx)
add_library(stm32rcos::h743xx ALIAS ${PROJECT_NAME}_h743xx)

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
