cmake_minimum_required(VERSION 3.22)

project(halx_driver LANGUAGES CXX)

# halx_driver
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  halx
)
target_include_directories(${PROJECT_NAME} INTERFACE
  include
)

# Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_OUTPUT_LANGUAGE Japanese)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
  set(DOXYGEN_DISABLE_INDEX NO)
  set(DOXYGEN_GENERATE_TREEVIEW NO)
  doxygen_add_docs(${PROJECT_NAME}_docs
    README.md
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()
