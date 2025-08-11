cmake_minimum_required(VERSION 3.22)

project(halx_driver LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR invalid
)
FetchContent_MakeAvailable(eigen)

# halx_driver
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  halx
)
target_include_directories(${PROJECT_NAME} INTERFACE
  ${eigen_SOURCE_DIR}  
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
  doxygen_add_docs(docs
    README.md
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()
