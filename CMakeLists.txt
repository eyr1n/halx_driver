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

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  halx
)
target_include_directories(${PROJECT_NAME} INTERFACE
  ${eigen_SOURCE_DIR}  
  include
)
