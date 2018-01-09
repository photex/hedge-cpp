

project(vectorial CXX)

add_library(vectorial INTERFACE)
target_include_directories(vectorial INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
