
project(easylogging CXX)


add_library(easylogging++ easylogging++.h easylogging++.cc)
target_include_directories(easylogging++ PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(easylogging++ PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
