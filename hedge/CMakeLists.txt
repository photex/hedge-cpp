
add_library(hedge STATIC hedge.hpp hedge.cpp)
target_include_directories(hedge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hedge mathfu easylogging++)
set_target_properties(hedge PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)

add_executable(hedge_test hedge_test.cpp)
target_link_libraries(hedge_test hedge catch)
set_target_properties(hedge_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
