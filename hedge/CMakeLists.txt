
if(MSVC)
  add_definitions(/DNOMINMAX)
endif()

add_library(hedge STATIC hedge.hpp hedge.cpp)
target_include_directories(hedge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hedge eigen easylogging++)
set_target_properties(hedge PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)

macro(make_test TEST_NAME SOURCE_FILE)
    add_executable(${TEST_NAME}-test ${SOURCE_FILE})
    target_link_libraries(${TEST_NAME}-test hedge catch)
    set_target_properties(${TEST_NAME}-test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME}-test -r console)
endmacro()

make_test(kernel-operations tests/kernel_operations.cpp)
make_test(high-level-api tests/high_level_api.cpp)
make_test(core-types tests/core_types.cpp)
