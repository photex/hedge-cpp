
if(MSVC)
  add_definitions(/DNOMINMAX)
endif()

add_library(hedge STATIC hedge.hpp hedge.cpp)
target_include_directories(hedge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hedge eigen easylogging++)
set_target_properties(hedge PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)

macro(make_test TEST_NAME SOURCE_FILE)
    set(_TARGET_NAME "${TEST_NAME}-test")
    add_executable(${_TARGET_NAME} ${SOURCE_FILE})
    target_link_libraries(${_TARGET_NAME} hedge catch)
    set_target_properties(${_TARGET_NAME} PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
    add_test(NAME ${TEST_NAME} COMMAND ${_TARGET_NAME} -r console)
endmacro()

make_test(kernel-operations tests/kernel_operations.cpp)
make_test(high-level-api tests/high_level_api.cpp)
make_test(core-types tests/core_types.cpp)
