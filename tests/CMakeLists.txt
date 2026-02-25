include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Helper macro to add a test executable that links to gector_kernel
macro(gk_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE gector_kernel)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

gk_add_test(test_smoke     test_smoke.cpp)
gk_add_test(test_vec2      test_vec2.cpp)
gk_add_test(test_vec3      test_vec3.cpp)
gk_add_test(test_vec4      test_vec4.cpp)
gk_add_test(test_mat2x2    test_mat2x2.cpp)
gk_add_test(test_mat3x3    test_mat3x3.cpp)
gk_add_test(test_mat4x4    test_mat4x4.cpp)
gk_add_test(test_quaternion test_quaternion.cpp)
gk_add_test(test_interval  test_interval.cpp)
