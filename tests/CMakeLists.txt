include(GoogleTest)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Helper macro: build a test executable, link gector_kernel + GTest, register
# every TEST() case with CTest so VS Test Explorer discovers them individually.
macro(gk_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE gector_kernel GTest::gtest_main)
    gtest_discover_tests(${name})
endmacro()

gk_add_test(test_smoke      test_smoke.cpp)
gk_add_test(test_vec2       test_vec2.cpp)
gk_add_test(test_vec3       test_vec3.cpp)
gk_add_test(test_vec4       test_vec4.cpp)
gk_add_test(test_mat2x2     test_mat2x2.cpp)
gk_add_test(test_mat3x3     test_mat3x3.cpp)
gk_add_test(test_mat4x4     test_mat4x4.cpp)
gk_add_test(test_quaternion test_quaternion.cpp)
gk_add_test(test_interval   test_interval.cpp)
# Iteration 1.3
gk_add_test(test_tolerances test_tolerances.cpp)
# Iteration 1.4
gk_add_test(test_handle     test_handle.cpp)
gk_add_test(test_arena      test_arena.cpp)
gk_add_test(test_entityid   test_entityid.cpp)
# Iteration 1.5
gk_add_test(test_result     test_result.cpp)
gk_add_test(test_log        test_log.cpp)
# Chapter 3: Parametric Surfaces
gk_add_test(test_surface_analytic  test_surface_analytic.cpp)
gk_add_test(test_surface_bspline   test_surface_bspline.cpp)
gk_add_test(test_surface_nurbs     test_surface_nurbs.cpp)
gk_add_test(test_surface_utils     test_surface_utils.cpp)
gk_add_test(test_surface_obj       test_surface_obj.cpp)
# Chapter 2: Parametric Curves
gk_add_test(test_curve_iface       test_curve_iface.cpp)
gk_add_test(test_curve_analytic    test_curve_analytic.cpp)
gk_add_test(test_curve_bspline     test_curve_bspline.cpp)
gk_add_test(test_curve_utils       test_curve_utils.cpp)

# Chapter 4: Topological Data Model
gk_add_test(test_brep_topology   test_brep_topology.cpp)
gk_add_test(test_brep_validator  test_brep_validator.cpp)
gk_add_test(test_brep_query      test_brep_query.cpp)
gk_add_test(test_brep_io         test_brep_io.cpp)

# Chapter 5: Primitive Solid Builders + Sketch
gk_add_test(test_builders_5_1   test_builders_5_1.cpp)
gk_add_test(test_builders_5_2   test_builders_5_2.cpp)
gk_add_test(test_sketch         test_sketch.cpp)
gk_add_test(test_extrusion      test_extrusion.cpp)

# Chapter 9: File Format I/O
gk_add_test(test_step_writer    test_step_writer.cpp)
gk_add_test(test_step_models    test_step_models.cpp)
