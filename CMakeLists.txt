cmake_minimum_required(VERSION 3.21)

project(GectorKernel
    VERSION 0.1.0
    DESCRIPTION "Gector Kernel - Geometric Vector Kernel"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Compiler warnings ────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
elseif(MSVC)
    add_compile_options(/W4 /WX)
endif()

# ── Sanitizers (Debug builds) ────────────────────────────────────────────────
option(GK_ENABLE_SANITIZERS "Enable ASan + UBSan in Debug builds" OFF)
if(GK_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# ── Kernel library ───────────────────────────────────────────────────────────
add_library(gector_kernel INTERFACE)
target_include_directories(gector_kernel INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/kernel/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(gector_kernel INTERFACE cxx_std_17)

# ── Google Test ──────────────────────────────────────────────────────────────
# Try a pre-installed copy first; if absent download via FetchContent.
option(GK_BUILD_TESTS "Build tests" ON)
if(GK_BUILD_TESTS)
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        v1.14.0
        )
        # Avoid overriding the parent project's compiler/linker settings on Windows.
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(BUILD_GMOCK  OFF CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    enable_testing()
    add_subdirectory(tests)
endif()
