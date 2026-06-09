# MiniCAD-PSX build.
#
# Two build modes:
#   HOST  (default): builds the integer kernel + unit tests with the system
#                    compiler. No PlayStation toolchain needed.
#                    -> cmake -B build && cmake --build build && ./build/test_kernel
#
#   PSX:   builds the actual PlayStation executable with PSn00bSDK.
#          Requires the PSn00bSDK toolchain file.
#          -> cmake -B build-psx -DCMAKE_TOOLCHAIN_FILE=$PSN00BSDK_LIBS/cmake/sdk.cmake
#             cmake --build build-psx
#
cmake_minimum_required(VERSION 3.21)

# Detect whether we're using the PSn00bSDK toolchain.
if(DEFINED PSN00BSDK_TARGET OR DEFINED ENV{PSN00BSDK_LIBS})
    set(MINICAD_PSX ON)
else()
    set(MINICAD_PSX OFF)
endif()

# PSn00bSDK's link step pulls in C++/ASM runtime objects, so the PSX build needs
# those languages enabled; host build is happy with the extra languages present.
project(minicad_psx C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Portable kernel sources (compile for BOTH host and PSX).
set(KERNEL_SRC
    src/foundation/fixed.c
    src/foundation/mem.c
    src/foundation/ivec.c
    src/kernel/brep.c
    src/kernel/ops.c
    src/kernel/sketch.c
    src/kernel/save.c
    src/model/feature.c
    src/app/history.c
    src/app/modeling.c
)

set(WARN -Wall -Wextra -Wconversion -Wpedantic)

if(MINICAD_PSX)
    # -------- PlayStation executable --------
    # No find_package needed: the PSn00bSDK toolchain file (sdk.cmake) already
    # defines psn00bsdk_add_executable and links libpsn00b.
    psn00bsdk_add_executable(minicad GPREL
        ${KERNEL_SRC}
        src/foundation/sintab.c
        src/render/render.c
        src/render/camera.c
        src/input/input.c
        src/io/memcard.c
        src/main.c
    )
    target_include_directories(minicad PRIVATE include)
    target_compile_definitions(minicad PRIVATE MINICAD_PSX=1)
    # Optional save-icon TIM: only embed it if the artist asset exists, otherwise
    # the build would hard-fail on a missing incbin (icon is not needed to boot).
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/icon.tim")
        psn00bsdk_target_incbin(minicad PRIVATE icon assets/icon.tim)
        target_compile_definitions(minicad PRIVATE MINICAD_HAVE_ICON=1)
    endif()
else()
    # -------- Host kernel library + tests --------
    add_library(minicad_kernel STATIC ${KERNEL_SRC})
    target_include_directories(minicad_kernel PUBLIC include)
    target_compile_definitions(minicad_kernel PUBLIC MINICAD_HOST=1)
    target_compile_options(minicad_kernel PRIVATE ${WARN})

    add_executable(test_kernel host_tests/test_kernel.c)
    target_link_libraries(test_kernel PRIVATE minicad_kernel m)  # libm: host sine-table build (fixed.c)
    target_compile_definitions(test_kernel PRIVATE MINICAD_HOST=1)
    target_compile_options(test_kernel PRIVATE ${WARN}
        $<$<CONFIG:Debug>:-fsanitize=address,undefined>)
    target_link_options(test_kernel PRIVATE
        $<$<CONFIG:Debug>:-fsanitize=address,undefined>)

    enable_testing()
    add_test(NAME kernel COMMAND test_kernel)
endif()
