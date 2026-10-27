cmake_minimum_required(VERSION 3.21)
project(ice-audio-lab LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_options(/W4 /MP /permissive- /utf-8 /fp:fast)
    add_compile_definitions(_USE_MATH_DEFINES)
else()
    add_compile_options(-Wall -Wextra)
    add_compile_options($<$<CONFIG:Release>:-O3 -ffast-math>)
endif()

# ------------------------------------------------------------------
# third-party: fetch on configure (no vcpkg needed)
# ------------------------------------------------------------------
include(FetchContent)

# Dear ImGui (backend: Win32 + OpenGL3) --------------------------------
set(IMGUI_NO_OBSOLETE_FUNCTIONS OFF CACHE INTERNAL "")
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)
# imgui fetch has no CMake; expose its include dir for backends
add_library(imgui_headers INTERFACE)
target_include_directories(imgui_headers INTERFACE "${imgui_SOURCE_DIR}" "${imgui_SOURCE_DIR}/backends")

# implot (plots on ImGui) ----------------------------------------------
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.16
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(implot)

# nlohmann/json (header-only) ------------------------------------------
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# miniaudio (header-only; we compile a tiny TU that includes the implementation)
FetchContent_Declare(
    miniaudio
    GIT_REPOSITORY https://github.com/mackron/miniaudio.git
    GIT_TAG        0.11.21
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(miniaudio)

add_library(miniaudio STATIC "${CMAKE_CURRENT_SOURCE_DIR}/audio/miniaudio_impl.c")
set_target_properties(miniaudio PROPERTIES LINKER_LANGUAGE C)
target_include_directories(miniaudio PUBLIC "${miniaudio_SOURCE_DIR}")
target_compile_definitions(miniaudio PRIVATE _CRT_SECURE_NO_WARNINGS)

# ------------------------------------------------------------------
# static lib: core (physics) + audio (dsp)
# ------------------------------------------------------------------
add_library(ice_core STATIC
    core/gas_system.cpp
    core/engine.cpp
    core/piston_engine.cpp
    core/combustion.cpp
    core/exhaust1d.cpp
    core/config.cpp
)
target_include_directories(ice_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ice_core PUBLIC nlohmann_json::nlohmann_json)

add_library(ice_audio STATIC
    audio/synth.cpp
    audio/ring_buffer.cpp
)
target_include_directories(ice_audio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ice_audio PUBLIC ice_core)

# ------------------------------------------------------------------
# app (Dear ImGui + miniaudio)
# ------------------------------------------------------------------
add_executable(ice-audio-lab
    app/main.cpp
    app/app.cpp
    app/ui.cpp
)
target_include_directories(ice-audio-lab PRIVATE
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_sources(ice-audio-lab PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_compile_definitions(ice-audio-lab PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
)
# stock imgui GL3 loader header lives in backends/
target_include_directories(ice-audio-lab PRIVATE "${imgui_SOURCE_DIR}/backends")
target_link_libraries(ice-audio-lab PRIVATE
    ice_audio
    miniaudio
    opengl32
    dwmapi
)

# ------------------------------------------------------------------
# headless smoke test (no UI, no audio device)
# ------------------------------------------------------------------
add_executable(ice-headless tests/headless.cpp)
target_link_libraries(ice-headless PRIVATE ice_core)
target_compile_definitions(ice-headless PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

add_executable(ice-firststep tests/firststep.cpp)
target_link_libraries(ice-firststep PRIVATE ice_core)
target_compile_definitions(ice-firststep PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

add_executable(ice-exhaust tests/exhaust_test.cpp)
target_link_libraries(ice-exhaust PRIVATE ice_core)
target_compile_definitions(ice-exhaust PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

add_executable(ice-validation tests/validation.cpp)
target_link_libraries(ice-validation PRIVATE ice_core)
target_compile_definitions(ice-validation PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

add_executable(ice-pulseshape tests/pulse_shape.cpp)
target_link_libraries(ice-pulseshape PRIVATE ice_core)
target_compile_definitions(ice-pulseshape PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

add_executable(ice-pulsesync tests/pulse_sync.cpp)
target_link_libraries(ice-pulsesync PRIVATE ice_core)
target_compile_definitions(ice-pulsesync PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

add_executable(ice-idleaudio tests/idle_audio.cpp)
target_link_libraries(ice-idleaudio PRIVATE ice_audio)
target_compile_definitions(ice-idleaudio PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)

