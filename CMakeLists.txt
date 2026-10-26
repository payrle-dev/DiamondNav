# DiamondNav: pathfinding and road routes for open.mp NPCs (README.md).
#
#   DiamondNav    the open.mp component (.so / .dll), loaded from components/
#   navbake       the offline baker: a world's collision (OBJ) -> navmesh/<map>.nav
#   navquery      a command-line check of a baked folder (paths and road routes)
#
# Both are 32-bit, like the server, so the baked tiles are byte-for-byte what the
# component loads.
#
#   Linux:    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
#                 -DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32
#   Windows:  cmake -S . -B build -A Win32 -T ClangCL
#             cmake --build build --config Release
#
# Dependencies are pinned here and fetched by CMake: the open.mp SDK, pawn-natives
# and the Pawn headers at the commits open.mp v1.6.7.3218 itself pins
# and Recast/Detour v1.6.0.

cmake_minimum_required(VERSION 3.19)
project(DiamondNav LANGUAGES C CXX VERSION 1.4.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# SOURCE_SUBDIR "-" (no CMakeLists there): fetch only, we compile what we need.
FetchContent_Declare(ompsdk
    GIT_REPOSITORY https://github.com/openmultiplayer/open.mp-sdk
    GIT_TAG 08cc69a32d96c0eb8fe063d26c78259e54ca853f)
FetchContent_Declare(pawnnatives
    GIT_REPOSITORY https://github.com/openmultiplayer/pawn-natives
    GIT_TAG 68f34b4dffdf5568e56437f8fdeec9408361373f
    SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/inc/pawn-natives     # included as <pawn-natives/...>
    SOURCE_SUBDIR -)
FetchContent_Declare(pawnamx
    GIT_REPOSITORY https://github.com/openmultiplayer/compiler
    GIT_TAG 5904ab8eac2a24706f303422cf1c307b319f76f0
    GIT_SUBMODULES ""
    SOURCE_SUBDIR -)
FetchContent_Declare(recast
    GIT_REPOSITORY https://github.com/recastnavigation/recastnavigation
    GIT_TAG 6dc1667f580357e8a2154c28b7867bea7e8ad3a7          # v1.6.0
    SOURCE_SUBDIR -)
FetchContent_MakeAvailable(ompsdk pawnnatives pawnamx recast)

# Only the Recast and Detour libraries, as static code inside our binaries.
file(GLOB RECAST_SRC ${recast_SOURCE_DIR}/Recast/Source/*.cpp)
file(GLOB DETOUR_SRC ${recast_SOURCE_DIR}/Detour/Source/*.cpp)
add_library(dnav_recast STATIC ${RECAST_SRC})
target_include_directories(dnav_recast PUBLIC ${recast_SOURCE_DIR}/Recast/Include)
add_library(dnav_detour STATIC ${DETOUR_SRC})
target_include_directories(dnav_detour PUBLIC ${recast_SOURCE_DIR}/Detour/Include)
# 64-bit polygon references: the whole map is one tiled mesh some day, and 32-bit
# refs run out of tile bits long before San Andreas does. Baker and component
# must agree (the tile data's link size depends on it).
target_compile_definitions(dnav_detour PUBLIC DT_POLYREF64=1)

# What the baker and the component share: the .nav file, the settings and the search.
# The built-in airports (airports/gta_sa.txt), compiled in as bytes.
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/airports/gta_sa.txt DNAV_GTA_AIRPORTS HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," DNAV_GTA_AIRPORTS "${DNAV_GTA_AIRPORTS}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/airports_gta.cpp
    "// Made by CMake from airports/gta_sa.txt.\nnamespace dnav {\nstatic const unsigned char gtaSa[] = { ${DNAV_GTA_AIRPORTS} 0 };\nextern const char* const AIRPORTS_GTA_SA = reinterpret_cast<const char*>(gtaSa);\n}\n")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/airports/gta_sa.txt)

add_library(dnav_common STATIC src/navfile.cpp src/pathfinder.cpp src/roads.cpp src/driving.cpp src/collision.cpp src/flying.cpp
    src/aircraft.cpp src/airports.cpp src/atc.cpp src/pilot.cpp ${CMAKE_CURRENT_BINARY_DIR}/airports_gta.cpp)
target_include_directories(dnav_common PUBLIC src)
find_package(Threads REQUIRED)
target_link_libraries(dnav_common PUBLIC dnav_detour Threads::Threads)

add_executable(navbake src/navbake.cpp)
target_link_libraries(navbake PRIVATE dnav_common dnav_recast dnav_detour)

add_executable(navquery src/navquery.cpp)
target_link_libraries(navquery PRIVATE dnav_common)

add_library(DiamondNav SHARED src/component.cpp)
set_target_properties(DiamondNav PROPERTIES PREFIX "")
target_include_directories(DiamondNav PRIVATE
    ${CMAKE_BINARY_DIR}/_deps/inc
    ${pawnnatives_SOURCE_DIR}
    ${pawnamx_SOURCE_DIR}/source
    ${pawnamx_SOURCE_DIR}/source/linux)
target_compile_definitions(DiamondNav PRIVATE HAVE_STDINT_H=1 PAWN_CELL_SIZE=32
    DNAV_VERSION_MAJOR=${PROJECT_VERSION_MAJOR} DNAV_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    DNAV_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_link_libraries(DiamondNav PRIVATE OMP-SDK dnav_common)
if(UNIX)
    # Ship one file: no libstdc++ version the VPS might not have.
    target_link_options(DiamondNav PRIVATE -static-libstdc++ -static-libgcc)
    target_link_options(navbake PRIVATE -static-libstdc++ -static-libgcc)
    target_link_options(navquery PRIVATE -static-libstdc++ -static-libgcc)
endif()
if(MSVC)
    set_property(TARGET DiamondNav navbake navquery dnav_common dnav_recast dnav_detour PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        # A whole-world bake holds every triangle and tile at once: 4 GB, not 2.
        target_link_options(navbake PRIVATE /LARGEADDRESSAWARE)
        target_link_options(navquery PRIVATE /LARGEADDRESSAWARE)
    endif()
endif()

option(DNAV_TESTS "Build the C++ tests" ON)
if(DNAV_TESTS)
    enable_testing()
    add_executable(dnav_tests tests/test_roads.cpp)
    target_link_libraries(dnav_tests PRIVATE dnav_common)
    add_test(NAME roads COMMAND dnav_tests)
    add_executable(dnav_driving_tests tests/test_driving.cpp)
    target_link_libraries(dnav_driving_tests PRIVATE dnav_common)
    add_test(NAME driving COMMAND dnav_driving_tests)
    add_executable(dnav_collision_tests tests/test_collision.cpp)
    target_link_libraries(dnav_collision_tests PRIVATE dnav_common)
    add_test(NAME collision COMMAND dnav_collision_tests)
    add_executable(dnav_flying_tests tests/test_flying.cpp)
    target_link_libraries(dnav_flying_tests PRIVATE dnav_common)
    add_test(NAME flying COMMAND dnav_flying_tests)
    add_executable(dnav_aircraft_tests tests/test_aircraft.cpp)
    target_link_libraries(dnav_aircraft_tests PRIVATE dnav_common)
    add_test(NAME aircraft COMMAND dnav_aircraft_tests)
    if(MSVC)
        # The static runtime, as dnav_common is built with.
        set_property(TARGET dnav_tests dnav_driving_tests dnav_collision_tests dnav_flying_tests dnav_aircraft_tests PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif()
endif()
