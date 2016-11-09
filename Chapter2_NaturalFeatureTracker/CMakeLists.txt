cmake_minimum_required(VERSION 3.3)
project(NaturalFeatureTracking)

set(CMAKE_CXX_STANDARD 11)

option(USE_OPENMP "Use OpenMP to for multithread processing" ON)

if (MSVC)
    option(LINK_STATIC_CRT "Link static version of CRT instead dynamic one" ON)

    if (LINK_STATIC_CRT)
        #We statically link to reduce dependancies
        foreach (flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
            if (${flag_var} MATCHES "/MD")
                string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
            endif (${flag_var} MATCHES "/MD")
            if (${flag_var} MATCHES "/MDd")
                string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
            endif (${flag_var} MATCHES "/MDd")
        endforeach (flag_var)
    endif (LINK_STATIC_CRT)

endif (MSVC)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIBRARY_DIR})

# Fixes use of debug OpenCV modules for ReleaseWithDebug information build type
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
if (MSVC)
    set_target_properties(zlib libtiff libjpeg libwebp libpng PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
endif()

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIR})
link_directories(${OpenGL_LIBRARY_DIR})

add_subdirectory(src)
