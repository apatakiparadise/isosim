project(Isosim)

cmake_minimum_required(VERSION 2.8.6)

# Settings.
# ---------
set(TARGET isosimTest CACHE TYPE STRING)
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")

# Find and hook up to OpenSim.
# ----------------------------
# OpenSim uses C++11 language features.
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

set(ROSBRIDGECPP_SRC_DIR "/home/uqmvale6/rosbridgecpp")
message(STATUS ${ROSBRIDGECPP_SRC_DIR})

include_directories(${ROSBRIDGECPP_SRC_DIR})
add_subdirectory(${ROSBRIDGECPP_SRC_DIR} rosbridgecpp)


########################
# include isosim includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)


# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES src/*.h src/*.cpp )

add_executable(${TARGET} ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})
target_link_libraries(${TARGET} rosbridgecpp)



#copy osim models to Models folder in running directory
file(GLOB OSIM_MODELS src/Models/*.osim)

file(MAKE_DIRECTORY build/Models)
#add_custom_command(
#    TARGET ${TARGET}
#    COMMAND ${CMAKE_COMMAND}
#    ARGS -E make_directory
#    ${CMAKE_BINARY_DIR}/Models
#    )   


foreach(osimModel ${OSIM_MODELS})
    get_filename_component(osimModelName, ${osimModel}, NAME)
    message(STATUS ${osimModel})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${osimModel}
        ${CMAKE_BINARY_DIR}/Models/${osimModelName})
endforeach(osimModel)


# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
file(MAKE_DIRECTORY build/Models/Geometry)
file(GLOB DATA_FILES src/Models/Geometry/*.vtp src/Models/Geometry/*.obj)
foreach(dataFile ${DATA_FILES})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${CMAKE_BINARY_DIR}/Models/Geometry)
endforeach(dataFile)

