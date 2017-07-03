project(PCLref)
cmake_minimum_required(VERSION 2.8)

# a macro to conveniently add sources
macro(add_sources)
    file(RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_src ${ARGN})
        if(_relPath)
            list (APPEND SRCS "${_relPath}/${_src}")
        else()
            list (APPEND SRCS "${_src}")
        endif()
    endforeach()
    if (_relPath)
        # propagate SRCS to parent directory
        set (SRCS ${SRCS} PARENT_SCOPE)
    endif()
endmacro()

# a macro to create and link my executables
macro (add_my_executable _name _proj _path)
  add_executable(${_name} ${_path}/${_proj}.cc ${SRCS})
  target_link_libraries(${_name} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
endmacro()

# set some CXX Flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -std=c++11 -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
#set( CMAKE_VERBOSE_MAKEFILE on )

# add BOOST stuff
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
set(MyBoost_INCD ${Boost_INCLUDE_DIR})
set(MyBoost_LIBD ${Boost_LIBRARY_DIR})
set(MyBoost_LIB  ${Boost_LIBRARIES})

# add OpenCV stuff
#find_package(OpenCV REQUIRED)

# add PCL stuff
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${MyBoost_INCD})
link_directories(${PCL_LIBRARY_DIRS} ${MyBoost_LIBD})
add_definitions(${PCL_DEFINITIONS})

message("pcl-incl> " ${PCL_INCLUDE_DIRS})
message("pcl-libs> " ${PCL_LIBRARY_DIRS})
message("pcl-def> " ${PCL_DEFINITIONS})

set(Boost_LIBRARIES ${Boost_LIBRARIES} ${MyBoost_LIB})

message("boost-incl> " ${Boost_INCLUDE_DIR})
message("boost-libd> " ${Boost_LIBRARY_DIR})
message("boost-libs> " ${Boost_LIBRARIES})
message("myboostlib> " ${MyBoost_LIB})

include_directories(core pcl_extensions pipeline point_cloud_processing quality_measures features keypoints correspondences transformations)
add_subdirectory(core)
add_subdirectory(pcl_extensions)
add_subdirectory(pipeline)
add_subdirectory(point_cloud_processing)
add_subdirectory(quality_measures)
add_subdirectory(features)
add_subdirectory(keypoints)
add_subdirectory(correspondences)
add_subdirectory(transformations)

add_subdirectory(tools)

aux_source_directory(. SRC)

#add_my_executable (test1 tests)
#add_my_executable (Test2 tests)
add_my_executable (pclref main .)
#add_my_executable (viewer tools)


