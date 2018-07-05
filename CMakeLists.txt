cmake_minimum_required(VERSION 3.2.0)
project(rrt)

include_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/lib)
find_package( OpenGL REQUIRED )
if(OPENGL_FOUND)
  include_directories(${OPENGL_INCLUDE_DIR})
  MESSAGE(STATUS "OpenGL_LIBRARIES is: " ${OPENGL_LIBRARIES})
endif()

find_package( GLUT REQUIRED )
if(GLUT_FOUND)
  include_directories(${GLUT_INCLUDE_DIR})
  MESSAGE(STATUS "GLUT_LIBRARIES is: " ${GLUT_LIBRARIES})
endif()

# note: the -fPIC flag is for building with Qt...
#       it complains about position-independent code otherwise
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# includes
# flann library for KD-Tree data structure
find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)

set(RRT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE PATH "Directory containing includes for the RRT")
include_directories(${RRT_INCLUDE_DIR})

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# Opencv - used for drawing rrt
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "${OpenCV_INCLUDE_DIRS}")

# debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# libRRT
################################################################################

set(lib_SRC
    "src/2dplane/GridStateSpace.cpp"
    "src/2dplane/ObstacleGrid.cpp"
)

# add_library("RRT" STATIC ${lib_SRC})
add_library("RRT" ${lib_SRC})
add_dependencies(RRT Flann)
# Fix 'LZ4 not found' issue on some systems
target_link_libraries(RRT ${FLANN_LIBRARIES})


# copy headers to install directory
set(lib_HEADERS
    "2dplane/StateSpace.hpp"
    "2dplane/GridStateSpace.hpp"
    "2dplane/ObstacleGrid.hpp"
    "2dplane/PlaneStateSpace.hpp"
    "rrt/BiRRT.hpp"
    "rrt/Path.hpp"
    "rrt/Tree.hpp"
#    "util.hpp"
)
foreach(file ${lib_HEADERS})
    get_filename_component(dir ${file} DIRECTORY)
    install(FILES src/rrt/${file} DESTINATION include/rrt/${dir})
endforeach()

# copy library to install directory
install(TARGETS RRT DESTINATION "lib")

# draw rrt by opencv
set(ui_SRC
    "src/rrt_drawing/main.cpp"
    "src/rrt_drawing/rrt_drawing.cpp"
)

add_executable("rrt_drawing" ${ui_SRC})
target_link_libraries("rrt_drawing" RRT)
target_link_libraries("rrt_drawing" ${OpenCV_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} libSOIL.a)
