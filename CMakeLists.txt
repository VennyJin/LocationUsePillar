cmake_minimum_required(VERSION 2.8.11)

####### Project name ######
set(PROJECT_NAME LocationUsePillar)
project(${PROJECT_NAME})
###########################

add_compile_options(-std=c++11)

################ PATH DEFINATION ####################

set(WORKSPACE_PATH "${CMAKE_CURRENT_LIST_DIR}/..")
set(CL_OUTPUT_DIRECTORY ${WORKSPACE_PATH}/output)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CL_OUTPUT_DIRECTORY})

#####################################################


################# INCLUDE DIRECTORIES ##################
find_package(OpenCV REQUIRED)

include_directories(
    .
    .
    ${CMAKE_CURRENT_LIST_DIR}/include
    /usr/include/ni
    #/usr/include/eigen3
    )

########################################################

############# Compiler options ##############
add_definitions(-O2)
add_definitions(-Wall)
# add_definitions(-DNDEBUG)
# set(CMAKE_BUILD_TYPE "Debug")


# Linker options
set(LINK_OPTIONS -lpthread)
#set(LINK_OPTIONS -lrealsense2)


# Library path
set(CMAKE_LDFLAGS "${CMAKE_LDFLAGS} -L. ")

####################OpenNI######################
FIND_PATH(OpenNI2_INCLUDE_DIRS OpenNI.h HINTS  $ENV{OPENNI2_INCLUDE} PATH_SUFFIXES openni2)
FIND_LIBRARY(OpenNI2_LIBRARY NAMES OpenNI2 HINTS  $ENV{OPENNI2_LIB} $ENV{OPENNI2_REDIST})
include_directories( ${OpenNI2_INCLUDE_DIRS} )
IF (OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARY)
SET(OpenNI2_FOUND TRUE)
ENDIF (OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARY)

IF (OpenNI2_FOUND)
# show which OpenNI2 was found only if not quiet
SET(OpenNI2_LIBRARIES ${OpenNI2_LIBRARY})
MESSAGE(STATUS "Found OpenNI2: ${OpenNI2_LIBRARIES}")
ELSE (OpenNI2_FOUND)
# fatal error if OpenNI2 is required but not found
IF (OpenNI2_FIND_REQUIRED)
MESSAGE(FATAL_ERROR "Could not find OpenNI2. Environment variables OPENNI2_INCLUDE (directory containing OpenNI.h) and OPENNI2_LIB (directory containing OpenNI2 library) could bet set.")
ENDIF (OpenNI2_FIND_REQUIRED)
ENDIF (OpenNI2_FOUND)
#################################################


###################Define the CXX sources#####################
set ( CXX_SRCS
	${CMAKE_CURRENT_LIST_DIR}/src/ImageProcess.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Featurefit.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/CoordinateTrans.cpp
    #${CMAKE_CURRENT_LIST_DIR}/Comlib_Linux/src/SerialCom.cpp
    #${CMAKE_CURRENT_LIST_DIR}/Comlib_Linux/src/com_lib.cpp
    #${CMAKE_CURRENT_LIST_DIR}/Comlib_Linux/src/com_execute.cpp
    #${CMAKE_CURRENT_LIST_DIR}/Comlib_Linux/src/com_command.cpp
    )
##############################################################

set_source_files_properties(
    ${CXX_SRCS} PROPERTIES COMPILE_FLAGS
    " -O2 -Wall")


add_executable(${PROJECT_NAME} ${CXX_SRCS})


target_link_libraries(${PROJECT_NAME} ${LINK_OPTIONS})


target_link_libraries(${PROJECT_NAME}
    ${OpenNI2_LIBRARIES}
    ${OpenCV_LIBS}
#    /usr/lib/x86_64-linux-gnu/librt.so
#    /usr/lib/arm-linux-gnueabihf/librt.so
    )

