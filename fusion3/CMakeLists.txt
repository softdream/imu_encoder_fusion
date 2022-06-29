CMAKE_MINIMUM_REQUIRED( VERSION 3.21.1 )

ADD_DEFINITIONS( -std=c++14 )

SET( EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin" )

#find required eigen
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

#find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

#directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src src_file )
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR} src_file )

INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include" )

#directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})



ADD_EXECUTABLE( encoder_imu_fusion ${src_file} )

TARGET_LINK_LIBRARIES( encoder_imu_fusion ${OpenCV_LIBS} )                                                 
