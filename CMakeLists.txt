cmake_minimum_required(VERSION 2.8.3)
project(kinematic_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp cv_bridge image_transport urdf kdl_parser tf tf_conversions naoqi_msgs actionlib image_geometry message_generation eigen_conversions camera_calibration_parsers moveit_core moveit_ros_planning hrl_kinematics rosbag ar_sys)
find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# OpenMP 
find_package(OpenMP)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alerror
    alvalue
    )
    
find_package(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR}) 

SET(G2O_LIBS   	
	${G2O_CORE_LIBRARY}
  	${G2O_STUFF_LIBRARY}   	
  	${G2O_TYPES_SLAM3D} 
)


find_package(Boost QUIET COMPONENTS program_options signals filesystem)

find_package(LibXML2 REQUIRED)
set(XMLLIB ${LibXML2_LIBRARIES})


find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   measurementData.msg
   calibrationResult.msg
   hotJoint.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CmdPauseService.srv
  Capture.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include include/pose_generation
#  LIBRARIES kinematic_calibration
CATKIN_DEPENDS sensor_msgs message_runtime #roscpp cv_bridge image_transport urdf kdl_parser tf tf_conversions nao_msgs actionlib image_geometry rosbag
DEPENDS OpenMP eigen yaml-cpp gtest OpenMP
)


include_directories(include include/pose_generation ${LibXML2_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(kinematic_calibration
#   src/${PROJECT_NAME}/kinematic_calibration.cpp
# )

## Declare a cpp executable
# add_executable(kinematic_calibration_node src/kinematic_calibration_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(kinematic_calibration_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kinematic_calibration kinematic_calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinematic_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)




### Context
add_library(context 
	src/common/CalibrationContext.cpp
	src/common/CheckerboardContext.cpp
	src/common/CircleContext.cpp
	src/common/ContextFactory.cpp
	src/common/MarkerContext.cpp
	src/common/ArucoContext.cpp
	src/common/ColorMarkerContext.cpp
	src/optimization/CheckerboardMeasurementEdge.cpp
	src/optimization/CircleMeasurementEdge.cpp
	src/optimization/TransformationVertex.cpp
	)
     set_property(TARGET context PROPERTY COMPILE_DEFINITIONS PARAMSPATH="${PROJECT_SOURCE_DIR}/include/data_capturing/color_marker_detection/")

target_link_libraries(context 
	checkerboardDetection
	circleDetection
	arucoMarkerDetection
        colorMarkerDetection
	frameImageConverter
	kinematicChain
	${G2O_LIBS}
        ${orocos_kdl_LIBRARIES}
	${catkin_LIBRARIES})


### ImagePreprocessing
add_executable(imagePreprocessingNode src/data_capturing/ImagePreprocessingNode.cpp)
target_link_libraries(imagePreprocessingNode ${catkin_LIBRARIES} ${OpenCV_LIBS} ${orocos_kdl_LIBRARIES})


### CheckerboardDetection
add_library(checkerboardDetection src/data_capturing/CheckerboardDetection.cpp src/data_capturing/MarkerDetection.cpp src/data_capturing/SinglePointMarkerDetection.cpp src/data_capturing/checkerboard_detection/copyOfCalibinit.cpp)
target_link_libraries(checkerboardDetection frameImageConverter ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(checkerboardDetectionMain src/data_capturing/CheckerboardDetectionMain.cpp)
target_link_libraries(checkerboardDetectionMain checkerboardDetection ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_add_gtest(checkerboardDetectionTest test/CheckerboardDetectionTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(checkerboardDetectionTest checkerboardDetection ${catkin_LIBRARIES})


### CircleDetection
add_library(circleDetection src/data_capturing/CircleDetection.cpp src/data_capturing/MarkerDetection.cpp src/data_capturing/SinglePointMarkerDetection.cpp)
target_link_libraries(circleDetection frameImageConverter ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(circleDetectionMain src/data_capturing/CircleDetectionMain.cpp)
target_link_libraries(circleDetectionMain circleDetection ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_add_gtest(circleDetectionTest test/CircleDetectionTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(circleDetectionTest circleDetection ${catkin_LIBRARIES})


### ColorMarkerDetection
add_library(colorMarkerDetection src/data_capturing/ColorMarkerDetection.cpp src/data_capturing/MarkerDetection.cpp src/data_capturing/SinglePointMarkerDetection.cpp include/data_capturing/color_marker_detection/CMDetect.cpp)
target_link_libraries(colorMarkerDetection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${XMLLIB})

add_executable(colorMarkerDetectionMain src/data_capturing/ColorMarkerDetectionMain.cpp)
set_property(TARGET colorMarkerDetectionMain PROPERTY COMPILE_DEFINITIONS PARAMSPATH="${PROJECT_SOURCE_DIR}/include/data_capturing/color_marker_detection/")
target_link_libraries(colorMarkerDetectionMain colorMarkerDetection ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_add_gtest(colorMarkerDetectionTest test/ColorMarkerDetectionTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
set_property(TARGET colorMarkerDetectionTest PROPERTY COMPILE_DEFINITIONS PARAMSPATH="${PROJECT_SOURCE_DIR}/include/data_capturing/color_marker_detection/")
target_link_libraries(colorMarkerDetectionTest colorMarkerDetection ${catkin_LIBRARIES} ${OpenCV_LIBS})

## ArucoMarkerDetection
add_library(arucoMarkerDetection src/data_capturing/ArucoMarkerDetection.cpp src/data_capturing/MarkerDetection.cpp src/data_capturing/SinglePointMarkerDetection.cpp)
target_link_libraries(arucoMarkerDetection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})

add_executable(arucoMarkerDetectionMain src/data_capturing/ArucoMarkerDetectionMain.cpp)
target_link_libraries(arucoMarkerDetectionMain arucoMarkerDetection ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_add_gtest(arucoMarkerDetectionTest test/ArucoMarkerDetectionTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(arucoMarkerDetectionTest arucoMarkerDetection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})



### KinematicChain
add_library(kinematicChain src/common/KinematicChain.cpp) 
target_link_libraries(kinematicChain ${catkin_LIBRARIES})

#add_executable(kinematicChainTfComparison src/common/KinematicChainTfComparison.cpp)
#target_link_libraries(kinematicChainTfComparison kinematicChain modelLoader ${catkin_LIBRARIES})

catkin_add_gtest(kinematicChainTest test/KinematicChainTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(kinematicChainTest kinematicChain modelLoader frameImageConverter ${catkin_LIBRARIES})


### ModelLoader
add_library(modelLoader src/common/ModelLoader.cpp) 
target_link_libraries(modelLoader ${catkin_LIBRARIES})

catkin_add_gtest(modelLoaderTest test/ModelLoaderTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(modelLoaderTest modelLoader ${catkin_LIBRARIES})


### FrameImageConverter
add_library(frameImageConverter src/common/FrameImageConverter.cpp)
target_link_libraries(frameImageConverter ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(frameImageConverterMain src/common/FrameImageConverterMain.cpp)
#target_link_libraries(frameImageConverterMain frameImageConverter modelLoader kinematicChain ${catkin_LIBRARIES} ${OpenCV_LIBS})


### ErrorModel
add_library(errorModel 
	src/common/ErrorModel.cpp 
	src/optimization/CalibrationState.cpp
	src/optimization/CameraIntrinsicsVertex.cpp
	src/optimization/JointOffsetVertex.cpp
	src/optimization/TransformationVertex.cpp
	src/common/MeasurementPose.cpp
	)
target_link_libraries(errorModel modelLoader frameImageConverter kinematicChain ${catkin_LIBRARIES} ${G2O_LIBS})

catkin_add_gtest(errorModelTest test/ErrorModelTest.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(errorModelTest errorModel ${catkin_LIBRARIES})


## PoseSampling 
add_library(poseSampling src/pose_generation/PoseSampling.cpp )
target_link_libraries(poseSampling errorModel modelLoader frameImageConverter helpers kinematicChain ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(poseSamplingMain src/pose_generation/PoseSamplingMain.cpp)
target_link_libraries(poseSamplingMain poseSampling)




# add_library(poseSelection
#    src/pose_generation/PoseSelectionNode.cpp
#    src/pose_generation/PoseSource.cpp
#    src/pose_generation/ValidationPoseSelectionNode.cpp
#    src/pose_generation/ValidationPoseSource.cpp
#    src/pose_generation/ObservabilityIndex.cpp
#    #src/pose_generation/PoseSampling.cpp
#    src/pose_generation/PoseSelectionStrategy.cpp
#    src/common/PoseSet.cpp
#    src/common/MeasurementPose.cpp
#    src/optimization/CalibrationState.cpp
#    )


#target_link_libraries(poseSelection ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})

### OnlineCalibration
add_executable(onlineCalibration 
   #src/calibration/CalibrationFramework.cpp
        src/calibration/OnlineCalibration.cpp
        )

target_link_libraries(onlineCalibration ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} validationNodeLib )

     add_executable(dataCaptureService 
        src/data_capturing/DataCapturingService.cpp
    )

 #add_library(modelLoader src/common/ModelLoader.cpp) 
 #target_link_libraries(modelLoader ${catkin_LIBRARIES})
 target_link_libraries(dataCaptureService ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} dataCapture modelLoader )


add_executable(validationOfflineCalibration src/calibration/ValidationOfflineCalibration.cpp
        )
target_link_libraries(validationOfflineCalibration ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} validationNodeLib)


#TODO: Re-enable this
#add_executable(testPoseSet
   #src/calibration/CalibrationFramework.cpp
   #   src/test/testPoseSet.cpp
   #     )
     #     target_link_libraries(testPoseSet ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} validationNodeLib )


### MeasurementToPose
add_executable(measurementToPoseNode
	src/common/MeasurementToPoseNode.cpp
	)
	
add_dependencies(measurementToPoseNode ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(measurementToPoseNode ${catkin_LIBRARIES})


### MeasurementImageExtractionNode
add_executable(measurementImageExtractionNode
	src/common/MeasurementImageExtractionNode.cpp
	)
	
  add_dependencies(measurementImageExtractionNode ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(measurementImageExtractionNode ${catkin_LIBRARIES} ${OpenCV_LIBS})


### DataCapture
add_library(dataCapture 
	src/data_capturing/DataCapture.cpp
	src/data_capturing/PauseManager.cpp
	)
target_link_libraries(dataCapture checkerboardDetection circleDetection context ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(dataCaptureMain src/data_capturing/DataCaptureMain.cpp)
target_link_libraries(dataCaptureMain dataCapture context modelLoader kinematicChain ${catkin_LIBRARIES} ${OpenCV_LIBS} ${orocos_kdl_LIBRARIES})


catkin_add_gtest(cameraIntrinsicsVertexTest test/CameraIntrinsicsVertexTest.cpp src/optimization/CameraIntrinsicsVertex.cpp)
target_link_libraries(cameraIntrinsicsVertexTest ${catkin_LIBRARIES} ${G2O_LIBS})

catkin_add_gtest(transformationVertexTest test/TransformationVertexTest.cpp src/optimization/TransformationVertex.cpp)
target_link_libraries(transformationVertexTest ${catkin_LIBRARIES} ${G2O_LIBS})

catkin_add_gtest(jointOffsetVertexTest test/JointOffsetVertexTest.cpp src/optimization/JointOffsetVertex.cpp)
target_link_libraries(jointOffsetVertexTest kinematicChain modelLoader ${catkin_LIBRARIES} ${G2O_LIBS})

## helpers
add_library(helpers src/common/helpers.cpp)
	
if(gsl_FOUND)
### ValidationNode

add_library(validationNodeLib 
	src/optimization/CalibrationState.cpp
	src/optimization/ValidationNode.cpp 
        src/optimization/OptimizationInterface.cpp 
	src/optimization/OptimizationNode.cpp 
	src/pose_generation/PoseSource.cpp
        src/pose_generation/PoseSamplingPoseSource.cpp
        src/pose_generation/FilePoseSource.cpp
        src/pose_generation/MeasurementMsgPoseSource.cpp
        #src/pose_generation/PoseSampling.cpp
        src/pose_generation/ValidationPoseSource.cpp
	src/pose_generation/ObservabilityIndex.cpp
	src/pose_generation/PoseSelectionStrategy.cpp
        src/pose_generation/DetmaxPoseSelectionStrategy.cpp
	src/optimization/JointOffsetOptimization.cpp
	src/optimization/G2oJointOffsetOptimization.cpp
	src/optimization/CameraIntrinsicsVertex.cpp
	src/optimization/JointOffsetVertex.cpp
	src/optimization/TransformationVertex.cpp
	#src/optimization/MeasurementEdge.cpp
	#src/optimization/CheckerboardMeasurementEdge.cpp
	#src/optimization/CircleMeasurementEdge.cpp
	src/optimization/LocOptKinCalState.cpp
	src/optimization/LocOptJointOffsetOptimization.cpp
	src/common/MeasurementPose.cpp
	src/common/PoseSet.cpp
	src/result_publishing/CameraTransformUpdate.cpp
	src/result_publishing/UrdfUpdate.cpp	
	src/result_publishing/JointUpdate.cpp
        src/common/sample.cpp
        )

target_link_libraries(validationNodeLib
   frameImageConverter
   modelLoader
   kinematicChain
   context
   helpers
   ${YAML_CPP_LIBRARIES}
   ${catkin_LIBRARIES} 
   ${G2O_LIBS}
   ${gsl_LIBRARIES}
   )

endif()

### PoseSelection
# add_executable(poseSelectionNode
# 	src/common/PoseSet.cpp
# 	src/pose_generation/ObservabilityIndex.cpp
# 	src/pose_generation/PoseSelectionNode.cpp
# 	src/pose_generation/PoseSelectionStrategy.cpp
# 	src/pose_generation/PoseSelectionNodeMain.cpp
# 	src/pose_generation/PoseSource.cpp
#         src/data_capturing/DataCapture.cpp
#         src/data_capturing/PauseManager.cpp
#         #src/common/sample.cpp
# 	)
# 	
# target_link_libraries(poseSelectionNode poseSampling errorModel ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} validationNodeLib )

### ValidationPoseSelection
#add_executable(validationPoseSelectionNode
#	src/common/PoseSet.cpp
#	src/pose_generation/ObservabilityIndex.cpp
#	src/pose_generation/PoseSelectionNode.cpp
#	src/pose_generation/ValidationPoseSelectionNode.cpp
#        src/pose_generation/ValidationPoseSource.cpp
#	src/pose_generation/PoseSelectionStrategy.cpp
#	src/pose_generation/PoseSource.cpp
        #src/common/sample.cpp
        #)
	
     #     target_link_libraries(validationPoseSelectionNode poseSampling errorModel ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} validationNodeLib)


### updateNode
add_executable(updateNode 
	src/result_publishing/UpdateNode.cpp 
	src/result_publishing/CameraTransformUpdate.cpp
	src/result_publishing/UrdfUpdate.cpp	
	src/result_publishing/JointUpdate.cpp
	)
add_dependencies(updateNode ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(updateNode modelLoader ${catkin_LIBRARIES})

catkin_add_gtest(jointUpdateTest test/JointUpdateTest.cpp src/result_publishing/JointUpdate.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(jointUpdateTest modelLoader ${catkin_LIBRARIES})

### Temperature
if( ${NAOqi_FOUND} ) 
    message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
        include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS})

    add_executable(temperatureNode src/data_capturing/TemperatureNode.cpp)
    target_link_libraries(temperatureNode
        ${catkin_LIBRARIES}
        ${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES}
	)
     add_dependencies(temperatureNode ${PROJECT_SOURCE_DIR}_generate_messages_cpp)
    add_dependencies(temperatureNode ${catkin_EXPORTED_TARGETS})

else()
    message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()

