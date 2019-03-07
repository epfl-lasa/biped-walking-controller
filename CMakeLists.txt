project(BipedWalkingGrasping)
cmake_minimum_required(VERSION 2.8)

find_package(YARP)
find_package(ICUB)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface 0.2.2 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(Curses REQUIRED)

find_library(libqpOASES NAMES libqpOASES.a PATHS "/usr/local/lib/")
list(APPEND CMAKE_MODULE_PATH "/usr/local/include/eigen3")

set(SRC_FOLDER      "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(HEADERS_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
set(qpOASES_DIR "/home/nbfigueroa/yarp_ws/qpOASES-3.2.1")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${qpOASES_DIR}/include)

set(HEADERS    ${HEADERS_FOLDER}/InitBalWlkParameters.h
               ${HEADERS_FOLDER}/CpMath_Utilities.h
               ${HEADERS_FOLDER}/CommunicationControl.h
               ${HEADERS_FOLDER}/RobotModel.h
               ${HEADERS_FOLDER}/TemplateModels.h
               ${HEADERS_FOLDER}/MPCOptimizer.h
               ${HEADERS_FOLDER}/PatternsGenerator.h
               ${HEADERS_FOLDER}/EstimatorCompensators.h
               ${HEADERS_FOLDER}/Data_logging.h
               ${HEADERS_FOLDER}/ReferencesCompensator.h
               ${HEADERS_FOLDER}/Grasping.h
               ${HEADERS_FOLDER}/OptimalFilters.h
               ${HEADERS_FOLDER}/CpBalWlkCtrlThread.h
               ${HEADERS_FOLDER}/ArmsForceTorqueKinChain.h
               ${HEADERS_FOLDER}/ArmsForceTorqueSensorsChains.h
               ${HEADERS_FOLDER}/QPOasesSolver.hpp)


set(SOURCES    ${SRC_FOLDER}/InitBalWlkParameters.cpp
               ${SRC_FOLDER}/CommunicationControl.cpp
               ${SRC_FOLDER}/RobotModel.cpp
               ${SRC_FOLDER}/TemplateModels.cpp
               ${SRC_FOLDER}/MPCOptimizer.cpp
               ${SRC_FOLDER}/PatternsGenerator.cpp
               ${SRC_FOLDER}/EstimatorCompensators.cpp
               ${SRC_FOLDER}/Data_logging.cpp            
               ${SRC_FOLDER}/ReferencesCompensator.cpp
               ${SRC_FOLDER}/Grasping.cpp
               ${SRC_FOLDER}/CpBalWlkCtrlThread.cpp)

# Defining variable and creating executables for COM Desired Motion planning
set(COM_SRC_FOLDER      "${CMAKE_CURRENT_SOURCE_DIR}/desired-CoM-vel-cmd/src")
set(COM_HEADERS_FOLDER  "${CMAKE_CURRENT_SOURCE_DIR}/desired-CoM-vel-cmd/include")

set(COM_WALKER_SRC      main_walker.cpp)
set(COM_WALKER_ROS_SRC  main_walker_ros.cpp)
set(COM_WALKER_LPVDS_SRC  main_walker_lpvDS.cpp)
set(COM_KEYBOARD_SRC    main_keyboard.cpp)

source_group("Source Files" FILES ${SOURCES} ${COM_SOURCES})
source_group("Header Files" FILES ${HEADERS} ${COM_HEADERS})

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${CURSES_INCLUDE_DIR}
                    ${HEADERS_FOLDER}
                    ${COM_HEADERS_FOLDER}
                    ${LPVDS_INCLUDE_DIRS} )

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_definitions(-D_USE_MATH_DEFINES)

# Executable for standalone walking controller (with fixed velocity/keyboard input/linear DS)
add_executable(${PROJECT_NAME} ${COM_WALKER_SRC} ${SOURCES} ${HEADERS} ${COM_SRC_FOLDER}/DesVelocityCommand.cpp ${COM_HEADERS_FOLDER}/DesVelocityCommand.h)
target_link_libraries(${PROJECT_NAME} ${libqpOASES} iKin ${YARP_LIBRARIES} ${yarpWholeBodyInterface_LIBRARIES} ${CURSES_LIBRARIES})

# Executable for standalone walking controller with DS motion generator through ROS
add_executable(${PROJECT_NAME}_ROS ${COM_WALKER_ROS_SRC} ${SOURCES} ${HEADERS} ${COM_SRC_FOLDER}/DesVelocityCommandROS.cpp ${COM_HEADERS_FOLDER}/DesVelocityCommandROS.h)
target_link_libraries(${PROJECT_NAME}_ROS ${libqpOASES} iKin ${YARP_LIBRARIES} ${yarpWholeBodyInterface_LIBRARIES} ${CURSES_LIBRARIES})


# Executable for standalone walking controller with DS motion generator via LPVDS class
add_executable(${PROJECT_NAME}_LPVDS ${COM_WALKER_LPVDS_SRC} ${SOURCES} ${HEADERS} ${COM_SRC_FOLDER}/DesVelocityCommand_lpvDS.cpp ${COM_HEADERS_FOLDER}/DesVelocityCommand_lpvDS.h)
target_link_libraries(${PROJECT_NAME}_LPVDS ${libqpOASES} iKin ${YARP_LIBRARIES} ${yarpWholeBodyInterface_LIBRARIES} ${CURSES_LIBRARIES} lpvDS)

# Executable for the keyboard command reader
add_executable(KeyboardReaderSender ${COM_KEYBOARD_SRC} ${COM_SRC_FOLDER}/KeyboardReaderSender.cpp ${COM_HEADERS_FOLDER}/KeyboardReaderSender.h)
target_link_libraries(KeyboardReaderSender  ${YARP_LIBRARIES} ${CURSES_LIBRARIES})
