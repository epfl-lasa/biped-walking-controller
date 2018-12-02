project(BipedWalkingGrasping)
cmake_minimum_required(VERSION 2.8)

find_package(YARP)
find_package(ICUB)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface 0.2.2 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(Curses REQUIRED)

find_library(libqpOASES NAMES libqpOASES.so PATHS "/home/icub/workspace/icubSources/codyco-superbuild/build/install/lib/")

list(APPEND CMAKE_MODULE_PATH "/usr/local/include/eigen3")

set(SRC_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(HEADERS_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(KEYBOARD_SRC ${CMAKE_CURRENT_SOURCE_DIR}/KeyboardCommandsReader/KeyboardCommandsReader.cpp)

set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
set(qpOASES_DIR "/home/icub/workspace/icubSources/codyco-superbuild/external/qpOASES")

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
               ${SRC_FOLDER}/CpBalWlkCtrlThread.cpp
               main.cpp)

source_group("Source Files" FILES ${SOURCES})
source_group("Header Files" FILES ${HEADERS})

#include_directories (${EIGEN_INCLUDE_DIR})
#include_directories (${qpOASES_INCLUDE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

include_directories(${CURSES_INCLUDE_DIR})

include_directories(${HEADERS_FOLDER})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_definitions(-D_USE_MATH_DEFINES)

# Executable for the walking controller
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${libqpOASES} iKin ${YARP_LIBRARIES} ${yarpWholeBodyInterface_LIBRARIES} ${CURSES_LIBRARIES})

# Executable for the keyboard reader
add_executable(KeyboardCommandsReader ${KEYBOARD_SRC})
target_link_libraries(KeyboardCommandsReader ${YARP_LIBRARIES} ${CURSES_LIBRARIES})