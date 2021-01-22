cmake_minimum_required(VERSION 3.10)

project(VelocityManager)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

FIND_PACKAGE(GTest REQUIRED)
FIND_PACKAGE(Eigen3 3.3 REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

set(TARGET_ESTIMATOR TargetEstimator)
set(TEST_NAME ${PROJECT_NAME}_UnitTest)
set(UTILS Utilities)

add_library(${PROJECT_NAME} STATIC
    include/velocity_manager/velocity_manager.h
    src/velocity_manager/velocity_manager.cpp
    include/velocity_manager/velocity_manager_class.h
    src/velocity_manager/velocity_manager_class.cpp
    )
target_link_libraries(${PROJECT_NAME} ${UTILS} Eigen3::Eigen)

add_library(${TARGET_ESTIMATOR} STATIC
    include/target_estimator/target_estimator.h
    src/target_estimator/target_estimator.cpp
    )

add_library(${UTILS} STATIC
    include/utils/math.h
    utils/math.cpp
    )

add_executable(${TEST_NAME}
    tests/velocity_manager.cpp
    tests/target_estimator.cpp
    )
target_link_libraries(${TEST_NAME}  ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} ${PROJECT_NAME} ${TARGET_ESTIMATOR} pthread)
