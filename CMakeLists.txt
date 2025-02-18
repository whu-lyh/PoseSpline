cmake_minimum_required(VERSION 3.0.0)
project(PoseSpline)

set(cmake_build_type Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

cs_add_library(${PROJECT_NAME}
        src/extern/spline_projection_error_simple.cpp
        src/extern/spline_projection_error.cpp
        src/extern/spline_projection_error1.cpp
        src/extern/spline_projection_error2.cpp
        src/extern/spline_projection_error3.cpp
        src/extern/spline_projection_error4.cpp
        src/extern/pinhole_project_error.cpp
        src/extern/project_error.cpp
        src/extern/RotateVectorError.cpp
        src/extern/TransformVectorError.cpp
        src/internal/pose_local_parameterization.cpp
        src/internal/utility.cpp
        src/QuaternionSplineUtility.cpp
        src/QuaternionSpline.cpp
        src/QuaternionSplineSampleError.cpp
        src/PoseSplineSampleError.cpp
        src/PoseSplineUtility.cpp
        src/OmegaExtrinsicTemperalError.cpp
        src/QuaternionOmegaSampleError.cpp
        src/PoseSpline.cpp
        src/VectorSplineSampleAutoError.cpp
        src/VectorSplineSampleVelocityError.cpp
        src/Time.cpp
        src/Duration.cpp
        src/PoseSplineSampleTemporalError.cpp
        src/maplab/quaternion_param_jpl.cpp
#        src/PoseSpline/LinearAccelerateSampleError.cpp
#        src/PoseSpline/AngularVelocitySampleError.cpp
        )

## Test
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(geometry_tests
            test/test_main.cpp
            test/PoseTest.cpp
            test/UtilityTest.cpp
            )
    if(TARGET geometry_tests)
        target_link_libraries(geometry_tests ${PROJECT_NAME} -lpthread -lm)
    endif()

    catkin_add_gtest(ceres_tests
            test/test_main.cpp
#            test/QuaternionCeresTest.cpp
#            test/testProjectError.cpp
#            test/testSplineRotateError.cpp
            test/testSplineProjectError.cpp
#            test/VectorSplineCeresTest.cpp
            )
    if(TARGET ceres_tests)
        target_link_libraries(ceres_tests ${PROJECT_NAME} -lpthread -lm)
    endif()

    catkin_add_gtest(spline_tests
            test/test_main.cpp
            test/SplineTest.cpp
            )
    if(TARGET spline_tests)
        target_link_libraries(spline_tests ${PROJECT_NAME} -lpthread -lm)
    endif()
endif()

add_executable(test_simulate_spline_project test/test_simulate_spline_project.cpp)
target_link_libraries(test_simulate_spline_project ${PROJECT_NAME} -lpthread -lm)

add_executable(test_simulate_project test/test_simulate_project.cpp)
target_link_libraries(test_simulate_project ${PROJECT_NAME} -lpthread -lm)

add_executable(test_vinsmono_imu_factor test/test_vinsmono_imu_factor.cpp)
target_link_libraries(test_vinsmono_imu_factor ${PROJECT_NAME}  -lpthread -lm)

add_executable(test_spline_imu_factor test/test_spline_imu_factor.cpp)
target_link_libraries(test_spline_imu_factor ${PROJECT_NAME}  -lpthread -lm)

add_executable(test_dynamic_spline_imu_factor test/test_dynamic_spline_imu_factor.cpp)
target_link_libraries(test_dynamic_spline_imu_factor ${PROJECT_NAME}  -lpthread -lm)

add_executable(test_JPL_imu_factor test/test_JPL_imu_factor.cpp)
target_link_libraries(test_JPL_imu_factor ${PROJECT_NAME}  -lpthread -lm)

add_executable(testPoseSplineSampleTemporalError test/testPoseSplineSampleTemporalError.cpp)
target_link_libraries(testPoseSplineSampleTemporalError ${PROJECT_NAME}  -lpthread -lm)

add_executable(testPoseSplineSampleError test/testPoseSplineSampleError.cpp)
target_link_libraries(testPoseSplineSampleError ${PROJECT_NAME}  -lpthread -lm)

cs_install()
cs_export()
