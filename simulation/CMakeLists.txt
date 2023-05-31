cmake_minimum_required(VERSION 3.10)
project(samplingFoldingData)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)


include_directories(
        ${EIGEN_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
	)

add_executable(simBC
        findBC/main.cpp
        findBC/world.cpp
        environment/elasticRod.cpp
        environment/elasticStretchingForce.cpp
        environment/elasticBendingForce.cpp
        environment/elasticTwistingForce.cpp
        environment/externalGravityForce.cpp
        environment/dampingForce.cpp
        environment/inertialForce.cpp
        environment/timeStepper.cpp
        findBC/setInput.cpp
        )

add_executable(simF
        measureF/main.cpp
	      measureF/world.cpp
        environment/elasticRod.cpp
        environment/elasticStretchingForce.cpp
        environment/elasticBendingForce.cpp
        environment/elasticTwistingForce.cpp
        environment/externalGravityForce.cpp
        environment/dampingForce.cpp
        environment/inertialForce.cpp
        environment/timeStepper.cpp
	      measureF/setInput.cpp
        )



target_link_libraries(simBC
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        Eigen3::Eigen
        lapack
        gfortran
        pthread
        rt
        m)

target_link_libraries(simF
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        Eigen3::Eigen
        lapack
        gfortran
        pthread
        rt
        m)
