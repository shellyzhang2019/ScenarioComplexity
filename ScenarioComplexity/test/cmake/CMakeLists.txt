cmake_minimum_required(VERSION 3.10)
project(
    solver_test
)

# find_package(
#     catkin REQUIRED COMPONENTS
# )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" -std=c99)

set(BASE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/math/solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Vehicle.cpp)

set(PYTHON_INCLUDE_DIRS /usr/include/python3.6m)
set(PYTHON_LIBRARIES /usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(/usr/include/eigen3)
include_directories(/usr/include/Ipopt-3.12.7)
include_directories(/usr/include/coin)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include/math)

# add_library(Control SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Control.cpp)
# add_library(solver SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../../src/math/solver.cpp)
# add_library(Vehicle SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Vehicle.cpp)


add_executable(solver_test
/home/plusai/PlusDrive/ScenarioComplexity/src/Control.cpp
/home/plusai/PlusDrive/ScenarioComplexity/src/math/solver.cpp
/home/plusai/PlusDrive/ScenarioComplexity/src/Vehicle.cpp
/home/plusai/PlusDrive/ScenarioComplexity/test/solver_test.cpp
)


target_link_libraries(solver_test
                      ${PYTHON_LIBRARIES})
# target_link_libraries(monte_carlo ${catkin_LIBRARIES})