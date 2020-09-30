cmake_minimum_required(VERSION 3.1)
project(example)
set(CMAKE_CXX_STANDARD 11)

find_package(CGAL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

# Add project files
file(GLOB SRCFILES *.cpp)
file(GLOB HFILES *.h)

###add_library(${PROJECT_NAME} SHARED  $<TARGET_OBJECTS:objlib>)
add_library(${PROJECT_NAME} SHARED ${SRCFILES} ${HFILES})
target_compile_definitions(${PROJECT_NAME} PUBLIC CGAL_PMP_USE_CERES_SOLVER )
target_link_libraries(${PROJECT_NAME} CGAL::CGAL ceres Eigen3::Eigen)
## End of the workaround