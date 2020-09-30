cmake_minimum_required(VERSION 3.1)
project(example)
set(CMAKE_CXX_STANDARD 11)

find_package(CGAL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# Add project files
file(GLOB SRCFILES *.cpp)
file(GLOB HFILES *.h)

add_library(${PROJECT_NAME} SHARED ${SRCFILES} ${HFILES})
target_link_libraries(${PROJECT_NAME} CGAL::CGAL Eigen3::Eigen)
## End of the workaround