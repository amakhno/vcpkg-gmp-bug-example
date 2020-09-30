cmake_minimum_required(VERSION 3.1)
project(example)
set(CMAKE_CXX_STANDARD 11)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(GMP REQUIRED)

# Add project files
file(GLOB SRCFILES *.cpp)
file(GLOB HFILES *.h)

add_library(${PROJECT_NAME} SHARED ${SRCFILES} ${HFILES})
target_link_libraries(${PROJECT_NAME} ${GMP_LIBRARIES})
include_directories(
    ${GMP_INCLUDE_DIR}
)
## End of the workaround