find_package(HAL REQUIRED)
find_package(GFlags REQUIRED)

include_directories(
    ../include/
    ${HAL_INCLUDE_DIRS}
    )

set(SRCS
    poseTest.cpp
    PoseTracker.cpp
    )

set(HEADERS
    PoseTracker.h)

add_executable(tracker ${SRCS} ${HEADERS})

target_link_libraries(tracker
    ../libs/
    ${HAL_LIBRARIES}
    )
