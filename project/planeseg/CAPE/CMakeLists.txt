#####################################
# Define Target Information
#####################################
set(TARGET_NAME capelib)
set(TARGET_INCLUDE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(TARGET_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cape.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Histogram.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Params.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/PlaneSeg.cpp
)

#####################################
# Required packages
#####################################
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

#####################################
# Define Target
#####################################
add_library(${TARGET_NAME} SHARED ${TARGET_SOURCES})

#####################################
# Define Target Properties
#####################################
target_include_directories(
        ${TARGET_NAME}
        SYSTEM
        PUBLIC
        $<BUILD_INTERFACE:${TARGET_INCLUDE_FOLDER}>
)

target_link_libraries(${TARGET_NAME} PUBLIC ${OpenCV_LIBS})
target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)
