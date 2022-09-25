#####################################
# Define Target Information
#####################################
set(TARGET_NAME peaclib)
set(TARGET_INCLUDE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(TARGET_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/peac.cpp)

#####################################
# Required packages
#####################################
find_package(OpenCV REQUIRED)
find_package(PCL COMPONENTS io REQUIRED)

#####################################
# Define Target
#####################################
add_library(${TARGET_NAME} SHARED ${TARGET_SOURCES})

#####################################
# Define Target Properties
#####################################
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${TARGET_INCLUDE_FOLDER}>)

# Suppress error from pcl_isnan
target_compile_options(${TARGET_NAME} PRIVATE "-fpermissive")

target_link_libraries(${TARGET_NAME} PUBLIC ${PCL_LIBRARIES})
target_link_libraries(${TARGET_NAME} PUBLIC ${OpenCV_LIBS})
