#####################################
# Define Target Information
#####################################
set(TARGET_NAME planeseg-benchmark)
set(TARGET_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

#####################################
# Define Target
#####################################
add_executable(${TARGET_NAME} ${TARGET_SOURCES})

#####################################
# Target dependencies
#####################################
add_dependencies(${TARGET_NAME} peaclib)
add_dependencies(${TARGET_NAME} benchmark::benchmark)

#####################################
# Install data for benchmark
#####################################
file(COPY configs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#####################################
# Define Target Properties
#####################################
target_link_libraries(${TARGET_NAME} PRIVATE peaclib)
target_link_libraries(${TARGET_NAME} PRIVATE benchmark::benchmark)