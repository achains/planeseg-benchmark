find_package(benchmark QUIET)

if (NOT benchmark_FOUND)
    include(FetchContent)
    FetchContent_Declare(
            benchmark
            GIT_REPOSITORY https://github.com/google/benchmark.git
            GIT_TAG        v1.7.0
    )
    # Is needed for google-benchmark to build properly
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest
            GIT_TAG        v1.12.0
    )
    FetchContent_GetProperties(benchmark)
    if (NOT benchmark_POPULATED)
        FetchContent_Populate(benchmark)
        FetchContent_MakeAvailable(googletest benchmark)
        add_subdirectory(${benchmark_SOURCE_DIR} ${benchmark_BINARY_DIR})
    endif()
endif()
