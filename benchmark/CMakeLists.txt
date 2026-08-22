find_package(benchmark CONFIG REQUIRED)

add_executable(
    ${PROJECT_NAME}.Benchmark
    UnrealVoxelSim/Movement/Voxel/ControllerBenchmark.cpp
)

target_link_libraries(
    ${PROJECT_NAME}.Benchmark
    PRIVATE
        ${PROJECT_NAME}
        UnrealVoxelSim.Ecs.EnTT
        benchmark::benchmark_main
)

UnrealVoxelSimConfigureTarget(${PROJECT_NAME}.Benchmark)
