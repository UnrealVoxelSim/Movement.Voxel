#include "UnrealVoxelSim/Movement/Voxel/Controller.h"

#include "UnrealVoxelSim/Ecs/Api/RegistryScopeId.h"
#include "UnrealVoxelSim/Ecs/EnTT/Registry.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/Cell.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/MaterialId.h"

#include <benchmark/benchmark.h>

#include <array>
#include <cstdint>
#include <vector>

namespace UnrealVoxelSim::Movement::Voxel
{
namespace
{

class FlatTerrain final : public UnrealVoxelSim::Voxel::Solid::Api::IReader
{
  public:
    [[nodiscard]] std::expected<UnrealVoxelSim::Voxel::Solid::Api::Cell, UnrealVoxelSim::Voxel::Api::ReadError> Read(
        const UnrealVoxelSim::Voxel::Api::Position position) const noexcept override
    {
        return position.Z <= 0
                   ? UnrealVoxelSim::Voxel::Solid::Api::Cell{UnrealVoxelSim::Voxel::Solid::Api::MaterialId{1}}
                   : UnrealVoxelSim::Voxel::Solid::Api::Cell{};
    }
};

void MovementTick(benchmark::State &state)
{
    const auto entityCount = static_cast<std::size_t>(state.range(0));
    FlatTerrain terrain;
    const std::array profiles{Api::GroundedProfile{Api::ProfileId{1}}};
    Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
    Controller controller{terrain, profiles};
    std::vector<Api::Intent> intents;
    intents.reserve(entityCount);
    for (std::size_t index = 0; index < entityCount; ++index)
    {
        const auto entity = registry.Create();
        const Api::Position spawn{Api::Scalar::FromWhole(static_cast<std::int32_t>(index % 32)),
                                  Api::Scalar::FromWhole(static_cast<std::int32_t>(index / 32)),
                                  Api::Scalar::FromWhole(1)};
        if (!controller.Add({entity, profiles[0].Id, spawn})) state.SkipWithError("Agent registration failed");
        intents.push_back({entity, {Api::Scalar::FromWhole(4), {}, {}}, false});
    }

    std::uint64_t tick{};
    for (auto _ : state)
    {
        static_cast<void>(_);
        const auto submitted = controller.Submit(intents);
        if (!submitted) state.SkipWithError("Intent batch submission failed");
        controller.Update({Simulation::Api::TickIndex{tick++}, Simulation::Api::StandardStepDuration});
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(entityCount));
}

BENCHMARK(MovementTick)->Arg(100)->Arg(500)->Arg(1000);

}
}
