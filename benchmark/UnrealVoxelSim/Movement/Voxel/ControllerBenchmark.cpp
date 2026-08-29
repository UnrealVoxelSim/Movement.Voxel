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
			[[nodiscard]] std::expected<UnrealVoxelSim::Voxel::Solid::Api::Cell, UnrealVoxelSim::Voxel::Api::ReadError>
			Read(const UnrealVoxelSim::Voxel::Api::Position position) const noexcept override
			{
				return position.Z <= 0
					? UnrealVoxelSim::Voxel::Solid::Api::Cell{UnrealVoxelSim::Voxel::Solid::Api::MaterialId{1}}
					: UnrealVoxelSim::Voxel::Solid::Api::Cell{};
			}
		};

		void MovementTick(benchmark::State& state)
		{
			using Scalar = Math::Api::FixedPointScalar;
			using Position = Spatial::Api::Position;
			using Velocity = Spatial::Api::LinearVelocity;

			const auto entityCount = static_cast<std::size_t>(state.range(0));
			FlatTerrain terrain;
			const std::array profiles{Api::GroundedProfile{Api::ProfileId{1}}};
			Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
			Controller::Access access{registry};
			Controller controller{access, terrain, profiles};
			std::vector<Ecs::Api::EntityId> entities;
			entities.reserve(entityCount);
			for (std::size_t index = 0; index < entityCount; ++index)
			{
				const auto entity = registry.Create();
				const Position spawn{Scalar::FromWhole(static_cast<std::int32_t>(index % 32)),
									 Scalar::FromWhole(static_cast<std::int32_t>(index / 32)),
									 Scalar::FromWhole(1)};
				if (!registry.Assign<Spatial::Api::PositionComponent>(entity, spawn) ||
					!registry.Assign<Spatial::Api::LinearVelocityComponent>(entity, Velocity{}) ||
					!registry.Assign<Api::MovementProfileComponent>(entity, profiles[0].Id) ||
					!registry.Assign<Api::GroundedComponent>(entity, true) ||
					!registry.Assign<Api::MovementInputComponent>(entity, Api::MovementInputComponent{}))
				{
					state.SkipWithError("Movement component composition failed");
					return;
				}
				entities.push_back(entity);
			}

			std::uint64_t tick{};
			for (auto _ : state)
			{
				static_cast<void>(_);
				for (const auto entity : entities)
				{
					if (!registry.Assign<Api::MovementInputComponent>(
							entity,
							Api::MovementInputComponent{
								Simulation::Api::TickIndex{tick}, {Scalar::FromWhole(4), {}, {}}, false}))
					{
						state.SkipWithError("Movement input update failed");
						return;
					}
				}
				controller.Update({Simulation::Api::TickIndex{tick}, Simulation::Api::StandardStepDuration});
				++tick;
				benchmark::ClobberMemory();
			}
			state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(entityCount));
		}

		BENCHMARK(MovementTick)->Arg(100)->Arg(500)->Arg(1000);
	} // namespace
} // namespace UnrealVoxelSim::Movement::Voxel
