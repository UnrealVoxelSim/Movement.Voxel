#pragma once

#include "UnrealVoxelSim/Ecs/Api/Access.h"
#include "UnrealVoxelSim/Movement/Api/GroundedComponent.h"
#include "UnrealVoxelSim/Movement/Api/GroundedProfile.h"
#include "UnrealVoxelSim/Movement/Api/IUpdater.h"
#include "UnrealVoxelSim/Movement/Api/InputComponent.h"
#include "UnrealVoxelSim/Movement/Api/ProfileComponent.h"
#include "UnrealVoxelSim/Spatial/Api/LinearVelocityComponent.h"
#include "UnrealVoxelSim/Spatial/Api/PositionComponent.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/IReader.h"

#include <cstdint>
#include <span>
#include <thread>
#include <vector>

namespace UnrealVoxelSim::Movement::Voxel
{
	class Controller final : public Api::IUpdater
	{
		using Query = Ecs::Api::Query<Ecs::Api::Read<Api::InputComponent, Api::ProfileComponent>,
									  Ecs::Api::Write<Spatial::Api::PositionComponent,
													  Spatial::Api::LinearVelocityComponent,
													  Api::GroundedComponent>>;
		using Permissions =
			Ecs::Api::Permissions<Ecs::Api::Read<Api::InputComponent, Api::ProfileComponent>,
								  Ecs::Api::Write<Spatial::Api::PositionComponent,
												  Spatial::Api::LinearVelocityComponent,
												  Api::GroundedComponent>>;
		using Raw = std::int64_t;

	public:
		using Access = Ecs::Api::Access<Permissions, Query>;

		Controller(Access access,
				   const UnrealVoxelSim::Voxel::Solid::Api::IReader& solids,
				   std::span<const Api::GroundedProfile> profiles);
		Controller(const Controller&) = delete;
		Controller& operator=(const Controller&) = delete;

		void Update(Simulation::Api::StepContext context) override;

	private:
		static constexpr Raw One = Math::Api::FixedPointScalar::OneRaw;
		static constexpr std::int64_t NanosecondsPerSecond = 1'000'000'000;

		[[nodiscard]] static Raw FloorCell(Raw value) noexcept;
		[[nodiscard]] static Raw ScaleByDuration(Raw value, std::int64_t nanoseconds) noexcept;
		[[nodiscard]] static bool IsWithinSpeed(Spatial::Api::LinearVelocity value,
												const Api::GroundedProfile& profile) noexcept;
		[[nodiscard]] const Api::GroundedProfile* FindProfile(Api::ProfileId id) const noexcept;
		[[nodiscard]] bool IsBlocked(Raw x, Raw y, Raw z) const noexcept;
		[[nodiscard]] bool Collides(Spatial::Api::Position position,
									const Api::GroundedProfile& profile) const noexcept;
		[[nodiscard]] Raw Sweep(Spatial::Api::Position& position,
								const Api::GroundedProfile& profile,
								int axis,
								Raw delta) const noexcept;
		[[nodiscard]] bool HasSupport(Spatial::Api::Position position,
									  const Api::GroundedProfile& profile) const noexcept;
		void UpdateEntity(Spatial::Api::Position& position,
						  Spatial::Api::LinearVelocity& velocity,
						  const Api::ProfileComponent& profile,
						  Api::GroundedComponent& grounded,
						  const Api::InputComponent& input,
						  Simulation::Api::StepContext context) const;

		Access m_Access;
		const UnrealVoxelSim::Voxel::Solid::Api::IReader& m_Solids;
		std::vector<Api::GroundedProfile> m_Profiles;
		std::thread::id m_OwnerThread{std::this_thread::get_id()};
	};
} // namespace UnrealVoxelSim::Movement::Voxel
