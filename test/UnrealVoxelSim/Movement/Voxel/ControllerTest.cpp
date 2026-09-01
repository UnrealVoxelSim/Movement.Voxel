#include "UnrealVoxelSim/Movement/Voxel/Controller.h"

#include "UnrealVoxelSim/Ecs/Api/RegistryScopeId.h"
#include "UnrealVoxelSim/Ecs/EnTT/Registry.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/Cell.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/MaterialId.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>

namespace UnrealVoxelSim::Movement::Voxel
{
	namespace
	{
		using Scalar = Math::Api::FixedPointScalar;
		using Position = Spatial::Api::Position;
		using Velocity = Spatial::Api::LinearVelocity;

		class Terrain final : public UnrealVoxelSim::Voxel::Solid::Api::IReader
		{
		public:
			[[nodiscard]] std::expected<UnrealVoxelSim::Voxel::Solid::Api::Cell, UnrealVoxelSim::Voxel::Api::ReadError>
			Read(const UnrealVoxelSim::Voxel::Api::Position position) const noexcept override
			{
				const bool ground = position.Z <= 0;
				const bool wall = Wall && position.X == 2 && position.Y == 0 && position.Z >= 1 && position.Z <= 2;
				const bool cornerLedge = CornerLedge && position.Z == 1 &&
					((position.X == 0 && position.Y == 0) || (position.X == 1 && position.Y == 0) ||
					 (position.X == 0 && position.Y == 1));
				return ground || wall || cornerLedge
					? UnrealVoxelSim::Voxel::Solid::Api::Cell{UnrealVoxelSim::Voxel::Solid::Api::MaterialId{1}}
					: UnrealVoxelSim::Voxel::Solid::Api::Cell{};
			}

			bool Wall{};
			bool CornerLedge{};
		};

		[[nodiscard]] constexpr Position Spawn(const int z = 1)
		{
			return {Scalar::FromRaw(Scalar::OneRaw / 2), Scalar::FromRaw(Scalar::OneRaw / 2), Scalar::FromWhole(z)};
		}

		class ControllerTest : public ::testing::Test
		{
		protected:
			ControllerTest() :
				Registry(Ecs::Api::RegistryScopeId{1}), Entity(Registry.Create()), MovementAccess(Registry),
				Movement(MovementAccess, TerrainState, Profiles)
			{
				Compose(Entity, Spawn());
			}

			void Compose(const Ecs::Api::EntityId entity, const Position position)
			{
				if (!Registry.Assign<Spatial::Api::PositionComponent>(entity, position) ||
					!Registry.Assign<Spatial::Api::LinearVelocityComponent>(entity, Velocity{}) ||
					!Registry.Assign<Api::ProfileComponent>(entity, Profiles[0].Id) ||
					!Registry.Assign<Api::GroundedComponent>(entity, true))
					throw std::runtime_error{"Test movement components could not be composed."};
				if (!Movement.SetIntent(entity, {}, {}))
					throw std::runtime_error{"Test movement intent could not be initialized."};
			}

			void SetInput(const Velocity velocity = {}, const bool jump = false)
			{
				ASSERT_TRUE(Movement.SetIntent(
					Entity, Simulation::Api::TickIndex{TickIndex}, Api::Intent{velocity, jump}));
			}

			void Tick()
			{
				Movement.Step({Simulation::Api::TickIndex{TickIndex++}, Simulation::Api::StandardStepDuration});
			}

			[[nodiscard]] const Spatial::Api::PositionComponent& PositionState() const
			{
				return Registry.Get<Spatial::Api::PositionComponent>(Entity)->get();
			}

			[[nodiscard]] const Spatial::Api::LinearVelocityComponent& VelocityState() const
			{
				return Registry.Get<Spatial::Api::LinearVelocityComponent>(Entity)->get();
			}

			[[nodiscard]] const Api::GroundedComponent& GroundedState() const
			{
				return Registry.Get<Api::GroundedComponent>(Entity)->get();
			}

			Terrain TerrainState;
			const std::array<Api::GroundedProfile, 1> Profiles{Api::GroundedProfile{Api::ProfileId{1}}};
			Ecs::EnTT::Registry Registry;
			Ecs::Api::EntityId Entity;
			Controller::Access MovementAccess;
			Controller Movement;
			std::uint64_t TickIndex{};
		};

		TEST_F(ControllerTest, UpdatesComposedStateWithoutPrivateAgentStorage)
		{
			EXPECT_TRUE(GroundedState().Value);
			EXPECT_EQ(PositionState().Value, Spawn());
			EXPECT_TRUE(Registry.Contains<Api::InputComponent>(Entity));
		}

		TEST_F(ControllerTest, RejectsIntentForEntityWithoutMovementComposition)
		{
			const auto entity = Registry.Create();
			const auto result = Movement.SetIntent(entity, {}, {});
			ASSERT_FALSE(result);
			EXPECT_EQ(result.error(), Api::IntentError::EntityNotMovable);
			EXPECT_FALSE(Registry.Contains<Api::InputComponent>(entity));
		}

		TEST_F(ControllerTest, RejectsIntentOutsideConfiguredMovementProfile)
		{
			const Velocity excessive{Scalar::FromRaw(Profiles[0].MaximumSpeed.Raw() + 1), {}, {}};
			const auto result = Movement.SetIntent(Entity, {}, Api::Intent{excessive, false});
			ASSERT_FALSE(result);
			EXPECT_EQ(result.error(), Api::IntentError::InvalidIntent);
		}

		TEST_F(ControllerTest, AppliesCurrentControllerInputForOneFixedTick)
		{
			SetInput({Scalar::FromWhole(4), {}, {}});
			Tick();
			EXPECT_GT(PositionState().Value.X, Spawn().X);
			EXPECT_LT(PositionState().Value.X, Scalar::FromWhole(1));
		}

		TEST_F(ControllerTest, IgnoresStaleControllerInput)
		{
			ASSERT_TRUE(Movement.SetIntent(
				Entity, Simulation::Api::TickIndex{7}, Api::Intent{{Scalar::FromWhole(4), {}, {}}, false}));
			Tick();
			EXPECT_EQ(PositionState().Value.X, Spawn().X);
		}

		TEST_F(ControllerTest, StopsAtSolidWall)
		{
			TerrainState.Wall = true;
			for (int tick = 0; tick < 50; ++tick)
			{
				SetInput({Scalar::FromWhole(4), {}, {}});
				Tick();
			}
			EXPECT_LE(PositionState().Value.X.Raw(),
					  Scalar::OneRaw + Scalar::OneRaw / 2 + Profiles[0].CollisionSkin.Raw());
			EXPECT_TRUE(GroundedState().Value);
		}

		TEST_F(ControllerTest, JumpUsesProfileVelocityAndGravity)
		{
			SetInput({}, true);
			Tick();
			EXPECT_FALSE(GroundedState().Value);
			EXPECT_GT(PositionState().Value.Z, Spawn().Z);
			EXPECT_GT(VelocityState().Value.Z, Scalar{});
		}

		TEST(ControllerRegressionTest, DropsIntoDiagonalOneCellOpeningWithoutEdgeVibration)
		{
			Terrain terrain;
			terrain.CornerLedge = true;
			const std::array profiles{Api::GroundedProfile{Api::ProfileId{1}}};
			Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
			const auto entity = registry.Create();
			ASSERT_TRUE(registry.Assign<Spatial::Api::PositionComponent>(entity, Spawn(2)));
			ASSERT_TRUE(registry.Assign<Spatial::Api::LinearVelocityComponent>(entity, Velocity{}));
			ASSERT_TRUE(registry.Assign<Api::ProfileComponent>(entity, profiles[0].Id));
			ASSERT_TRUE(registry.Assign<Api::GroundedComponent>(entity, true));
			Controller::Access access{registry};
			Controller controller{access, terrain, profiles};
			ASSERT_TRUE(controller.SetIntent(entity, {}, {}));

			constexpr auto target = Scalar::OneRaw + Scalar::OneRaw / 2;
			const auto diagonal = Scalar::FromRaw(profiles[0].MaximumSpeed.Raw() * 1000 / 1414);
			for (std::uint64_t tick = 0; tick < 100; ++tick)
			{
				const auto& position = registry.Get<Spatial::Api::PositionComponent>(entity)->get().Value;
				const auto direction = [](const std::int64_t difference) -> std::int64_t
				{
					return difference < 0 ? -1 : difference > 0 ? 1 : 0;
				};
				const Velocity desired{Scalar::FromRaw(diagonal.Raw() * direction(target - position.X.Raw())),
									   Scalar::FromRaw(diagonal.Raw() * direction(target - position.Y.Raw())),
									   {}};
				ASSERT_TRUE(controller.SetIntent(
					entity, Simulation::Api::TickIndex{tick}, Api::Intent{desired, false}));
				controller.Step({Simulation::Api::TickIndex{tick}, Simulation::Api::StandardStepDuration});
			}
			EXPECT_LT(registry.Get<Spatial::Api::PositionComponent>(entity)->get().Value.Z,
					  Scalar::FromRaw(Scalar::OneRaw + Scalar::OneRaw / 4));
			EXPECT_TRUE(registry.Get<Api::GroundedComponent>(entity)->get().Value);
		}
	} // namespace
} // namespace UnrealVoxelSim::Movement::Voxel
