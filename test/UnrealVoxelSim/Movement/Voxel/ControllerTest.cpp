#include "UnrealVoxelSim/Movement/Voxel/Controller.h"

#include "UnrealVoxelSim/Ecs/Api/RegistryScopeId.h"
#include "UnrealVoxelSim/Ecs/EnTT/Registry.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/Cell.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/MaterialId.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace UnrealVoxelSim::Movement::Voxel
{
namespace
{

class Terrain final : public UnrealVoxelSim::Voxel::Solid::Api::IReader
{
  public:
    [[nodiscard]] std::expected<UnrealVoxelSim::Voxel::Solid::Api::Cell, UnrealVoxelSim::Voxel::Api::ReadError> Read(
        const UnrealVoxelSim::Voxel::Api::Position position) const noexcept override
    {
        const bool ground = position.Z <= 0;
        const bool wall = Wall && position.X == 2 && position.Y == 0 && position.Z >= 1 && position.Z <= 2;
        const bool cornerLedge = CornerLedge && position.Z == 1 &&
                                 ((position.X == 0 && position.Y == 0) ||
                                  (position.X == 1 && position.Y == 0) ||
                                  (position.X == 0 && position.Y == 1));
        return ground || wall || cornerLedge
                   ? UnrealVoxelSim::Voxel::Solid::Api::Cell{UnrealVoxelSim::Voxel::Solid::Api::MaterialId{1}}
                   : UnrealVoxelSim::Voxel::Solid::Api::Cell{};
    }

    bool Wall{};
    bool CornerLedge{};
};

[[nodiscard]] constexpr Api::Position Spawn()
{
    return {Api::Scalar::FromRaw(Api::Scalar::OneRaw / 2), Api::Scalar::FromRaw(Api::Scalar::OneRaw / 2),
            Api::Scalar::FromWhole(1)};
}

class ControllerTest : public ::testing::Test
{
  protected:
    ControllerTest() : Registry(Ecs::Api::RegistryScopeId{1}), Entity(Registry.Create()), m_Controller(m_Terrain, Profiles)
    {
        const auto added = m_Controller.Add({Entity, Profiles[0].Id, Spawn()});
        if (!added) throw std::runtime_error{"Test agent registration failed."};
    }

    void Tick()
    {
        m_Controller.Update({Simulation::Api::TickIndex{TickIndex++}, Simulation::Api::StandardStepDuration});
    }

    Terrain m_Terrain;
    const std::array<Api::GroundedProfile, 1> Profiles{Api::GroundedProfile{Api::ProfileId{1}}};
    Ecs::EnTT::Registry Registry;
    Ecs::Api::EntityId Entity;
    Controller m_Controller;
    std::uint64_t TickIndex{};
};

TEST_F(ControllerTest, RegistersGroundedContinuousState)
{
    const auto state = m_Controller.Read(Entity);

    ASSERT_TRUE(state);
    EXPECT_TRUE(state->Grounded);
    EXPECT_EQ(state->Location, Spawn());
}

TEST_F(ControllerTest, AppliesContinuousIntentForOneFixedTick)
{
    ASSERT_TRUE(m_Controller.Submit(std::array{Api::Intent{Entity, {Api::Scalar::FromWhole(4), {}, {}}, false}}));

    Tick();

    const auto state = m_Controller.Read(Entity);
    ASSERT_TRUE(state);
    EXPECT_GT(state->Location.X, Spawn().X);
    EXPECT_LT(state->Location.X, Api::Scalar::FromWhole(1));
}

TEST_F(ControllerTest, StopsAtSolidWall)
{
    m_Terrain.Wall = true;
    for (int tick = 0; tick < 50; ++tick)
    {
        ASSERT_TRUE(m_Controller.Submit(std::array{Api::Intent{Entity, {Api::Scalar::FromWhole(4), {}, {}}, false}}));
        Tick();
    }

    const auto state = m_Controller.Read(Entity);
    ASSERT_TRUE(state);
    EXPECT_LE(state->Location.X.Raw(),
              Api::Scalar::FromRaw(Api::Scalar::OneRaw + Api::Scalar::OneRaw / 2 +
                                   Profiles[0].CollisionSkin.Raw()).Raw());
    EXPECT_TRUE(state->Grounded);
}

TEST_F(ControllerTest, JumpUsesProfileVelocityAndGravity)
{
    ASSERT_TRUE(m_Controller.Submit(std::array{Api::Intent{Entity, {}, true}}));

    Tick();

    const auto state = m_Controller.Read(Entity);
    ASSERT_TRUE(state);
    EXPECT_FALSE(state->Grounded);
    EXPECT_GT(state->Location.Z, Spawn().Z);
    EXPECT_GT(state->Velocity.Z, Api::Scalar{});
}

TEST_F(ControllerTest, RejectsTwoIntentsForOneEntity)
{
    const std::array intents{Api::Intent{Entity, {}, false}, Api::Intent{Entity, {}, true}};

    const auto result = m_Controller.Submit(intents);

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().Error, Api::IntentErrorType::DuplicateEntity);
}

TEST(ControllerRegressionTest, DropsIntoDiagonalOneCellOpeningWithoutEdgeVibration)
{
    Terrain terrain;
    terrain.CornerLedge = true;
    const std::array profiles{Api::GroundedProfile{Api::ProfileId{1}}};
    Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
    const auto entity = registry.Create();
    Controller controller{terrain, profiles};
    const Api::Position spawn{Api::Scalar::FromRaw(Api::Scalar::OneRaw / 2),
                              Api::Scalar::FromRaw(Api::Scalar::OneRaw / 2), Api::Scalar::FromWhole(2)};
    ASSERT_TRUE(controller.Add({entity, profiles[0].Id, spawn}));

    constexpr auto target = Api::Scalar::OneRaw + Api::Scalar::OneRaw / 2;
    const auto diagonal = Api::Scalar::FromRaw(profiles[0].MaximumSpeed.Raw() * 1000 / 1414);
    for (std::uint64_t tick = 0; tick < 100; ++tick)
    {
        const auto current = controller.Read(entity);
        ASSERT_TRUE(current);
        const auto direction = [](const std::int64_t difference) -> std::int64_t {
            return difference < 0 ? -1 : difference > 0 ? 1 : 0;
        };
        const Api::Intent intent{
            entity,
            {Api::Scalar::FromRaw(diagonal.Raw() * direction(target - current->Location.X.Raw())),
             Api::Scalar::FromRaw(diagonal.Raw() * direction(target - current->Location.Y.Raw())), {}},
            false};
        ASSERT_TRUE(controller.Submit(std::array{intent}));
        controller.Update({Simulation::Api::TickIndex{tick}, Simulation::Api::StandardStepDuration});
    }

    const auto state = controller.Read(entity);
    ASSERT_TRUE(state);
    EXPECT_LT(state->Location.Z, Api::Scalar::FromRaw(Api::Scalar::OneRaw + Api::Scalar::OneRaw / 4));
    EXPECT_TRUE(state->Grounded);
}

} // namespace
} // namespace UnrealVoxelSim::Movement::Voxel
