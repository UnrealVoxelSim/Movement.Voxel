#pragma once

#include "UnrealVoxelSim/Movement/Api/GroundedProfile.h"
#include "UnrealVoxelSim/Movement/Api/ICommands.h"
#include "UnrealVoxelSim/Movement/Api/IIntentSink.h"
#include "UnrealVoxelSim/Movement/Api/IReader.h"
#include "UnrealVoxelSim/Movement/Api/IUpdater.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/IReader.h"

#include <memory>
#include <span>

namespace UnrealVoxelSim::Movement::Voxel
{

class Controller final : public Api::ICommands, public Api::IIntentSink, public Api::IReader, public Api::IUpdater
{
  public:
    Controller(const UnrealVoxelSim::Voxel::Solid::Api::IReader &solids,
               std::span<const Api::GroundedProfile> profiles);
    ~Controller() override;

    Controller(const Controller &) = delete;
    Controller &operator=(const Controller &) = delete;
    Controller(Controller &&) = delete;
    Controller &operator=(Controller &&) = delete;

    [[nodiscard]] std::expected<void, Api::CommandError> Add(Api::Register registration) override;
    [[nodiscard]] std::expected<void, Api::CommandError> Remove(Ecs::Api::EntityId entity) override;
    [[nodiscard]] std::expected<void, Api::IntentError> Submit(std::span<const Api::Intent> intents) override;
    [[nodiscard]] std::expected<Api::State, Api::ReadError> Read(Ecs::Api::EntityId entity) const noexcept override;
    void Update(Simulation::Api::StepContext context) override;

  private:
    class Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
