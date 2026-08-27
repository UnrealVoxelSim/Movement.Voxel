#include "UnrealVoxelSim/Movement/Voxel/Controller.h"

#include "UnrealVoxelSim/Voxel/Api/Position.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace UnrealVoxelSim::Movement::Voxel
{
namespace
{

using Raw = std::int64_t;
constexpr Raw One = Api::Scalar::OneRaw;
constexpr std::int64_t NanosecondsPerSecond = 1'000'000'000;

struct Agent final
{
    Ecs::Api::EntityId Entity;
    Api::State State;
};

[[nodiscard]] Raw FloorCell(const Raw value) noexcept
{
    auto quotient = value / One;
    if (value % One < 0)
    {
        --quotient;
    }
    return quotient;
}

[[nodiscard]] Raw ScaleByDuration(const Raw value, const std::int64_t nanoseconds) noexcept
{
    const auto quotient = value / NanosecondsPerSecond;
    const auto remainder = value % NanosecondsPerSecond;
    return quotient * nanoseconds + remainder * nanoseconds / NanosecondsPerSecond;
}

[[nodiscard]] bool IsWithinSpeed(const Api::Vector velocity, const Api::GroundedProfile &profile) noexcept
{
    const auto maximum = profile.MaximumSpeed.Raw();
    return velocity.Z.Raw() == 0 && velocity.X.Raw() >= -maximum && velocity.X.Raw() <= maximum &&
           velocity.Y.Raw() >= -maximum && velocity.Y.Raw() <= maximum;
}

} // namespace

class Controller::Impl final
{
  public:
    Impl(const UnrealVoxelSim::Voxel::Solid::Api::IReader &solids, std::span<const Api::GroundedProfile> profiles)
        : Solids(solids), Profiles(profiles.begin(), profiles.end())
    {
        if (Profiles.empty() || std::ranges::any_of(Profiles, [](const auto &profile) { return !profile.IsValid(); }))
        {
            throw std::invalid_argument{"At least one valid grounded movement profile is required."};
        }
        std::ranges::sort(Profiles, {}, &Api::GroundedProfile::Id);
        if (std::ranges::adjacent_find(Profiles, {}, &Api::GroundedProfile::Id) != Profiles.end())
        {
            throw std::invalid_argument{"Movement profile identifiers must be unique."};
        }
    }

    void AssertOwnerThread() const noexcept
    {
        assert(std::this_thread::get_id() == OwnerThread);
    }

    [[nodiscard]] const Api::GroundedProfile *FindProfile(const Api::ProfileId id) const noexcept
    {
        const auto iterator = std::ranges::lower_bound(Profiles, id, {}, &Api::GroundedProfile::Id);
        return iterator != Profiles.end() && iterator->Id == id ? &*iterator : nullptr;
    }

    [[nodiscard]] auto FindAgent(const Ecs::Api::EntityId entity) noexcept
    {
        return std::ranges::lower_bound(Agents, entity, {}, &Agent::Entity);
    }

    [[nodiscard]] auto FindAgent(const Ecs::Api::EntityId entity) const noexcept
    {
        return std::ranges::lower_bound(Agents, entity, {}, &Agent::Entity);
    }

    [[nodiscard]] bool IsBlocked(const Raw x, const Raw y, const Raw z) const noexcept
    {
        if (x < std::numeric_limits<std::int32_t>::min() || x > std::numeric_limits<std::int32_t>::max() ||
            y < std::numeric_limits<std::int32_t>::min() || y > std::numeric_limits<std::int32_t>::max() ||
            z < std::numeric_limits<std::int32_t>::min() || z > std::numeric_limits<std::int32_t>::max())
        {
            return true;
        }
        const auto cell = Solids.Read({static_cast<std::int32_t>(x), static_cast<std::int32_t>(y),
                                       static_cast<std::int32_t>(z)});
        return !cell || !cell->IsEmpty();
    }

    [[nodiscard]] bool Collides(const Api::Position position, const Api::GroundedProfile &profile) const noexcept
    {
        const auto halfWidth = static_cast<Raw>(profile.Width) * One / 2 - profile.CollisionSkin.Raw();
        const auto halfLength = static_cast<Raw>(profile.Length) * One / 2 - profile.CollisionSkin.Raw();
        const auto minimumX = FloorCell(position.X.Raw() - halfWidth);
        const auto maximumX = FloorCell(position.X.Raw() + halfWidth - 1);
        const auto minimumY = FloorCell(position.Y.Raw() - halfLength);
        const auto maximumY = FloorCell(position.Y.Raw() + halfLength - 1);
        const auto minimumZ = FloorCell(position.Z.Raw());
        const auto maximumZ = FloorCell(position.Z.Raw() + static_cast<Raw>(profile.Height) * One - 1);
        for (auto z = minimumZ; z <= maximumZ; ++z)
        {
            for (auto y = minimumY; y <= maximumY; ++y)
            {
                for (auto x = minimumX; x <= maximumX; ++x)
                {
                    if (IsBlocked(x, y, z))
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    [[nodiscard]] Raw Sweep(Api::Position &position, const Api::GroundedProfile &profile, const int axis,
                            const Raw delta) const noexcept
    {
        if (delta == 0)
        {
            return 0;
        }
        auto candidate = position;
        auto &candidateAxis = axis == 0 ? candidate.X : axis == 1 ? candidate.Y : candidate.Z;
        candidateAxis = Api::Scalar::FromRaw(candidateAxis.Raw() + delta);
        if (!Collides(candidate, profile))
        {
            position = candidate;
            return delta;
        }

        const auto sign = delta > 0 ? Raw{1} : Raw{-1};
        Raw allowed{};
        Raw blocked = delta > 0 ? delta : -delta;
        while (allowed + 1 < blocked)
        {
            const auto middle = allowed + (blocked - allowed) / 2;
            candidate = position;
            auto &middleAxis = axis == 0 ? candidate.X : axis == 1 ? candidate.Y : candidate.Z;
            middleAxis = Api::Scalar::FromRaw(middleAxis.Raw() + sign * middle);
            if (Collides(candidate, profile))
            {
                blocked = middle;
            }
            else
            {
                allowed = middle;
            }
        }
        auto &positionAxis = axis == 0 ? position.X : axis == 1 ? position.Y : position.Z;
        positionAxis = Api::Scalar::FromRaw(positionAxis.Raw() + sign * allowed);
        return sign * allowed;
    }

    [[nodiscard]] bool HasSupport(const Api::Position position, const Api::GroundedProfile &profile) const noexcept
    {
        auto probe = position;
        probe.Z = Api::Scalar::FromRaw(probe.Z.Raw() - 1);
        return Collides(probe, profile);
    }

    const UnrealVoxelSim::Voxel::Solid::Api::IReader &Solids;
    std::vector<Api::GroundedProfile> Profiles;
    std::vector<Agent> Agents;
    std::vector<Api::Intent> Intents;
    std::thread::id OwnerThread{std::this_thread::get_id()};
};

Controller::Controller(const UnrealVoxelSim::Voxel::Solid::Api::IReader &solids,
                       const std::span<const Api::GroundedProfile> profiles)
    : m_Impl(std::make_unique<Impl>(solids, profiles))
{
}

Controller::~Controller() = default;

std::expected<void, Api::CommandError> Controller::Add(const Api::Register registration)
{
    m_Impl->AssertOwnerThread();
    if (!registration.Entity.IsValid()) return std::unexpected{Api::CommandError::InvalidEntity};
    const auto *profile = m_Impl->FindProfile(registration.Profile);
    if (!profile) return std::unexpected{Api::CommandError::UnknownProfile};
    const auto iterator = m_Impl->FindAgent(registration.Entity);
    if (iterator != m_Impl->Agents.end() && iterator->Entity == registration.Entity)
        return std::unexpected{Api::CommandError::AlreadyRegistered};
    if (m_Impl->Collides(registration.Location, *profile))
        return std::unexpected{Api::CommandError::LocationBlocked};
    Api::State state{registration.Location, {}, registration.Profile, m_Impl->HasSupport(registration.Location, *profile)};
    m_Impl->Agents.insert(iterator, Agent{registration.Entity, state});
    return {};
}

std::expected<void, Api::CommandError> Controller::Remove(const Ecs::Api::EntityId entity)
{
    m_Impl->AssertOwnerThread();
    const auto iterator = m_Impl->FindAgent(entity);
    if (iterator == m_Impl->Agents.end() || iterator->Entity != entity)
        return std::unexpected{Api::CommandError::NotRegistered};
    m_Impl->Agents.erase(iterator);
    return {};
}

std::expected<void, Api::IntentError> Controller::Submit(const std::span<const Api::Intent> intents)
{
    m_Impl->AssertOwnerThread();
    std::vector<Api::Intent> ordered(intents.begin(), intents.end());
    std::ranges::sort(ordered, {}, &Api::Intent::Entity);
    for (std::size_t index = 0; index < ordered.size(); ++index)
    {
        if (index != 0 && ordered[index - 1].Entity == ordered[index].Entity)
            return std::unexpected{Api::IntentError{Api::IntentErrorType::DuplicateEntity, index}};
        const auto agent = m_Impl->FindAgent(ordered[index].Entity);
        if (agent == m_Impl->Agents.end() || agent->Entity != ordered[index].Entity)
            return std::unexpected{Api::IntentError{Api::IntentErrorType::EntityNotRegistered, index}};
        const auto *profile = m_Impl->FindProfile(agent->State.Profile);
        if (!profile || !IsWithinSpeed(ordered[index].DesiredVelocity, *profile))
            return std::unexpected{Api::IntentError{Api::IntentErrorType::InvalidVelocity, index}};
    }
    m_Impl->Intents = std::move(ordered);
    return {};
}

std::expected<Api::State, Api::ReadError> Controller::Read(const Ecs::Api::EntityId entity) const noexcept
{
    m_Impl->AssertOwnerThread();
    const auto iterator = m_Impl->FindAgent(entity);
    if (iterator == m_Impl->Agents.end() || iterator->Entity != entity)
        return std::unexpected{Api::ReadError::NotRegistered};
    return iterator->State;
}

void Controller::Update(const Simulation::Api::StepContext context)
{
    m_Impl->AssertOwnerThread();
    const auto nanoseconds = context.Duration.Value().count();
    std::size_t intentIndex{};
    for (auto &agent : m_Impl->Agents)
    {
        const auto *profile = m_Impl->FindProfile(agent.State.Profile);
        assert(profile != nullptr);
        Api::Intent intent{agent.Entity};
        while (intentIndex < m_Impl->Intents.size() && m_Impl->Intents[intentIndex].Entity < agent.Entity) ++intentIndex;
        if (intentIndex < m_Impl->Intents.size() && m_Impl->Intents[intentIndex].Entity == agent.Entity)
            intent = m_Impl->Intents[intentIndex];

        agent.State.Velocity.X = intent.DesiredVelocity.X;
        agent.State.Velocity.Y = intent.DesiredVelocity.Y;
        if (intent.Jump && agent.State.Grounded)
        {
            agent.State.Velocity.Z = profile->JumpSpeed;
            agent.State.Grounded = false;
        }

        if (!agent.State.Grounded)
            agent.State.Velocity.Z += Api::Scalar::FromRaw(ScaleByDuration(profile->Gravity.Raw(), nanoseconds));

        const auto vertical = ScaleByDuration(agent.State.Velocity.Z.Raw(), nanoseconds);
        if (vertical != 0)
        {
            const auto moved = m_Impl->Sweep(agent.State.Location, *profile, 2, vertical);
            if (moved != vertical)
            {
                agent.State.Grounded = vertical < 0;
                agent.State.Velocity.Z = {};
            }
        }

        static_cast<void>(m_Impl->Sweep(agent.State.Location, *profile, 0,
                                       ScaleByDuration(agent.State.Velocity.X.Raw(), nanoseconds)));
        static_cast<void>(m_Impl->Sweep(agent.State.Location, *profile, 1,
                                       ScaleByDuration(agent.State.Velocity.Y.Raw(), nanoseconds)));
        if (agent.State.Grounded && !m_Impl->HasSupport(agent.State.Location, *profile))
            agent.State.Grounded = false;
    }
    m_Impl->Intents.clear();
}

} // namespace UnrealVoxelSim::Movement::Voxel
