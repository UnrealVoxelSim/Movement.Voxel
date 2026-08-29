#include "UnrealVoxelSim/Movement/Voxel/Controller.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <stdexcept>

namespace UnrealVoxelSim::Movement::Voxel
{
	Controller::Controller(Access access,
	                       const UnrealVoxelSim::Voxel::Solid::Api::IReader& solids,
	                       const std::span<const Api::GroundedProfile> profiles) :
		m_Access(access),
		m_Solids(solids),
		m_Profiles(profiles.begin(), profiles.end())
	{
		if (m_Profiles.empty() ||
			std::ranges::any_of(m_Profiles, [](const auto& profile) { return !profile.IsValid(); }))
			throw std::invalid_argument{"At least one valid grounded movement profile is required."};
		std::ranges::sort(m_Profiles, {}, &Api::GroundedProfile::Id);
		if (std::ranges::adjacent_find(m_Profiles, {}, &Api::GroundedProfile::Id) != m_Profiles.end())
			throw std::invalid_argument{"Movement profile identifiers must be unique."};
	}

	Controller::Raw Controller::FloorCell(const Raw value) noexcept
	{
		auto quotient = value / One;
		if (value % One < 0)
			--quotient;
		return quotient;
	}

	Controller::Raw Controller::ScaleByDuration(const Raw value, const std::int64_t nanoseconds) noexcept
	{
		return value / NanosecondsPerSecond * nanoseconds +
			value % NanosecondsPerSecond * nanoseconds / NanosecondsPerSecond;
	}

	bool Controller::IsWithinSpeed(const Spatial::Api::LinearVelocity value,
	                               const Api::GroundedProfile& profile) noexcept
	{
		const auto maximum = profile.MaximumSpeed.Raw();
		return value.Z.Raw() == 0 && value.X.Raw() >= -maximum && value.X.Raw() <= maximum &&
			value.Y.Raw() >= -maximum && value.Y.Raw() <= maximum;
	}

	const Api::GroundedProfile* Controller::FindProfile(const Api::ProfileId id) const noexcept
	{
		const auto iterator = std::ranges::lower_bound(m_Profiles, id, {}, &Api::GroundedProfile::Id);
		return iterator != m_Profiles.end() && iterator->Id == id ? &*iterator : nullptr;
	}

	bool Controller::IsBlocked(const Raw x, const Raw y, const Raw z) const noexcept
	{
		if (x < std::numeric_limits<std::int32_t>::min() || x > std::numeric_limits<std::int32_t>::max() ||
			y < std::numeric_limits<std::int32_t>::min() || y > std::numeric_limits<std::int32_t>::max() ||
			z < std::numeric_limits<std::int32_t>::min() || z > std::numeric_limits<std::int32_t>::max())
			return true;
		const auto cell =
			m_Solids.Read({static_cast<std::int32_t>(x), static_cast<std::int32_t>(y), static_cast<std::int32_t>(z)});
		return !cell || !cell->IsEmpty();
	}

	bool Controller::Collides(const Spatial::Api::Position position, const Api::GroundedProfile& profile) const noexcept
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
			for (auto y = minimumY; y <= maximumY; ++y)
				for (auto x = minimumX; x <= maximumX; ++x)
					if (IsBlocked(x, y, z))
						return true;
		return false;
	}

	Controller::Raw Controller::Sweep(Spatial::Api::Position& position,
	                                  const Api::GroundedProfile& profile,
	                                  const int axis,
	                                  const Raw delta) const noexcept
	{
		if (delta == 0)
			return 0;
		auto candidate = position;
		auto& candidateAxis = axis == 0 ? candidate.X : axis == 1 ? candidate.Y : candidate.Z;
		candidateAxis = Math::Api::FixedPointScalar::FromRaw(candidateAxis.Raw() + delta);
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
			auto& middleAxis = axis == 0 ? candidate.X : axis == 1 ? candidate.Y : candidate.Z;
			middleAxis = Math::Api::FixedPointScalar::FromRaw(middleAxis.Raw() + sign * middle);
			if (Collides(candidate, profile))
				blocked = middle;
			else
				allowed = middle;
		}
		auto& positionAxis = axis == 0 ? position.X : axis == 1 ? position.Y : position.Z;
		positionAxis = Math::Api::FixedPointScalar::FromRaw(positionAxis.Raw() + sign * allowed);
		return sign * allowed;
	}

	bool Controller::HasSupport(const Spatial::Api::Position position,
	                            const Api::GroundedProfile& profile) const noexcept
	{
		auto probe = position;
		probe.Z = Math::Api::FixedPointScalar::FromRaw(probe.Z.Raw() - 1);
		return Collides(probe, profile);
	}

	void Controller::UpdateEntity(Spatial::Api::Position& position,
	                              Spatial::Api::LinearVelocity& velocity,
	                              const Api::ProfileComponent& profileComponent,
	                              Api::GroundedComponent& grounded,
	                              const Api::InputComponent& input,
	                              const Simulation::Api::StepContext context) const
	{
		const auto* profile = FindProfile(profileComponent.Profile);
		assert(profile != nullptr);
		const auto desired = input.Tick == context.Tick && IsWithinSpeed(input.DesiredVelocity, *profile)
			                     ? input.DesiredVelocity
			                     : Spatial::Api::LinearVelocity{};
		velocity.X = desired.X;
		velocity.Y = desired.Y;
		if (input.Tick == context.Tick && input.JumpRequested && grounded.Value)
		{
			velocity.Z = profile->JumpSpeed;
			grounded.Value = false;
		}
		const auto nanoseconds = context.Duration.Value().count();
		if (!grounded.Value)
			velocity.Z += Math::Api::FixedPointScalar::FromRaw(ScaleByDuration(profile->Gravity.Raw(), nanoseconds));
		const auto vertical = ScaleByDuration(velocity.Z.Raw(), nanoseconds);
		if (vertical != 0)
		{
			const auto moved = Sweep(position, *profile, 2, vertical);
			if (moved != vertical)
			{
				grounded.Value = vertical < 0;
				velocity.Z = {};
			}
		}
		static_cast<void>(Sweep(position, *profile, 0, ScaleByDuration(velocity.X.Raw(), nanoseconds)));
		static_cast<void>(Sweep(position, *profile, 1, ScaleByDuration(velocity.Y.Raw(), nanoseconds)));
		if (grounded.Value && !HasSupport(position, *profile))
			grounded.Value = false;
	}

	void Controller::Update(const Simulation::Api::StepContext context)
	{
		assert(std::this_thread::get_id() == m_OwnerThread);
		m_Access.ForEach(Query{},
		                 [this, context](Ecs::Api::EntityId,
		                                 const Api::InputComponent& input,
		                                 const Api::ProfileComponent& profile,
		                                 Spatial::Api::PositionComponent& position,
		                                 Spatial::Api::LinearVelocityComponent& velocity,
		                                 Api::GroundedComponent& grounded)
		                 {
			                 UpdateEntity(position.Value, velocity.Value, profile, grounded, input, context);
		                 });
	}
}
