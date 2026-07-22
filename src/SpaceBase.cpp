#include "OrbitCore/SpaceBase.hpp"

#include "OrbitCore/Physics.hpp"

#include <stdexcept>
#include <utility>

namespace orbitcore {

SpaceBase::SpaceBase(std::string name, Vector3 positionMeters, double dockingRadiusMeters)
    : name_(std::move(name)),
      positionMeters_(positionMeters),
      dockingRadiusMeters_(dockingRadiusMeters)
{
    if (name_.empty()) {
        throw std::invalid_argument("Space base name cannot be empty.");
    }
    if (!positionMeters_.isFinite()) {
        throw std::invalid_argument("Space base position must be finite.");
    }
    physics::validatePositive(dockingRadiusMeters_, "dockingRadiusMeters");
}

const std::string& SpaceBase::name() const noexcept
{
    return name_;
}

const Vector3& SpaceBase::position() const noexcept
{
    return positionMeters_;
}

double SpaceBase::dockingRadiusMeters() const noexcept
{
    return dockingRadiusMeters_;
}

double SpaceBase::distanceTo(const Vector3& spacecraftPosition) const
{
    return physics::distance(spacecraftPosition, positionMeters_);
}

Vector3 SpaceBase::directionFrom(const Vector3& spacecraftPosition) const
{
    return physics::directionToward(spacecraftPosition, positionMeters_);
}

bool SpaceBase::isDockingRange(const Vector3& spacecraftPosition) const
{
    return physics::hasCollisionRisk(spacecraftPosition, positionMeters_, dockingRadiusMeters_);
}

} // namespace orbitcore
