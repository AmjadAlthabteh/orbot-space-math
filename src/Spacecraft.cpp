#include "OrbitCore/Spacecraft.hpp"

#include "OrbitCore/Physics.hpp"

#include <stdexcept>
#include <utility>

namespace orbitcore {

Spacecraft::Spacecraft(
    std::string name,
    double dryMassKg,
    double fuelKg,
    Vector3 positionMeters,
    Vector3 velocityMetersPerSecond)
    : name_(std::move(name)),
      dryMassKg_(dryMassKg),
      fuelKg_(fuelKg),
      positionMeters_(positionMeters),
      velocityMetersPerSecond_(velocityMetersPerSecond)
{
    if (name_.empty()) {
        throw std::invalid_argument("Spacecraft name cannot be empty.");
    }
    physics::validatePositive(dryMassKg_, "dryMassKg");
    physics::validateNonNegative(fuelKg_, "fuelKg");
    if (!positionMeters_.isFinite() || !velocityMetersPerSecond_.isFinite()) {
        throw std::invalid_argument("Spacecraft vectors must be finite.");
    }
}

const std::string& Spacecraft::name() const noexcept
{
    return name_;
}

double Spacecraft::dryMassKg() const noexcept
{
    return dryMassKg_;
}

double Spacecraft::fuelKg() const noexcept
{
    return fuelKg_;
}

double Spacecraft::totalMassKg() const noexcept
{
    return dryMassKg_ + fuelKg_;
}

const Vector3& Spacecraft::position() const noexcept
{
    return positionMeters_;
}

const Vector3& Spacecraft::velocity() const noexcept
{
    return velocityMetersPerSecond_;
}

void Spacecraft::setPosition(const Vector3& positionMeters)
{
    if (!positionMeters.isFinite()) {
        throw std::invalid_argument("positionMeters must be finite.");
    }
    positionMeters_ = positionMeters;
}

void Spacecraft::setVelocity(const Vector3& velocityMetersPerSecond)
{
    if (!velocityMetersPerSecond.isFinite()) {
        throw std::invalid_argument("velocityMetersPerSecond must be finite.");
    }
    velocityMetersPerSecond_ = velocityMetersPerSecond;
}

void Spacecraft::applyAcceleration(
    const Vector3& accelerationMetersPerSecondSquared,
    double deltaTimeSeconds)
{
    physics::validateNonNegative(deltaTimeSeconds, "deltaTimeSeconds");
    if (!accelerationMetersPerSecondSquared.isFinite()) {
        throw std::invalid_argument("accelerationMetersPerSecondSquared must be finite.");
    }
    velocityMetersPerSecond_ += accelerationMetersPerSecondSquared * deltaTimeSeconds;
}

void Spacecraft::advance(double deltaTimeSeconds)
{
    physics::validateNonNegative(deltaTimeSeconds, "deltaTimeSeconds");
    positionMeters_ += velocityMetersPerSecond_ * deltaTimeSeconds;
}

void Spacecraft::consumeFuel(double amountKg)
{
    physics::validateNonNegative(amountKg, "amountKg");
    if (amountKg > fuelKg_) {
        throw std::invalid_argument("Maneuver requires more fuel than the spacecraft has.");
    }
    fuelKg_ -= amountKg;
}

} // namespace orbitcore
