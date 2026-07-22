#include "OrbitCore/Physics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace orbitcore::physics {

void validatePositive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be positive and finite.");
    }
}

void validateNonNegative(double value, const char* name)
{
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be non-negative and finite.");
    }
}

double distance(const Vector3& from, const Vector3& to)
{
    return (to - from).magnitude();
}

double gravitationalForce(double massA, double massB, double separationMeters)
{
    validatePositive(massA, "massA");
    validatePositive(massB, "massB");
    validatePositive(separationMeters, "separationMeters");
    return gravitationalConstant * massA * massB / (separationMeters * separationMeters);
}

double orbitalVelocity(double centralMassKg, double orbitRadiusMeters)
{
    validatePositive(centralMassKg, "centralMassKg");
    validatePositive(orbitRadiusMeters, "orbitRadiusMeters");
    return std::sqrt(gravitationalConstant * centralMassKg / orbitRadiusMeters);
}

double escapeVelocity(double centralMassKg, double radiusMeters)
{
    validatePositive(centralMassKg, "centralMassKg");
    validatePositive(radiusMeters, "radiusMeters");
    return std::sqrt(2.0 * gravitationalConstant * centralMassKg / radiusMeters);
}

double orbitalPeriod(double centralMassKg, double orbitRadiusMeters)
{
    validatePositive(centralMassKg, "centralMassKg");
    validatePositive(orbitRadiusMeters, "orbitRadiusMeters");
    constexpr double pi = 3.14159265358979323846;
    return 2.0 * pi * std::sqrt((orbitRadiusMeters * orbitRadiusMeters * orbitRadiusMeters) / (gravitationalConstant * centralMassKg));
}

double kineticEnergy(double massKg, const Vector3& velocity)
{
    validatePositive(massKg, "massKg");
    if (!velocity.isFinite()) {
        throw std::invalid_argument("velocity must be finite.");
    }
    return 0.5 * massKg * velocity.magnitudeSquared();
}

double gravitationalPotentialEnergy(double massKg, double centralMassKg, double radiusMeters)
{
    validatePositive(massKg, "massKg");
    validatePositive(centralMassKg, "centralMassKg");
    validatePositive(radiusMeters, "radiusMeters");
    return -gravitationalConstant * massKg * centralMassKg / radiusMeters;
}

double specificOrbitalEnergy(
    double centralMassKg,
    const Vector3& position,
    const Vector3& velocity)
{
    validatePositive(centralMassKg, "centralMassKg");
    if (!position.isFinite() || !velocity.isFinite()) {
        throw std::invalid_argument("position and velocity must be finite.");
    }
    const double radius = position.magnitude();
    validatePositive(radius, "position radius");
    return 0.5 * velocity.magnitudeSquared() - gravitationalConstant * centralMassKg / radius;
}

double thrustAcceleration(double thrustNewtons, double spacecraftMassKg)
{
    validateNonNegative(thrustNewtons, "thrustNewtons");
    validatePositive(spacecraftMassKg, "spacecraftMassKg");
    return thrustNewtons / spacecraftMassKg;
}

double deltaVFromSpecificImpulse(
    double specificImpulseSeconds,
    double initialMassKg,
    double finalMassKg)
{
    validatePositive(specificImpulseSeconds, "specificImpulseSeconds");
    validatePositive(initialMassKg, "initialMassKg");
    validatePositive(finalMassKg, "finalMassKg");
    if (finalMassKg > initialMassKg) {
        throw std::invalid_argument("finalMassKg cannot exceed initialMassKg.");
    }
    return specificImpulseSeconds * standardGravity * std::log(initialMassKg / finalMassKg);
}

HohmannTransfer hohmannTransfer(
    double centralMassKg,
    double startingOrbitRadiusMeters,
    double targetOrbitRadiusMeters)
{
    validatePositive(centralMassKg, "centralMassKg");
    validatePositive(startingOrbitRadiusMeters, "startingOrbitRadiusMeters");
    validatePositive(targetOrbitRadiusMeters, "targetOrbitRadiusMeters");

    constexpr double pi = 3.14159265358979323846;
    const double mu = gravitationalConstant * centralMassKg;
    const double semiMajorAxis = 0.5 * (startingOrbitRadiusMeters + targetOrbitRadiusMeters);
    const double departureDeltaV = std::sqrt(mu / startingOrbitRadiusMeters)
        * (std::sqrt((2.0 * targetOrbitRadiusMeters) / (startingOrbitRadiusMeters + targetOrbitRadiusMeters)) - 1.0);
    const double arrivalDeltaV = std::sqrt(mu / targetOrbitRadiusMeters)
        * (1.0 - std::sqrt((2.0 * startingOrbitRadiusMeters) / (startingOrbitRadiusMeters + targetOrbitRadiusMeters)));
    const double transferTime = pi * std::sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / mu);

    return HohmannTransfer{
        std::abs(departureDeltaV),
        std::abs(arrivalDeltaV),
        std::abs(departureDeltaV) + std::abs(arrivalDeltaV),
        transferTime
    };
}

Vector3 gravitationalAcceleration(
    const Vector3& position,
    const Vector3& centralBodyPosition,
    double centralMassKg)
{
    validatePositive(centralMassKg, "centralMassKg");
    const Vector3 direction = centralBodyPosition - position;
    const double radiusSquared = direction.magnitudeSquared();
    if (radiusSquared == 0.0) {
        throw std::invalid_argument("Spacecraft cannot occupy the central body's exact position.");
    }

    const double accelerationMagnitude = gravitationalConstant * centralMassKg / radiusSquared;
    return direction.normalized() * accelerationMagnitude;
}

Vector3 estimateFuturePosition(
    const Vector3& position,
    const Vector3& velocity,
    const Vector3& acceleration,
    double deltaTimeSeconds)
{
    validateNonNegative(deltaTimeSeconds, "deltaTimeSeconds");
    return position + velocity * deltaTimeSeconds + acceleration * (0.5 * deltaTimeSeconds * deltaTimeSeconds);
}

Vector3 directionToward(const Vector3& from, const Vector3& to)
{
    return (to - from).normalized();
}

bool hasCollisionRisk(
    const Vector3& positionA,
    const Vector3& positionB,
    double safetyRadiusMeters)
{
    validatePositive(safetyRadiusMeters, "safetyRadiusMeters");
    return distance(positionA, positionB) <= safetyRadiusMeters;
}

double fuelUsageForManeuver(
    double spacecraftMassKg,
    double deltaVMetersPerSecond,
    double engineEfficiency)
{
    validatePositive(spacecraftMassKg, "spacecraftMassKg");
    validateNonNegative(deltaVMetersPerSecond, "deltaVMetersPerSecond");
    validatePositive(engineEfficiency, "engineEfficiency");
    if (engineEfficiency > 1.0) {
        throw std::invalid_argument("engineEfficiency cannot exceed 1.0.");
    }

    // Simple planning model: higher efficiency lowers propellant cost for the same delta-v.
    constexpr double maneuverScaling = 9500.0;
    return spacecraftMassKg * (deltaVMetersPerSecond / maneuverScaling) / engineEfficiency;
}

} // namespace orbitcore::physics
