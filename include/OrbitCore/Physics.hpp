#pragma once

#include "OrbitCore/Vector3.hpp"

namespace orbitcore::physics {

constexpr double gravitationalConstant = 6.67430e-11;
constexpr double earthMassKg = 5.97219e24;
constexpr double earthRadiusMeters = 6.371e6;
constexpr double standardGravity = 9.80665;

struct HohmannTransfer {
    double departureDeltaV;
    double arrivalDeltaV;
    double totalDeltaV;
    double transferTimeSeconds;
};

[[nodiscard]] double distance(const Vector3& from, const Vector3& to);
[[nodiscard]] double gravitationalForce(double massA, double massB, double separationMeters);
[[nodiscard]] double orbitalVelocity(double centralMassKg, double orbitRadiusMeters);
[[nodiscard]] double escapeVelocity(double centralMassKg, double radiusMeters);
[[nodiscard]] double orbitalPeriod(double centralMassKg, double orbitRadiusMeters);
[[nodiscard]] double kineticEnergy(double massKg, const Vector3& velocity);
[[nodiscard]] double gravitationalPotentialEnergy(double massKg, double centralMassKg, double radiusMeters);
[[nodiscard]] double specificOrbitalEnergy(
    double centralMassKg,
    const Vector3& position,
    const Vector3& velocity);
[[nodiscard]] double thrustAcceleration(double thrustNewtons, double spacecraftMassKg);
[[nodiscard]] double deltaVFromSpecificImpulse(
    double specificImpulseSeconds,
    double initialMassKg,
    double finalMassKg);
[[nodiscard]] HohmannTransfer hohmannTransfer(
    double centralMassKg,
    double startingOrbitRadiusMeters,
    double targetOrbitRadiusMeters);
[[nodiscard]] Vector3 gravitationalAcceleration(
    const Vector3& position,
    const Vector3& centralBodyPosition,
    double centralMassKg);
[[nodiscard]] Vector3 estimateFuturePosition(
    const Vector3& position,
    const Vector3& velocity,
    const Vector3& acceleration,
    double deltaTimeSeconds);
[[nodiscard]] Vector3 directionToward(const Vector3& from, const Vector3& to);
[[nodiscard]] bool hasCollisionRisk(
    const Vector3& positionA,
    const Vector3& positionB,
    double safetyRadiusMeters);
[[nodiscard]] double fuelUsageForManeuver(
    double spacecraftMassKg,
    double deltaVMetersPerSecond,
    double engineEfficiency);

void validatePositive(double value, const char* name);
void validateNonNegative(double value, const char* name);

} // namespace orbitcore::physics
