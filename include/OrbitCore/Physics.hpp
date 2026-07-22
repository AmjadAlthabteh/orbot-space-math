#pragma once

#include "OrbitCore/Vector3.hpp"

namespace orbitcore::physics {

constexpr double gravitationalConstant = 6.67430e-11;
constexpr double earthMassKg = 5.97219e24;
constexpr double earthRadiusMeters = 6.371e6;
constexpr double pi = 3.14159265358979323846;
constexpr double standardGravity = 9.80665;
constexpr double speedOfLightMetersPerSecond = 299792458.0;
constexpr double seaLevelAtmosphericDensity = 1.225;

struct HohmannTransfer {
    double departureDeltaV;
    double arrivalDeltaV;
    double totalDeltaV;
    double transferTimeSeconds;
};

struct ClosestApproach {
    double timeSeconds;
    double distanceMeters;
};

struct OrbitalElements {
    double semiMajorAxisMeters;
    double eccentricity;
    double inclinationRadians;
    double specificAngularMomentum;
};

[[nodiscard]] double distance(const Vector3& from, const Vector3& to);
[[nodiscard]] double degreesToRadians(double degrees);
[[nodiscard]] double radiansToDegrees(double radians);
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
[[nodiscard]] OrbitalElements orbitalElements(
    double centralMassKg,
    const Vector3& position,
    const Vector3& velocity);
[[nodiscard]] double circularOrbitRadius(double centralMassKg, double orbitalSpeedMetersPerSecond);
[[nodiscard]] double sphereOfInfluenceRadius(
    double orbitRadiusMeters,
    double smallerBodyMassKg,
    double largerBodyMassKg);
[[nodiscard]] double hillSphereRadius(
    double orbitRadiusMeters,
    double smallerBodyMassKg,
    double largerBodyMassKg);
[[nodiscard]] double synodicPeriod(double firstOrbitalPeriodSeconds, double secondOrbitalPeriodSeconds);
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
[[nodiscard]] double phaseAngleRadians(const Vector3& fromCentralBodyA, const Vector3& fromCentralBodyB);
[[nodiscard]] double lineOfSightRate(
    const Vector3& observerPosition,
    const Vector3& observerVelocity,
    const Vector3& targetPosition,
    const Vector3& targetVelocity);
[[nodiscard]] ClosestApproach closestApproach(
    const Vector3& relativePosition,
    const Vector3& relativeVelocity);
[[nodiscard]] bool hasPredictedCollisionRisk(
    const Vector3& positionA,
    const Vector3& velocityA,
    const Vector3& positionB,
    const Vector3& velocityB,
    double safetyRadiusMeters,
    double lookaheadSeconds);
[[nodiscard]] double atmosphericDensity(
    double altitudeMeters,
    double referenceDensityKgPerCubicMeter = seaLevelAtmosphericDensity,
    double scaleHeightMeters = 8500.0);
[[nodiscard]] double dragForce(
    double atmosphericDensityKgPerCubicMeter,
    double speedMetersPerSecond,
    double dragCoefficient,
    double crossSectionAreaSquareMeters);
[[nodiscard]] double solarRadiationPressureForce(
    double solarFluxWattsPerSquareMeter,
    double reflectivityCoefficient,
    double crossSectionAreaSquareMeters);
[[nodiscard]] double signalLightTime(double rangeMeters);
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
