#include "OrbitCore/Physics.hpp"

#include <algorithm>
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

double degreesToRadians(double degrees)
{
    if (!std::isfinite(degrees)) {
        throw std::invalid_argument("degrees must be finite.");
    }
    return degrees * pi / 180.0;
}

double radiansToDegrees(double radians)
{
    if (!std::isfinite(radians)) {
        throw std::invalid_argument("radians must be finite.");
    }
    return radians * 180.0 / pi;
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

OrbitalElements orbitalElements(
    double centralMassKg,
    const Vector3& position,
    const Vector3& velocity)
{
    validatePositive(centralMassKg, "centralMassKg");
    if (!position.isFinite() || !velocity.isFinite()) {
        throw std::invalid_argument("position and velocity must be finite.");
    }

    const double mu = gravitationalConstant * centralMassKg;
    const double radius = position.magnitude();
    validatePositive(radius, "position radius");

    const Vector3 angularMomentum = position.cross(velocity);
    const double h = angularMomentum.magnitude();
    validatePositive(h, "specific angular momentum");

    const double energy = specificOrbitalEnergy(centralMassKg, position, velocity);
    if (energy == 0.0) {
        throw std::invalid_argument("Parabolic trajectories do not have a finite semi-major axis.");
    }

    const Vector3 eccentricityVector = (velocity.cross(angularMomentum) / mu) - (position / radius);
    const double cosineInclination = angularMomentum.z / h;
    const double boundedCosine = std::max(-1.0, std::min(1.0, cosineInclination));

    return OrbitalElements{
        -mu / (2.0 * energy),
        eccentricityVector.magnitude(),
        std::acos(boundedCosine),
        h
    };
}

double circularOrbitRadius(double centralMassKg, double orbitalSpeedMetersPerSecond)
{
    validatePositive(centralMassKg, "centralMassKg");
    validatePositive(orbitalSpeedMetersPerSecond, "orbitalSpeedMetersPerSecond");
    return gravitationalConstant * centralMassKg / (orbitalSpeedMetersPerSecond * orbitalSpeedMetersPerSecond);
}

double sphereOfInfluenceRadius(
    double orbitRadiusMeters,
    double smallerBodyMassKg,
    double largerBodyMassKg)
{
    validatePositive(orbitRadiusMeters, "orbitRadiusMeters");
    validatePositive(smallerBodyMassKg, "smallerBodyMassKg");
    validatePositive(largerBodyMassKg, "largerBodyMassKg");
    return orbitRadiusMeters * std::pow(smallerBodyMassKg / largerBodyMassKg, 2.0 / 5.0);
}

double hillSphereRadius(
    double orbitRadiusMeters,
    double smallerBodyMassKg,
    double largerBodyMassKg)
{
    validatePositive(orbitRadiusMeters, "orbitRadiusMeters");
    validatePositive(smallerBodyMassKg, "smallerBodyMassKg");
    validatePositive(largerBodyMassKg, "largerBodyMassKg");
    return orbitRadiusMeters * std::cbrt(smallerBodyMassKg / (3.0 * largerBodyMassKg));
}

double synodicPeriod(double firstOrbitalPeriodSeconds, double secondOrbitalPeriodSeconds)
{
    validatePositive(firstOrbitalPeriodSeconds, "firstOrbitalPeriodSeconds");
    validatePositive(secondOrbitalPeriodSeconds, "secondOrbitalPeriodSeconds");
    const double relativeRate = std::abs((1.0 / firstOrbitalPeriodSeconds) - (1.0 / secondOrbitalPeriodSeconds));
    validatePositive(relativeRate, "relative orbital rate");
    return 1.0 / relativeRate;
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

double phaseAngleRadians(const Vector3& fromCentralBodyA, const Vector3& fromCentralBodyB)
{
    const double radiusA = fromCentralBodyA.magnitude();
    const double radiusB = fromCentralBodyB.magnitude();
    validatePositive(radiusA, "fromCentralBodyA radius");
    validatePositive(radiusB, "fromCentralBodyB radius");

    const double cosineAngle = fromCentralBodyA.dot(fromCentralBodyB) / (radiusA * radiusB);
    return std::acos(std::max(-1.0, std::min(1.0, cosineAngle)));
}

double lineOfSightRate(
    const Vector3& observerPosition,
    const Vector3& observerVelocity,
    const Vector3& targetPosition,
    const Vector3& targetVelocity)
{
    const Vector3 relativePosition = targetPosition - observerPosition;
    const Vector3 relativeVelocity = targetVelocity - observerVelocity;
    const double range = relativePosition.magnitude();
    validatePositive(range, "range");
    return relativePosition.dot(relativeVelocity) / range;
}

ClosestApproach closestApproach(
    const Vector3& relativePosition,
    const Vector3& relativeVelocity)
{
    if (!relativePosition.isFinite() || !relativeVelocity.isFinite()) {
        throw std::invalid_argument("relativePosition and relativeVelocity must be finite.");
    }

    const double speedSquared = relativeVelocity.magnitudeSquared();
    if (speedSquared == 0.0) {
        return ClosestApproach{0.0, relativePosition.magnitude()};
    }

    const double unconstrainedTime = -relativePosition.dot(relativeVelocity) / speedSquared;
    const double time = std::max(0.0, unconstrainedTime);
    return ClosestApproach{time, (relativePosition + relativeVelocity * time).magnitude()};
}

double atmosphericDensity(
    double altitudeMeters,
    double referenceDensityKgPerCubicMeter,
    double scaleHeightMeters)
{
    validateNonNegative(altitudeMeters, "altitudeMeters");
    validatePositive(referenceDensityKgPerCubicMeter, "referenceDensityKgPerCubicMeter");
    validatePositive(scaleHeightMeters, "scaleHeightMeters");
    return referenceDensityKgPerCubicMeter * std::exp(-altitudeMeters / scaleHeightMeters);
}

double dragForce(
    double atmosphericDensityKgPerCubicMeter,
    double speedMetersPerSecond,
    double dragCoefficient,
    double crossSectionAreaSquareMeters)
{
    validateNonNegative(atmosphericDensityKgPerCubicMeter, "atmosphericDensityKgPerCubicMeter");
    validateNonNegative(speedMetersPerSecond, "speedMetersPerSecond");
    validatePositive(dragCoefficient, "dragCoefficient");
    validatePositive(crossSectionAreaSquareMeters, "crossSectionAreaSquareMeters");
    return 0.5 * atmosphericDensityKgPerCubicMeter * speedMetersPerSecond * speedMetersPerSecond * dragCoefficient * crossSectionAreaSquareMeters;
}

double solarRadiationPressureForce(
    double solarFluxWattsPerSquareMeter,
    double reflectivityCoefficient,
    double crossSectionAreaSquareMeters)
{
    validatePositive(solarFluxWattsPerSquareMeter, "solarFluxWattsPerSquareMeter");
    validateNonNegative(reflectivityCoefficient, "reflectivityCoefficient");
    validatePositive(crossSectionAreaSquareMeters, "crossSectionAreaSquareMeters");
    return solarFluxWattsPerSquareMeter * reflectivityCoefficient * crossSectionAreaSquareMeters / speedOfLightMetersPerSecond;
}

double signalLightTime(double rangeMeters)
{
    validateNonNegative(rangeMeters, "rangeMeters");
    return rangeMeters / speedOfLightMetersPerSecond;
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
