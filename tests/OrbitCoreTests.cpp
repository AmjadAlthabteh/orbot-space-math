#include "OrbitCore/Physics.hpp"
#include "OrbitCore/SpaceBase.hpp"
#include "OrbitCore/Spacecraft.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

bool nearlyEqual(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Callable>
void requireThrows(Callable callable, const std::string& message)
{
    try {
        callable();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
}

} // namespace

int main()
{
    using orbitcore::Spacecraft;
    using orbitcore::Vector3;
    namespace physics = orbitcore::physics;

    try {
        const Vector3 a{3.0, 4.0, 0.0};
        const Vector3 b{1.0, 2.0, 3.0};

        require(nearlyEqual(a.magnitude(), 5.0, 1e-12), "Vector magnitude failed.");
        require(nearlyEqual(a.normalized().magnitude(), 1.0, 1e-12), "Vector normalization failed.");
        require(nearlyEqual(a.dot(b), 11.0, 1e-12), "Vector dot product failed.");
        require(a.cross(b).x == 12.0 && a.cross(b).y == -9.0 && a.cross(b).z == 2.0, "Vector cross product failed.");

        require(nearlyEqual(physics::distance(Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 3.0, 4.0}), 5.0, 1e-12), "Distance failed.");
        require(nearlyEqual(physics::degreesToRadians(180.0), physics::pi, 1e-12), "Degrees to radians failed.");
        require(nearlyEqual(physics::radiansToDegrees(physics::pi), 180.0, 1e-12), "Radians to degrees failed.");
        require(nearlyEqual(physics::orbitalVelocity(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 7672.599, 1.0), "Orbital velocity failed.");
        require(nearlyEqual(physics::escapeVelocity(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 10850.691, 1.0), "Escape velocity failed.");
        require(nearlyEqual(physics::orbitalPeriod(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 5544.901, 2.0), "Orbital period failed.");
        require(nearlyEqual(physics::circularOrbitRadius(physics::earthMassKg, 7672.599), physics::earthRadiusMeters + 400000.0, 2000.0), "Circular orbit radius failed.");
        require(nearlyEqual(physics::kineticEnergy(2.0, Vector3{3.0, 0.0, 4.0}), 25.0, 1e-12), "Kinetic energy failed.");
        require(physics::gravitationalPotentialEnergy(1000.0, physics::earthMassKg, physics::earthRadiusMeters) < 0.0, "Potential energy sign failed.");
        require(physics::specificOrbitalEnergy(physics::earthMassKg, Vector3{physics::earthRadiusMeters + 400000.0, 0.0, 0.0}, Vector3{0.0, 7672.599, 0.0}) < 0.0, "Specific orbital energy failed.");
        const auto elements = physics::orbitalElements(
            physics::earthMassKg,
            Vector3{physics::earthRadiusMeters + 400000.0, 0.0, 0.0},
            Vector3{0.0, physics::orbitalVelocity(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 0.0});
        require(nearlyEqual(elements.eccentricity, 0.0, 1e-12), "Orbital eccentricity failed.");
        require(nearlyEqual(elements.inclinationRadians, 0.0, 1e-12), "Orbital inclination failed.");
        require(nearlyEqual(elements.semiMajorAxisMeters, physics::earthRadiusMeters + 400000.0, 1e-6), "Semi-major axis failed.");
        require(physics::sphereOfInfluenceRadius(1.496e11, 5.97219e24, 1.9885e30) > 9.0e8, "Sphere of influence failed.");
        require(physics::hillSphereRadius(1.496e11, 5.97219e24, 1.9885e30) > 1.0e9, "Hill sphere failed.");
        require(physics::synodicPeriod(365.25 * 86400.0, 686.98 * 86400.0) > 6.0e7, "Synodic period failed.");
        require(nearlyEqual(physics::thrustAcceleration(2000.0, 500.0), 4.0, 1e-12), "Thrust acceleration failed.");
        require(physics::deltaVFromSpecificImpulse(300.0, 1000.0, 900.0) > 300.0, "Rocket equation delta-v failed.");
        const auto transfer = physics::hohmannTransfer(physics::earthMassKg, physics::earthRadiusMeters + 400000.0, physics::earthRadiusMeters + 800000.0);
        require(transfer.totalDeltaV > 200.0 && transfer.transferTimeSeconds > 2700.0, "Hohmann transfer failed.");
        require(nearlyEqual(physics::phaseAngleRadians(Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0}), 1.57079632679, 1e-10), "Phase angle failed.");
        require(nearlyEqual(physics::lineOfSightRate(Vector3{0.0, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, Vector3{100.0, 0.0, 0.0}, Vector3{-2.0, 0.0, 0.0}), -2.0, 1e-12), "Line-of-sight rate failed.");
        const auto closest = physics::closestApproach(Vector3{100.0, 10.0, 0.0}, Vector3{-10.0, 0.0, 0.0});
        require(nearlyEqual(closest.timeSeconds, 10.0, 1e-12), "Closest approach time failed.");
        require(nearlyEqual(closest.distanceMeters, 10.0, 1e-12), "Closest approach distance failed.");
        require(physics::hasPredictedCollisionRisk(
            Vector3{0.0, 0.0, 0.0},
            Vector3{10.0, 0.0, 0.0},
            Vector3{100.0, 5.0, 0.0},
            Vector3{-10.0, 0.0, 0.0},
            10.0,
            6.0), "Predicted collision risk failed.");
        require(physics::atmosphericDensity(8500.0) < physics::seaLevelAtmosphericDensity, "Atmospheric density failed.");
        require(nearlyEqual(physics::dragForce(1.0, 10.0, 2.0, 3.0), 300.0, 1e-12), "Drag force failed.");
        require(physics::solarRadiationPressureForce(1361.0, 1.0, 10.0) > 0.0, "Solar radiation pressure failed.");
        require(nearlyEqual(physics::signalLightTime(physics::speedOfLightMetersPerSecond), 1.0, 1e-12), "Signal light time failed.");
        require(physics::hasCollisionRisk(Vector3{0.0, 0.0, 0.0}, Vector3{3.0, 4.0, 0.0}, 5.0), "Collision risk failed.");

        orbitcore::SpaceBase base{"Base", Vector3{100.0, 0.0, 0.0}, 10.0};
        require(nearlyEqual(base.distanceTo(Vector3{90.0, 0.0, 0.0}), 10.0, 1e-12), "SpaceBase distance failed.");
        require(base.isDockingRange(Vector3{90.0, 0.0, 0.0}), "SpaceBase docking range failed.");
        require(nearlyEqual(base.directionFrom(Vector3{0.0, 0.0, 0.0}).x, 1.0, 1e-12), "SpaceBase direction failed.");

        Spacecraft craft{"Test", 1000.0, 100.0, Vector3{10.0, 0.0, 0.0}, Vector3{0.0, 2.0, 0.0}};
        craft.applyAcceleration(Vector3{1.0, 0.0, 0.0}, 2.0);
        craft.advance(3.0);
        require(nearlyEqual(craft.position().x, 16.0, 1e-12), "Spacecraft x integration failed.");
        require(nearlyEqual(craft.position().y, 6.0, 1e-12), "Spacecraft y integration failed.");
        craft.consumeFuel(12.5);
        require(nearlyEqual(craft.fuelKg(), 87.5, 1e-12), "Fuel accounting failed.");

        requireThrows([] {
            const auto value = Vector3{}.normalized();
            static_cast<void>(value);
        }, "Zero vector normalization should throw.");
        requireThrows([] {
            const auto value = physics::gravitationalForce(1.0, 1.0, 0.0);
            static_cast<void>(value);
        }, "Zero separation should throw.");
        requireThrows([&craft] { craft.consumeFuel(1000.0); }, "Overburn should throw.");

        std::cout << "OrbitCore tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "OrbitCore tests failed: " << error.what() << '\n';
        return 1;
    }
}
