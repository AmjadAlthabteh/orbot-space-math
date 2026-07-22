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
        require(nearlyEqual(physics::orbitalVelocity(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 7672.599, 1.0), "Orbital velocity failed.");
        require(nearlyEqual(physics::escapeVelocity(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 10850.691, 1.0), "Escape velocity failed.");
        require(nearlyEqual(physics::orbitalPeriod(physics::earthMassKg, physics::earthRadiusMeters + 400000.0), 5544.901, 2.0), "Orbital period failed.");
        require(nearlyEqual(physics::kineticEnergy(2.0, Vector3{3.0, 0.0, 4.0}), 25.0, 1e-12), "Kinetic energy failed.");
        require(physics::gravitationalPotentialEnergy(1000.0, physics::earthMassKg, physics::earthRadiusMeters) < 0.0, "Potential energy sign failed.");
        require(physics::specificOrbitalEnergy(physics::earthMassKg, Vector3{physics::earthRadiusMeters + 400000.0, 0.0, 0.0}, Vector3{0.0, 7672.599, 0.0}) < 0.0, "Specific orbital energy failed.");
        require(nearlyEqual(physics::thrustAcceleration(2000.0, 500.0), 4.0, 1e-12), "Thrust acceleration failed.");
        require(physics::deltaVFromSpecificImpulse(300.0, 1000.0, 900.0) > 300.0, "Rocket equation delta-v failed.");
        const auto transfer = physics::hohmannTransfer(physics::earthMassKg, physics::earthRadiusMeters + 400000.0, physics::earthRadiusMeters + 800000.0);
        require(transfer.totalDeltaV > 200.0 && transfer.transferTimeSeconds > 2700.0, "Hohmann transfer failed.");
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
