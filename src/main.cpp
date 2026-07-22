#include "OrbitCore/Physics.hpp"
#include "OrbitCore/SpaceBase.hpp"
#include "OrbitCore/Spacecraft.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

void printStep(
    int step,
    const orbitcore::Spacecraft& craft,
    const orbitcore::Vector3& target,
    const orbitcore::SpaceBase& base,
    const orbitcore::Vector3& companionPosition)
{
    const double targetDistance = orbitcore::physics::distance(craft.position(), target);
    const double baseDistance = base.distanceTo(craft.position());
    const bool collisionWarning = orbitcore::physics::hasCollisionRisk(
        craft.position(),
        companionPosition,
        500.0);

    std::cout << "Step " << std::setw(2) << step
              << " | position " << craft.position()
              << " | velocity " << craft.velocity()
              << " | target distance " << targetDistance << " m"
              << " | base distance " << baseDistance << " m"
              << " | fuel " << craft.fuelKg() << " kg";

    if (collisionWarning) {
        std::cout << " | COLLISION RISK";
    }
    if (base.isDockingRange(craft.position())) {
        std::cout << " | DOCKING WINDOW";
    }
    std::cout << '\n';
}

} // namespace

int main()
{
    using orbitcore::Spacecraft;
    using orbitcore::Vector3;
    namespace physics = orbitcore::physics;

    try {
        const Vector3 earthCenter{0.0, 0.0, 0.0};
        const Vector3 target{7.05e6, 12000.0, 900.0};
        Vector3 nearbyCraftPosition{physics::earthRadiusMeters + 420300.0, 200.0, 0.0};
        const orbitcore::SpaceBase orbitalBase{
            "Aster Gate",
            Vector3{physics::earthRadiusMeters + 421500.0, 45000.0, 120.0},
            2500.0
        };

        Spacecraft explorer{
            "OrbitCore-1",
            1200.0,
            420.0,
            Vector3{physics::earthRadiusMeters + 420000.0, 0.0, 0.0},
            Vector3{0.0, physics::orbitalVelocity(physics::earthMassKg, physics::earthRadiusMeters + 420000.0), 0.0}
        };

        const double startingDistance = physics::distance(explorer.position(), target);
        const double force = physics::gravitationalForce(
            explorer.totalMassKg(),
            physics::earthMassKg,
            explorer.position().magnitude());
        const Vector3 direction = physics::directionToward(explorer.position(), target);
        const Vector3 baseDirection = orbitalBase.directionFrom(explorer.position());
        const Vector3 angularMomentumHint = explorer.position().cross(explorer.velocity());
        const double radialVelocity = explorer.velocity().dot(explorer.position().normalized());
        const double escapeSpeed = physics::escapeVelocity(physics::earthMassKg, explorer.position().magnitude());
        const double orbitPeriod = physics::orbitalPeriod(physics::earthMassKg, explorer.position().magnitude());
        const double kinetic = physics::kineticEnergy(explorer.totalMassKg(), explorer.velocity());
        const double potential = physics::gravitationalPotentialEnergy(
            explorer.totalMassKg(),
            physics::earthMassKg,
            explorer.position().magnitude());
        const double specificEnergy = physics::specificOrbitalEnergy(
            physics::earthMassKg,
            explorer.position(),
            explorer.velocity());
        const auto transfer = physics::hohmannTransfer(
            physics::earthMassKg,
            explorer.position().magnitude(),
            physics::earthRadiusMeters + 900000.0);

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "OrbitCore spacecraft navigation simulator\n";
        std::cout << "Initial target distance: " << startingDistance << " m\n";
        std::cout << "Nearest space base: " << orbitalBase.name()
                  << " at " << orbitalBase.distanceTo(explorer.position()) << " m\n";
        std::cout << "Initial Earth gravity force: " << force << " N\n";
        std::cout << "Escape velocity here: " << escapeSpeed << " m/s\n";
        std::cout << "Orbital period estimate: " << orbitPeriod << " s\n";
        std::cout << "Kinetic energy: " << kinetic << " J\n";
        std::cout << "Gravitational potential energy: " << potential << " J\n";
        std::cout << "Specific orbital energy: " << specificEnergy << " J/kg\n";
        std::cout << "Hohmann transfer to 900 km orbit: delta-v " << transfer.totalDeltaV
                  << " m/s, transfer time " << transfer.transferTimeSeconds << " s\n";
        std::cout << "Required target direction: " << direction << '\n';
        std::cout << "Required base direction: " << baseDirection << '\n';
        std::cout << "Position x velocity cross product: " << angularMomentumHint << '\n';
        std::cout << "Radial velocity dot product: " << radialVelocity << " m/s\n\n";

        const double maneuverDeltaV = 18.0;
        const double fuelUsed = physics::fuelUsageForManeuver(explorer.totalMassKg(), maneuverDeltaV, 0.72);
        const double chemicalDeltaV = physics::deltaVFromSpecificImpulse(
            310.0,
            explorer.totalMassKg(),
            explorer.totalMassKg() - fuelUsed);
        const double burnAcceleration = physics::thrustAcceleration(24000.0, explorer.totalMassKg());
        explorer.consumeFuel(fuelUsed);
        explorer.setVelocity(explorer.velocity() + direction * maneuverDeltaV);
        std::cout << "Maneuver fuel burn: " << fuelUsed << " kg"
                  << " | rocket equation delta-v check " << chemicalDeltaV << " m/s"
                  << " | thrust acceleration " << burnAcceleration << " m/s^2\n\n";

        constexpr double timeStepSeconds = 10.0;
        for (int step = 0; step <= 20; ++step) {
            printStep(step, explorer, target, orbitalBase, nearbyCraftPosition);

            const Vector3 acceleration = physics::gravitationalAcceleration(
                explorer.position(),
                earthCenter,
                physics::earthMassKg);
            const Vector3 predicted = physics::estimateFuturePosition(
                explorer.position(),
                explorer.velocity(),
                acceleration,
                timeStepSeconds);

            explorer.applyAcceleration(acceleration, timeStepSeconds);
            explorer.setPosition(predicted);
            nearbyCraftPosition += Vector3{12.0, 60.0, 0.0};
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
