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
    const orbitcore::Vector3& companionPosition,
    const orbitcore::Vector3& companionVelocity)
{
    const double targetDistance = orbitcore::physics::distance(craft.position(), target);
    const double baseDistance = base.distanceTo(craft.position());
    const bool collisionWarning = orbitcore::physics::hasCollisionRisk(
        craft.position(),
        companionPosition,
        500.0);
    const bool predictedCollision = orbitcore::physics::hasPredictedCollisionRisk(
        craft.position(),
        craft.velocity(),
        companionPosition,
        companionVelocity,
        1000.0,
        60.0);

    std::cout << "Step " << std::setw(2) << step
              << " | position " << craft.position()
              << " | velocity " << craft.velocity()
              << " | target distance " << targetDistance << " m"
              << " | base distance " << baseDistance << " m"
              << " | fuel " << craft.fuelKg() << " kg";

    if (collisionWarning) {
        std::cout << " | COLLISION RISK";
    }
    if (predictedCollision) {
        std::cout << " | TRAJECTORY CONFLICT";
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
        const Vector3 nearbyCraftVelocity{12.0, 60.0, 0.0};
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
        const auto elements = physics::orbitalElements(
            physics::earthMassKg,
            explorer.position(),
            explorer.velocity());
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
        const double phaseAngle = physics::phaseAngleRadians(explorer.position(), target);
        const double targetRangeRate = physics::lineOfSightRate(
            explorer.position(),
            explorer.velocity(),
            target,
            Vector3{0.0, 7600.0, 0.0});
        const auto approach = physics::closestApproach(
            nearbyCraftPosition - explorer.position(),
            nearbyCraftVelocity - explorer.velocity());
        const double density = physics::atmosphericDensity(explorer.position().magnitude() - physics::earthRadiusMeters);
        const double drag = physics::dragForce(density, explorer.velocity().magnitude(), 2.2, 18.0);
        const double solarPressure = physics::solarRadiationPressureForce(1361.0, 1.35, 22.0);
        const double lightTime = physics::signalLightTime(startingDistance);

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "OrbitCore spacecraft navigation simulator\n";
        std::cout << "Initial target distance: " << startingDistance << " m\n";
        std::cout << "Nearest space base: " << orbitalBase.name()
                  << " at " << orbitalBase.distanceTo(explorer.position()) << " m\n";
        std::cout << "Initial Earth gravity force: " << force << " N\n";
        std::cout << "Escape velocity here: " << escapeSpeed << " m/s\n";
        std::cout << "Orbital period estimate: " << orbitPeriod << " s\n";
        std::cout << "Orbital elements: semi-major axis " << elements.semiMajorAxisMeters
                  << " m, eccentricity " << elements.eccentricity
                  << ", inclination " << elements.inclinationRadians << " rad / "
                  << physics::radiansToDegrees(elements.inclinationRadians) << " deg\n";
        std::cout << "Kinetic energy: " << kinetic << " J\n";
        std::cout << "Gravitational potential energy: " << potential << " J\n";
        std::cout << "Specific orbital energy: " << specificEnergy << " J/kg\n";
        std::cout << "Hohmann transfer to 900 km orbit: delta-v " << transfer.totalDeltaV
                  << " m/s, transfer time " << transfer.transferTimeSeconds << " s\n";
        std::cout << "Phase angle to target: " << phaseAngle << " rad / "
                  << physics::radiansToDegrees(phaseAngle) << " deg\n";
        std::cout << "Line-of-sight range rate: " << targetRangeRate << " m/s\n";
        std::cout << "Closest approach with nearby craft: " << approach.distanceMeters
                  << " m in " << approach.timeSeconds << " s\n";
        std::cout << "Atmospheric density estimate: " << density << " kg/m^3\n";
        std::cout << "Drag force estimate: " << drag << " N\n";
        std::cout << "Solar radiation pressure force: " << solarPressure << " N\n";
        std::cout << "Signal light time to target: " << lightTime << " s\n";
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
            printStep(step, explorer, target, orbitalBase, nearbyCraftPosition, nearbyCraftVelocity);

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
            nearbyCraftPosition += nearbyCraftVelocity;
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return 1;
    }
}
