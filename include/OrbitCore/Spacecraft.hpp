#pragma once

#include "OrbitCore/Vector3.hpp"

#include <string>

namespace orbitcore {

class Spacecraft {
public:
    Spacecraft(
        std::string name,
        double dryMassKg,
        double fuelKg,
        Vector3 positionMeters,
        Vector3 velocityMetersPerSecond);

    ~Spacecraft() = default;
    Spacecraft(const Spacecraft&) = default;
    Spacecraft& operator=(const Spacecraft&) = default;
    Spacecraft(Spacecraft&&) noexcept = default;
    Spacecraft& operator=(Spacecraft&&) noexcept = default;

    [[nodiscard]] const std::string& name() const noexcept;
    [[nodiscard]] double dryMassKg() const noexcept;
    [[nodiscard]] double fuelKg() const noexcept;
    [[nodiscard]] double totalMassKg() const noexcept;
    [[nodiscard]] const Vector3& position() const noexcept;
    [[nodiscard]] const Vector3& velocity() const noexcept;

    void setPosition(const Vector3& positionMeters);
    void setVelocity(const Vector3& velocityMetersPerSecond);
    void applyAcceleration(const Vector3& accelerationMetersPerSecondSquared, double deltaTimeSeconds);
    void advance(double deltaTimeSeconds);
    void consumeFuel(double amountKg);

private:
    std::string name_;
    double dryMassKg_;
    double fuelKg_;
    Vector3 positionMeters_;
    Vector3 velocityMetersPerSecond_;
};

} // namespace orbitcore
