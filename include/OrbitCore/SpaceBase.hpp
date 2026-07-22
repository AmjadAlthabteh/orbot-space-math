#pragma once

#include "OrbitCore/Vector3.hpp"

#include <string>

namespace orbitcore {

class SpaceBase {
public:
    SpaceBase(std::string name, Vector3 positionMeters, double dockingRadiusMeters);

    [[nodiscard]] const std::string& name() const noexcept;
    [[nodiscard]] const Vector3& position() const noexcept;
    [[nodiscard]] double dockingRadiusMeters() const noexcept;

    [[nodiscard]] double distanceTo(const Vector3& spacecraftPosition) const;
    [[nodiscard]] Vector3 directionFrom(const Vector3& spacecraftPosition) const;
    [[nodiscard]] bool isDockingRange(const Vector3& spacecraftPosition) const;

private:
    std::string name_;
    Vector3 positionMeters_;
    double dockingRadiusMeters_;
};

} // namespace orbitcore
