#pragma once

#include <iosfwd>

namespace orbitcore {

class Vector3 {
public:
    double x;
    double y;
    double z;

    constexpr Vector3() noexcept : x(0.0), y(0.0), z(0.0) {}
    constexpr Vector3(double xValue, double yValue, double zValue) noexcept
        : x(xValue), y(yValue), z(zValue) {}

    [[nodiscard]] double magnitude() const;
    [[nodiscard]] double magnitudeSquared() const noexcept;
    [[nodiscard]] Vector3 normalized() const;
    [[nodiscard]] bool isFinite() const noexcept;

    [[nodiscard]] double dot(const Vector3& other) const noexcept;
    [[nodiscard]] Vector3 cross(const Vector3& other) const noexcept;

    Vector3& operator+=(const Vector3& other) noexcept;
    Vector3& operator-=(const Vector3& other) noexcept;
    Vector3& operator*=(double scalar) noexcept;
    Vector3& operator/=(double scalar);
};

[[nodiscard]] Vector3 operator+(Vector3 lhs, const Vector3& rhs) noexcept;
[[nodiscard]] Vector3 operator-(Vector3 lhs, const Vector3& rhs) noexcept;
[[nodiscard]] Vector3 operator-(const Vector3& value) noexcept;
[[nodiscard]] Vector3 operator*(Vector3 value, double scalar) noexcept;
[[nodiscard]] Vector3 operator*(double scalar, Vector3 value) noexcept;
[[nodiscard]] Vector3 operator/(Vector3 value, double scalar);

std::ostream& operator<<(std::ostream& os, const Vector3& value);

} // namespace orbitcore
