#include "OrbitCore/Vector3.hpp"

#include <cmath>
#include <ostream>
#include <stdexcept>

namespace orbitcore {

double Vector3::magnitude() const
{
    return std::sqrt(magnitudeSquared());
}

double Vector3::magnitudeSquared() const noexcept
{
    return x * x + y * y + z * z;
}

Vector3 Vector3::normalized() const
{
    const double length = magnitude();
    if (length == 0.0) {
        throw std::invalid_argument("Cannot normalize a zero-length vector.");
    }
    return *this / length;
}

bool Vector3::isFinite() const noexcept
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double Vector3::dot(const Vector3& other) const noexcept
{
    return x * other.x + y * other.y + z * other.z;
}

Vector3 Vector3::cross(const Vector3& other) const noexcept
{
    return Vector3{
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    };
}

Vector3& Vector3::operator+=(const Vector3& other) noexcept
{
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) noexcept
{
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3& Vector3::operator*=(double scalar) noexcept
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector3& Vector3::operator/=(double scalar)
{
    if (scalar == 0.0) {
        throw std::invalid_argument("Cannot divide a vector by zero.");
    }
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

Vector3 operator+(Vector3 lhs, const Vector3& rhs) noexcept
{
    return lhs += rhs;
}

Vector3 operator-(Vector3 lhs, const Vector3& rhs) noexcept
{
    return lhs -= rhs;
}

Vector3 operator-(const Vector3& value) noexcept
{
    return Vector3{-value.x, -value.y, -value.z};
}

Vector3 operator*(Vector3 value, double scalar) noexcept
{
    return value *= scalar;
}

Vector3 operator*(double scalar, Vector3 value) noexcept
{
    return value *= scalar;
}

Vector3 operator/(Vector3 value, double scalar)
{
    return value /= scalar;
}

std::ostream& operator<<(std::ostream& os, const Vector3& value)
{
    os << '(' << value.x << ", " << value.y << ", " << value.z << ')';
    return os;
}

} // namespace orbitcore
