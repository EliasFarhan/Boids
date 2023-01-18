#pragma once

#include <SFML/System/Vector2.hpp>

#include <cmath>
#include <vector>
#include <span>

namespace boids
{

class Radian
{
public:
    explicit constexpr Radian(float angle): angle_(angle){}
    [[nodiscard]] constexpr float GetValue() const { return angle_; }
private:
    float angle_ = 0.0f;
};

struct Vec2f
{
    float x = 0.0f, y = 0.0f;

    constexpr Vec2f operator+(Vec2f v) const
    {
        return {v.x+x, v.y+y};
    }

    constexpr Vec2f& operator+=(Vec2f v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    constexpr Vec2f operator-(Vec2f v) const
    {
        return {x-v.x, y-v.y};
    }

    constexpr Vec2f operator*(float f) const
    {
        return {x*f, y*f};
    }

    constexpr Vec2f operator/(float f) const
    {
        return {x/f, y/f};
    }

    constexpr static float Dot(Vec2f v1, Vec2f v2)
    {
        return v1.x*v2.x+v1.y*v2.y;
    }

    constexpr static float SquareDistance(Vec2f v1, Vec2f v2)
    {
        const auto v = v2 - v1;
        return Dot(v, v);
    }

    static float Distance(Vec2f v1, Vec2f v2)
    {
        return std::sqrt(SquareDistance(v1, v2));
    }

    [[nodiscard]] constexpr float SquareMagnitude() const
    {
        return x*x+y*y;
    }

    [[nodiscard]] float Magnitude() const
    {
        return std::sqrt(SquareMagnitude());
    }

    [[nodiscard]] constexpr Vec2f Normalized() const
    {
        return (*this) / Magnitude();
    }

    constexpr static Radian AngleBetween(Vec2f v1, Vec2f v2)
    {
        return Radian{ std::acos(Dot(v1, v2)/v1.Magnitude()/v2.Magnitude()) };
    }

    constexpr static Vec2f up()
    {
        return { 0,1 };
    }

    [[nodiscard]] Vec2f Rotated(Radian r) const
    {
        const float sin = std::sin(r.GetValue());
        const float cos = std::cos(r.GetValue());
        return { (cos * x) - (sin * y) , (sin * x) + (cos * y) };
    }

    explicit operator sf::Vector2f() const { return sf::Vector2f(x, y); }
};


struct Boid
{
    Vec2f pos;
    Vec2f vel;
};

class BoidManager
{
public:
    void Begin();
    void Update(float dt);
    [[nodiscard]] std::span<Boid> GetBoids() { return currentBoids_; }
    [[nodiscard]] std::size_t GetMaxNeighborCount() const { return maxCount_; }
private:
    std::vector<Boid> previousBoids_;
    std::vector<Boid> currentBoids_;
    std::size_t maxCount_ = 0;
};
} // namespace boids