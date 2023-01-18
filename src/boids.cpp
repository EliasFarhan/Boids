#include "boids.h"
#include "const.h"

#include <random>
#include <numbers>

namespace boids
{
void BoidManager::Begin()
{
    previousBoids_.resize(boidsNumber);
    //TODO put random positions and velocities
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> disW(0.0f, static_cast<float>(width)/pixelPerMeter);
    std::uniform_real_distribution<float> disH(0.0f, static_cast<float>(height)/pixelPerMeter);
    std::uniform_real_distribution<float> disAngle(0.0f, std::numbers::pi_v<float>);


    for(auto& boid : previousBoids_)
    {
        boid.pos = Vec2f{ disW(gen), disH(gen)};
        boid.vel = Vec2f::up().Rotated(Radian{ disAngle(gen) });
    }

    currentBoids_.resize(boidsNumber);
    std::swap(previousBoids_, currentBoids_);
}

void BoidManager::Update(float dt)
{
    std::swap(previousBoids_, currentBoids_);
    maxCount_ = 0;
    for(std::size_t i = 0; i < boidsNumber; i++)
    {
        auto& boid = currentBoids_[i];
        boid = previousBoids_[i];

        Vec2f center;
        std::size_t count = 0;

        Vec2f avoidForce;

        for(std::size_t j = 0; j < boidsNumber;j++)
        {
            if(i == j)
                continue;
            const auto& neighborBoid = previousBoids_[j];
            const auto delta = neighborBoid.pos - boid.pos;
            const auto distance = delta.Magnitude();
            if(distance > radius)
                continue;

            Vec2f dir = delta.Normalized();
            avoidForce += dir / distance;
            

            center += neighborBoid.pos;
            ++count;

        }
        Vec2f totalForce;
        //converge to center of mass
        if (count != 0)
        {
            center = center / static_cast<float>(count);
            const auto deltaToCenter = center - boid.pos;
            const auto dist = deltaToCenter.Magnitude();
            const auto dir = deltaToCenter.Normalized();

            totalForce += dir * dist / radius;

        }
        //TODO avoid neighbors
        totalForce += avoidForce;
        //TODO align direction

        //go back to world center if too far
        {
            constexpr auto worldCenter = Vec2f{ width / pixelPerMeter * 0.5f,height / pixelPerMeter * 0.5f };

            const auto deltaToCenter = worldCenter - boid.pos;
            const auto dist = deltaToCenter.Magnitude();
            const auto dir = deltaToCenter.Normalized();

            totalForce += dir * dist / worldCenterFactor;
        }

        boid.vel += totalForce * dt;
        boid.pos += boid.vel * dt;

        maxCount_ = std::max(maxCount_, count);
    }
    
}
} // namespace boids
