#include "boids.h"
#include "const.h"

#include <random>
#include <numbers>

namespace boids
{
void BoidManager::Begin()
{
    previousBoids_.resize(boidsNumber);
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> disW(0.0f, static_cast<float>(worldRadius)/pixelPerMeter);
    std::uniform_real_distribution<float> disH(0.0f, static_cast<float>(worldRadius)/pixelPerMeter);
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
    constexpr auto worldCenter = Vec2f{ width / pixelPerMeter * 0.5f,height / pixelPerMeter * 0.5f };
    std::swap(previousBoids_, currentBoids_);
    maxCount_ = 0;
    #pragma omp parallel for 
    for(int i = 0; i < boidsNumber; i++)
    {
        auto& boid = currentBoids_[i];
        boid = previousBoids_[i];

        Vec2f center;
        Vec2f neighborVel;
        std::size_t count = 0;

        Vec2f avoidForce;

        for(int j = 0; j < boidsNumber;j++)
        {
            if(i == j)
                continue;
            const auto& neighborBoid = previousBoids_[j];
            const auto delta = neighborBoid.pos - boid.pos;
            const auto distance = delta.Magnitude();
            if(distance > radius)
                continue;
            neighborVel += neighborBoid.vel;
            const Vec2f dir = delta.Normalized();
            avoidForce -= dir * avoidFactor / distance;

            center += neighborBoid.pos;
            ++count;

        }
        Vec2f totalForce;
        //converge to center of mass
        if (count != 0)
        {
            center = center / static_cast<float>(count);
            const auto deltaToCenter = center - boid.pos;
            if (deltaToCenter.SquareMagnitude() > convergeDeadZone*convergeDeadZone)
            {
                const auto dir = deltaToCenter.Normalized();
                totalForce += dir;
            }
        }
        //avoid neighbors
        totalForce += avoidForce;
        
        //align direction
        if (count != 0)
        {
            neighborVel = neighborVel / static_cast<float>(count);
            const auto deltaVel = neighborVel - boid.vel;
            if (deltaVel.SquareMagnitude() > alignDeadZone * alignDeadZone)
            {
                totalForce += deltaVel.Normalized() * alignFactor;
            }
        }

        //go back to world center if too far
        const auto deltaToCenter = worldCenter - boid.pos;
        if(deltaToCenter.SquareMagnitude() > worldCenterFactor*worldCenterFactor)
        {

            totalForce += deltaToCenter / worldCenterFactor;
        }

        boid.vel += totalForce * dt;
        if(boid.vel.SquareMagnitude() > maxSpeed * maxSpeed)
        {
            boid.vel = boid.vel.Normalized() * maxSpeed;
        }
        boid.pos += boid.vel * dt;

        maxCount_ = std::max(maxCount_, count);
    }
    
}
} // namespace boids
