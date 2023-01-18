#pragma once

namespace boids
{
constexpr int width = 1280;
constexpr int height = 720;
constexpr std::size_t boidsNumber = 1000;
constexpr float radius = 2.0f;
constexpr float pixelPerMeter = 100.0f;
constexpr float worldCenterFactor = static_cast<float>(width) / pixelPerMeter;

}