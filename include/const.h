#pragma once

namespace boids
{
constexpr int width = 1280;
constexpr int height = 720;
constexpr std::size_t boidsNumber = 1000; //1000
constexpr float radius = 1.5f;
constexpr float avoidFactor = 0.35f;
constexpr float alignFactor = 0.2f;
constexpr float alignDeadZone = 0.01f;
constexpr float convergeFactor = 2.0f;
constexpr float convergeDeadZone = 0.01f;
constexpr float maxSpeed = 4.0f;
constexpr float pixelPerMeter = 100.0f;
constexpr float worldRadius = static_cast<float>(width);
constexpr float worldCenterFactor = worldRadius / pixelPerMeter;
constexpr float boidPixelHeight = 20.0f;
constexpr float zoomFactor = 10.0f;
constexpr auto margin = worldCenterFactor;
constexpr auto worldWidth = static_cast<std::size_t>((2.0f * worldRadius / pixelPerMeter + 2u*margin) / radius);

}