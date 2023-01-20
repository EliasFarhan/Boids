#include "boids.h"

#include "imgui.h"
#include "imgui-SFML.h"

#include <SFML/Graphics.hpp>

#include "const.h"

int main()
{

    sf::RenderWindow window(sf::VideoMode(boids::width, boids::height), "Boids");
    //window.setFramerateLimit(5);
    sf::Clock clock;
    ImGui::SFML::Init(window);

    boids::BoidManager boidManager;
    boidManager.Begin();

    sf::VertexArray array;
    array.setPrimitiveType(sf::Triangles);
    array.resize(boids::boidsNumber * 3u);
    for(std::size_t i = 0; i < boids::boidsNumber * 3u; i++)
    {
        array[i].color = sf::Color::Red;
    }
    auto view = window.getDefaultView();
    while (window.isOpen())
    {
        auto dt = clock.restart();
        sf::Event event{};
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if(event.type == sf::Event::Resized)
            {
                view.setSize(event.size.width, event.size.height);
                window.setView(view);
            }
            if(event.type == sf::Event::MouseWheelScrolled)
            {
                view.zoom((1.0f-boids::zoomFactor*event.mouseWheelScroll.delta * dt.asSeconds()));
                window.setView(view);
            }

            ImGui::SFML::ProcessEvent(window, event);
        }
        boidManager.Update2(dt.asSeconds());
        const auto boids = boidManager.GetBoids();
        for(std::size_t i = 0; i < boids::boidsNumber; ++i)
        {
            const auto& boid = boids[i];
            const auto angle = boids::Vec2f::AngleBetween(boids::Vec2f::up(), boid.vel);

            auto& topVertex = array[i * 3];
            topVertex.position = static_cast<sf::Vector2f>(boid.pos * boids::pixelPerMeter + boids::Vec2f::up().Rotated(angle)*boids::boidPixelHeight);

            auto& leftVertex = array[i * 3 + 1];
            leftVertex.position = static_cast<sf::Vector2f>(boid.pos * boids::pixelPerMeter + boids::Vec2f{ -0.5, 0.0f }.Rotated(angle) * boids::boidPixelHeight);


            auto& rightVertex = array[i * 3 + 2];
            rightVertex.position = static_cast<sf::Vector2f>(boid.pos * boids::pixelPerMeter + boids::Vec2f{ 0.5f, 0.0f }.Rotated(angle) * boids::boidPixelHeight);
        }

        ImGui::SFML::Update(window, dt);

        ImGui::Begin("Boids");
        ImGui::Text("FPS: %f", 1.0f / dt.asSeconds());
        ImGui::End();

        window.clear(sf::Color::Black);
        window.draw(array);
        ImGui::SFML::Render(window);
        window.display();
    }

    return 0;
}
