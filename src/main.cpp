#include "boids.h"
#include <SFML/Graphics.hpp>

#include "const.h"

int main()
{

    sf::RenderWindow window(sf::VideoMode(boids::width, boids::height), "Boids");
    //window.setFramerateLimit(5);
    sf::Clock clock;


    boids::BoidManager boidManager;
    boidManager.Begin();

    sf::VertexArray array;
    array.resize(boids::boidsNumber * 3u);
    for(std::size_t i = 0; i < boids::boidsNumber * 3u; i++)
    {
        array[i].color = sf::Color::Red;
    }
    
    while (window.isOpen())
    {
        auto dt = clock.restart();
        sf::Event event{};
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        boidManager.Update(dt.asSeconds());
        const auto boids = boidManager.GetBoids();
        for(std::size_t i = 0; i < boids::boidsNumber; ++i)
        {
            const auto& boid = boids[i];
            const auto angle = boids::Vec2f::AngleBetween(boid.vel, boids::Vec2f::up());

            auto& topVertex = array[i * 3];
            topVertex.position = static_cast<sf::Vector2f>(boid.pos * boids::pixelPerMeter + boids::Vec2f::up().Rotated(angle));

            auto& leftVertex = array[i * 3 + 1];
            leftVertex.position = static_cast<sf::Vector2f>(boid.pos * boids::pixelPerMeter + boids::Vec2f(-0.5, 0.0f).Rotated(angle));


            auto& rightVertex = array[i * 3 + 2];
            rightVertex.position = static_cast<sf::Vector2f>(boid.pos * boids::pixelPerMeter + boids::Vec2f(0.5f, 0.0f).Rotated(angle));
        }

        window.clear(sf::Color::Black);
        window.draw(array);
        window.display();
    }

    return 0;
}
