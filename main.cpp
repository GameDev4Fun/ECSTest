#include <thread>
#include <chrono>
using namespace std::literals::chrono_literals;

#include <iostream>

#include <entt/entt.hpp>

#include <fmt/core.h>

struct Vector3
{
    float X;
    float Y;
    float Z;
};

struct Position
{
    float x, y, z;
};

struct Velocity
{
    float dx, dy, dz;
};

struct RigidBody
{
    std::array<Vector3, 255> forces;
    float mass;

    RigidBody(float mass) : mass(mass){}
};

struct PhysicSystem
{
    PhysicSystem(entt::registry& world) : world(world)
    {

    }

    void Update(std::chrono::milliseconds deltaTime)
    {
        auto view = world.view<Velocity, const RigidBody>();
        view.each([deltaTime](const auto entity, Velocity& velocity, const RigidBody& rigidbody)
          {
            Vector3 totalForce {0.0f, 0.0f, 0.0f};
            for(auto vec : rigidbody.forces)
            {
                totalForce.X += vec.X;
                totalForce.Y += vec.Y;
                totalForce.Z += vec.Z;
            }
            Vector3 accel {0.0f, 0.0f, 0.0f};
            accel.Z = totalForce.Z / rigidbody.mass;
              accel.Y = totalForce.Y / rigidbody.mass;
              accel.X = totalForce.X / rigidbody.mass;
            velocity.dz += accel.Z * (deltaTime.count()/1000.0f);
              velocity.dy += accel.Y * (deltaTime.count()/1000.0f);
              velocity.dx += accel.X * (deltaTime.count()/1000.0f);
          });
    }

private:
    entt::registry& world;
};

struct MoveSystem
{
    MoveSystem(entt::registry& world) : world(world)
    {

    }

    void Update(std::chrono::milliseconds  deltaTime)
    {
        auto view = world.view<const Velocity, Position>();
        view.each([deltaTime](const auto entity, const Velocity& velocity, Position& position)
                  {
                      position.z += velocity.dz * (deltaTime.count()/1000.0f);
                  });
    }

private:
    entt::registry& world;
};

struct DebugPrinter
{
    DebugPrinter(entt::registry& world) : world(world)
    {

    }

    void Update(std::chrono::milliseconds deltaTime)
    {
        auto view = world.view<const Velocity, const Position>();
        view.each([](const auto entity, const Velocity& velocity, const Position& position)
                  {
                      fmt::print("Position  Z : {}", position.z);
                      fmt::print("Velocity  Z : {}", velocity.dz);
                  });
    }

private:
    entt::registry& world;
};

int main(int argc, char* argv[]){
    entt::registry registry;

    for(int i =0; i < 10'000; ++i)
    {
        const auto entity = registry.create();
        registry.emplace<Position>(entity, i * 10.0f, i * 10.0f);
        registry.emplace<Velocity>(entity, 0.0f, 0.0f, 0.0f);
        registry.emplace<RigidBody>(entity, i + 1.0f);
    }

    PhysicSystem physic{registry};
    MoveSystem move{registry};
    DebugPrinter printer{registry};

    bool stop = false;
    while(!stop)
    {
        auto deltaTime = 30ms;
        physic.Update(deltaTime);
        move.Update(deltaTime);
        printer.Update(deltaTime);

        std::this_thread::sleep_for(30ms);
    }
}