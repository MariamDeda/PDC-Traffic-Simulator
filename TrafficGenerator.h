#ifndef TRAFFIC_GENERATOR_H
#define TRAFFIC_GENERATOR_H

#include "Intersection.h"
#include <random>

enum class ScenarioType {
    NORMAL,
    RUSH_HOUR_NS,
    RUSH_HOUR_EW,
    PEAK_SWITCH,
    ACCIDENT,
    LOW_TRAFFIC
};

class TrafficGenerator {
public:
    TrafficGenerator(ScenarioType scenario);

    void update(Intersection& intersection, double current_time);

private:
    ScenarioType scenario_;

    std::mt19937 rng_;
    std::uniform_real_distribution<double> prob_dist_;

    double get_spawn_rate(double time);
    Direction get_direction(double time);
    TurnType get_turn();

    std::shared_ptr<Vehicle> create_vehicle(double current_time);
};

#endif