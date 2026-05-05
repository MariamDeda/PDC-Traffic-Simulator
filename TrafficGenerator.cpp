#include "TrafficGenerator.h"

#include <string>

// ===================== CONSTRUCTOR =====================

TrafficGenerator::TrafficGenerator(ScenarioType scenario)
    : scenario_(scenario),
      rng_(std::random_device{}()),
      prob_dist_(0.0, 1.0) {}


// ===================== SPAWN RATE =====================

double TrafficGenerator::get_spawn_rate(double time) {
    (void)time;

    switch (scenario_) {
        case ScenarioType::NORMAL:
            return 0.3;

        case ScenarioType::RUSH_HOUR_NS:
            return 0.7;

        case ScenarioType::RUSH_HOUR_EW:
            return 0.7;

        case ScenarioType::PEAK_SWITCH:
            return 0.6;

        case ScenarioType::ACCIDENT:
            return 0.5;

        case ScenarioType::LOW_TRAFFIC:
            return 0.1;

        default:
            return 0.3;
    }
}


// ===================== DIRECTION SELECTION =====================

Direction TrafficGenerator::get_direction(double time) {
    double r = prob_dist_(rng_);

    switch (scenario_) {
        case ScenarioType::RUSH_HOUR_NS:
            if (r < 0.4) return Direction::NORTH;
            if (r < 0.8) return Direction::SOUTH;
            if (r < 0.9) return Direction::EAST;
            return Direction::WEST;

        case ScenarioType::RUSH_HOUR_EW:
            if (r < 0.4) return Direction::EAST;
            if (r < 0.8) return Direction::WEST;
            if (r < 0.9) return Direction::NORTH;
            return Direction::SOUTH;

        case ScenarioType::PEAK_SWITCH:
            if (time < 100.0) {
                return (r < 0.5) ? Direction::NORTH : Direction::SOUTH;
            }
            return (r < 0.5) ? Direction::EAST : Direction::WEST;

        case ScenarioType::ACCIDENT:
            if (r < 0.3) return Direction::NORTH;
            if (r < 0.6) return Direction::SOUTH;
            if (r < 0.8) return Direction::WEST;
            return Direction::EAST;

        case ScenarioType::LOW_TRAFFIC:
        case ScenarioType::NORMAL:
        default: {
            std::uniform_int_distribution<int> dir_dist(0, 3);
            return static_cast<Direction>(dir_dist(rng_));
        }
    }
}


// ===================== TURN SELECTION =====================

TurnType TrafficGenerator::get_turn() {
    double r = prob_dist_(rng_);

    if (r < 0.2) {
        return TurnType::LEFT;
    }

    if (r < 0.7) {
        return TurnType::STRAIGHT;
    }

    return TurnType::RIGHT;
}


// ===================== VEHICLE CREATION =====================

std::shared_ptr<Vehicle> TrafficGenerator::create_vehicle(double current_time) {
    static int vehicle_counter = 0;

    Direction dir = get_direction(current_time);
    TurnType turn = get_turn();
    VehicleType type = VehicleType::CAR;

    std::string id = "v" + std::to_string(vehicle_counter++);

    return std::make_shared<Vehicle>(
        id,
        type,
        dir,
        turn,
        current_time
    );
}


// ===================== UPDATE =====================

void TrafficGenerator::update(Intersection& intersection, double current_time) {
    double spawn_rate = get_spawn_rate(current_time);

    if (prob_dist_(rng_) < spawn_rate) {
        auto vehicle = create_vehicle(current_time);
        intersection.add_vehicle(vehicle);
    }

    // Placeholder only. The current Intersection::request_pedestrian_phase()
    // does not queue or trigger an actual pedestrian phase yet.
    if (scenario_ == ScenarioType::NORMAL && prob_dist_(rng_) < 0.05) {
        intersection.request_pedestrian_phase();
    }
}