#include "TrafficGenerator.h"

TrafficGenerator::TrafficGenerator(ScenarioType scenario)
    : scenario_(scenario), rng_(std::random_device{}()), prob_dist_(0.0, 1.0) {}


// spawns
double TrafficGenerator::get_spawn_rate(double time) {
    switch (scenario_) {
        case ScenarioType::NORMAL: return 0.3;
        case ScenarioType::RUSH_HOUR_NS: return 0.7;
        case ScenarioType::RUSH_HOUR_EW: return 0.7;
        case ScenarioType::LOW_TRAFFIC: return 0.1;

        case ScenarioType::PEAK_SWITCH:
            return 0.6;

        case ScenarioType::ACCIDENT:
            return 0.5;

        default: return 0.3;
    }
}


//directions
Direction TrafficGenerator::get_direction(double time) {

    double r = prob_dist_(rng_);

    switch (scenario_) {

        case ScenarioType::RUSH_HOUR_NS:
            if (r < 0.4) return Direction::NORTH;
            else if (r < 0.8) return Direction::SOUTH;
            else if (r < 0.9) return Direction::EAST;
            else return Direction::WEST;

        case ScenarioType::RUSH_HOUR_EW:
            if (r < 0.4) return Direction::EAST;
            else if (r < 0.8) return Direction::WEST;
            else if (r < 0.9) return Direction::NORTH;
            else return Direction::SOUTH;

        case ScenarioType::PEAK_SWITCH:
            if (time < 100) {
                return (r < 0.5) ? Direction::NORTH : Direction::SOUTH;
            } else {
                return (r < 0.5) ? Direction::EAST : Direction::WEST;
            }

        case ScenarioType::ACCIDENT:
            // Reduce EAST traffic (simulate blockage)
            if (r < 0.3) return Direction::NORTH;
            else if (r < 0.6) return Direction::SOUTH;
            else if (r < 0.8) return Direction::WEST;
            else return Direction::EAST;

        case ScenarioType::LOW_TRAFFIC:
        case ScenarioType::NORMAL:
        default:
            return static_cast<Direction>(rand() % 4);
    }
}


// ===== Turn type =====
TurnType TrafficGenerator::get_turn() {
    double r = prob_dist_(rng_);

    if (r < 0.2) return TurnType::LEFT;
    else if (r < 0.7) return TurnType::STRAIGHT;
    else return TurnType::RIGHT;
}


// ===== Create vehicle =====
std::shared_ptr<Vehicle> TrafficGenerator::create_vehicle(double current_time) {

    Direction dir = get_direction(current_time);
    TurnType turn = get_turn();

    VehicleType type = VehicleType::CAR;

    return std::make_shared<Vehicle>(
        "v" + std::to_string(rand()),
        type,
        dir,
        turn,
        current_time
    );
}


// ===== MAIN UPDATE FUNCTION =====
void TrafficGenerator::update(Intersection& intersection, double current_time) {

    double spawn_rate = get_spawn_rate(current_time);

    if (prob_dist_(rng_) < spawn_rate) {
        auto vehicle = create_vehicle(current_time);
        intersection.add_vehicle(vehicle);
    }

    // Optional: trigger pedestrians randomly
    if (scenario_ == ScenarioType::NORMAL && prob_dist_(rng_) < 0.05) {
        intersection.request_pedestrian_phase();
    }
}