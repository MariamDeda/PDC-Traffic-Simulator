#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>

// ===================== ENUMS =====================

enum class SignalPhase {
    NORTH_SOUTH_GREEN,
    NORTH_SOUTH_YELLOW,
    EAST_WEST_GREEN,
    EAST_WEST_YELLOW,
    PEDESTRIAN_CROSSING
};

enum class Direction {
    NORTH = 0,
    SOUTH = 1,
    EAST = 2,
    WEST = 3
};

enum class TurnType {
    STRAIGHT,
    LEFT,
    RIGHT
};

enum class VehicleType {
    CAR,
    TRUCK,
    EMERGENCY
};

// ===================== VEHICLE =====================

struct Vehicle {
    std::string id;
    VehicleType type;
    Direction direction;
    TurnType turn;
    double arrival_time;
    double wait_time;
    bool priority;

    Vehicle(const std::string& id,
            VehicleType type,
            Direction dir,
            TurnType turn,
            double arrival_time)
        : id(id),
          type(type),
          direction(dir),
          turn(turn),
          arrival_time(arrival_time),
          wait_time(0.0),
          priority(type == VehicleType::EMERGENCY) {}
};

// ===================== LANE STATE =====================

struct LaneState {
    Direction direction;
    std::vector<TurnType> allowed_turns;
    std::vector<std::shared_ptr<Vehicle>> vehicles;

    LaneState(Direction dir, std::vector<TurnType> turns)
        : direction(dir), allowed_turns(std::move(turns)) {}

    size_t queue_length() const {
        return vehicles.size();
    }
};

// ===================== TIMING CONSTRAINTS =====================

struct TimingConstraints {
    int min_green;
    int max_green;
    int yellow_time;
    int pedestrian_time;

    TimingConstraints()
        : min_green(15),
          max_green(60),
          yellow_time(4),
          pedestrian_time(10) {}

    bool validate(SignalPhase phase, int duration) const {
        if (phase == SignalPhase::NORTH_SOUTH_GREEN ||
            phase == SignalPhase::EAST_WEST_GREEN) {
            return duration >= min_green && duration <= max_green;
        }

        if (phase == SignalPhase::NORTH_SOUTH_YELLOW ||
            phase == SignalPhase::EAST_WEST_YELLOW) {
            return duration == yellow_time;
        }

        if (phase == SignalPhase::PEDESTRIAN_CROSSING) {
            return duration >= pedestrian_time;
        }

        return true;
    }
};

// ===================== INTERSECTION STATE =====================

struct IntersectionState {
    std::string id;
    SignalPhase current_phase;
    double phase_timer;
    std::unordered_map<Direction, std::unordered_map<std::string, int>> queue_lengths;
    std::unordered_map<Direction, double> wait_times;
    int vehicles_processed;
};

// ===================== INTERSECTION =====================

class Intersection {
private:
    std::string id_;
    TimingConstraints constraints_;
    SignalPhase current_phase_;
    double phase_timer_;

    // 4 directions, each with 2 lanes:
    // lane 0 = left-turn lane
    // lane 1 = straight/right lane
    std::vector<std::vector<LaneState>> lanes_;

    int vehicles_processed_;
    double total_wait_time_;

private:
    void initialize_lanes() {
        lanes_.resize(4);

        lanes_[static_cast<int>(Direction::NORTH)].push_back(
            LaneState(Direction::NORTH, {TurnType::LEFT})
        );
        lanes_[static_cast<int>(Direction::NORTH)].push_back(
            LaneState(Direction::NORTH, {TurnType::STRAIGHT, TurnType::RIGHT})
        );

        lanes_[static_cast<int>(Direction::SOUTH)].push_back(
            LaneState(Direction::SOUTH, {TurnType::LEFT})
        );
        lanes_[static_cast<int>(Direction::SOUTH)].push_back(
            LaneState(Direction::SOUTH, {TurnType::STRAIGHT, TurnType::RIGHT})
        );

        lanes_[static_cast<int>(Direction::EAST)].push_back(
            LaneState(Direction::EAST, {TurnType::LEFT})
        );
        lanes_[static_cast<int>(Direction::EAST)].push_back(
            LaneState(Direction::EAST, {TurnType::STRAIGHT, TurnType::RIGHT})
        );

        lanes_[static_cast<int>(Direction::WEST)].push_back(
            LaneState(Direction::WEST, {TurnType::LEFT})
        );
        lanes_[static_cast<int>(Direction::WEST)].push_back(
            LaneState(Direction::WEST, {TurnType::STRAIGHT, TurnType::RIGHT})
        );
    }

    bool is_green_phase() const {
        return current_phase_ == SignalPhase::NORTH_SOUTH_GREEN ||
               current_phase_ == SignalPhase::EAST_WEST_GREEN;
    }

    std::unordered_map<Direction, std::unordered_map<std::string, int>>
    compute_queue_lengths() const {
        std::unordered_map<Direction, std::unordered_map<std::string, int>> queues;

        for (int d = 0; d < 4; ++d) {
            Direction dir = static_cast<Direction>(d);

            queues[dir] = {
                {"left", 0},
                {"straight", 0},
                {"right", 0},
                {"total", 0}
            };

            for (const auto& vehicle : lanes_[d][0].vehicles) {
                if (vehicle->turn == TurnType::LEFT) {
                    queues[dir]["left"]++;
                    queues[dir]["total"]++;
                }
            }

            for (const auto& vehicle : lanes_[d][1].vehicles) {
                if (vehicle->turn == TurnType::STRAIGHT) {
                    queues[dir]["straight"]++;
                } else if (vehicle->turn == TurnType::RIGHT) {
                    queues[dir]["right"]++;
                }

                queues[dir]["total"]++;
            }
        }

        return queues;
    }

    std::unordered_map<Direction, double> compute_wait_times() const {
        std::unordered_map<Direction, double> wait_times;

        for (int d = 0; d < 4; ++d) {
            Direction dir = static_cast<Direction>(d);

            double total_wait = 0.0;
            size_t total_vehicles = 0;

            for (int lane_idx = 0; lane_idx < 2; ++lane_idx) {
                for (const auto& vehicle : lanes_[d][lane_idx].vehicles) {
                    total_wait += vehicle->wait_time;
                    total_vehicles++;
                }
            }

            wait_times[dir] =
                total_vehicles > 0
                    ? total_wait / static_cast<double>(total_vehicles)
                    : 0.0;
        }

        return wait_times;
    }

    std::unordered_map<Direction, std::unordered_map<TurnType, bool>>
    get_movement_opportunities() const {
        std::unordered_map<Direction, std::unordered_map<TurnType, bool>> opportunities;

        for (int d = 0; d < 4; ++d) {
            Direction dir = static_cast<Direction>(d);

            opportunities[dir] = {
                {TurnType::STRAIGHT, false},
                {TurnType::LEFT, false},
                {TurnType::RIGHT, false}
            };
        }

        if (current_phase_ == SignalPhase::NORTH_SOUTH_GREEN) {
            opportunities[Direction::NORTH][TurnType::STRAIGHT] = true;
            opportunities[Direction::NORTH][TurnType::LEFT] = true;
            opportunities[Direction::NORTH][TurnType::RIGHT] = true;

            opportunities[Direction::SOUTH][TurnType::STRAIGHT] = true;
            opportunities[Direction::SOUTH][TurnType::LEFT] = true;
            opportunities[Direction::SOUTH][TurnType::RIGHT] = true;
        }

        else if (current_phase_ == SignalPhase::EAST_WEST_GREEN) {
            opportunities[Direction::EAST][TurnType::STRAIGHT] = true;
            opportunities[Direction::EAST][TurnType::LEFT] = true;
            opportunities[Direction::EAST][TurnType::RIGHT] = true;

            opportunities[Direction::WEST][TurnType::STRAIGHT] = true;
            opportunities[Direction::WEST][TurnType::LEFT] = true;
            opportunities[Direction::WEST][TurnType::RIGHT] = true;
        }

        return opportunities;
    }

    void process_vehicles(double delta_time, double current_time) {
        auto opportunities = get_movement_opportunities();

        for (int d = 0; d < 4; ++d) {
            for (int lane_idx = 0; lane_idx < 2; ++lane_idx) {
                auto& lane = lanes_[d][lane_idx];

                if (lane.vehicles.empty()) {
                    continue;
                }

                int processed = 0;
                int max_process_this_tick = static_cast<int>(10.0 * delta_time);

                while (!lane.vehicles.empty() && processed < max_process_this_tick) {
                    auto vehicle = lane.vehicles.front();

                    bool can_move = opportunities[vehicle->direction][vehicle->turn];

                    if (!can_move) {
                        break;
                    }

                    vehicles_processed_++;
                    total_wait_time_ += current_time - vehicle->arrival_time;

                    lane.vehicles.erase(lane.vehicles.begin());
                    processed++;
                }
            }
        }
    }

    void update_wait_times(double delta_time) {
        for (int d = 0; d < 4; ++d) {
            for (int lane_idx = 0; lane_idx < 2; ++lane_idx) {
                for (auto& vehicle : lanes_[d][lane_idx].vehicles) {
                    vehicle->wait_time += delta_time;
                }
            }
        }
    }

    static std::string phase_to_string(SignalPhase phase) {
        switch (phase) {
            case SignalPhase::NORTH_SOUTH_GREEN:
                return "NS Green";
            case SignalPhase::NORTH_SOUTH_YELLOW:
                return "NS Yellow";
            case SignalPhase::EAST_WEST_GREEN:
                return "EW Green";
            case SignalPhase::EAST_WEST_YELLOW:
                return "EW Yellow";
            case SignalPhase::PEDESTRIAN_CROSSING:
                return "Pedestrian Crossing";
            default:
                return "Unknown";
        }
    }

public:
    Intersection(const std::string& id,
                 const TimingConstraints& constraints = TimingConstraints())
        : id_(id),
          constraints_(constraints),
          current_phase_(SignalPhase::NORTH_SOUTH_GREEN),
          phase_timer_(static_cast<double>(constraints.min_green)),
          vehicles_processed_(0),
          total_wait_time_(0.0) {
        initialize_lanes();
    }

    bool set_phase(SignalPhase phase, double duration, double current_time) {
        (void)current_time;

        if (!constraints_.validate(phase, static_cast<int>(duration))) {
            return false;
        }

        current_phase_ = phase;
        phase_timer_ = duration;
        return true;
    }

    IntersectionState get_state() const {
        IntersectionState state;

        state.id = id_;
        state.current_phase = current_phase_;
        state.phase_timer = phase_timer_;
        state.queue_lengths = compute_queue_lengths();
        state.wait_times = compute_wait_times();
        state.vehicles_processed = vehicles_processed_;

        return state;
    }

    void request_pedestrian_phase() {
        // Placeholder for future pedestrian request queueing.
        // Current simulation manually calls set_phase(PEDESTRIAN_CROSSING, ...).
    }

    void update(double delta_time, double current_time) {
        phase_timer_ -= delta_time;

        if (phase_timer_ <= 0.0) {
            if (current_phase_ == SignalPhase::NORTH_SOUTH_GREEN) {
                current_phase_ = SignalPhase::NORTH_SOUTH_YELLOW;
                phase_timer_ = constraints_.yellow_time;
            }

            else if (current_phase_ == SignalPhase::NORTH_SOUTH_YELLOW) {
                current_phase_ = SignalPhase::EAST_WEST_GREEN;
                phase_timer_ = constraints_.min_green;
            }

            else if (current_phase_ == SignalPhase::EAST_WEST_GREEN) {
                current_phase_ = SignalPhase::EAST_WEST_YELLOW;
                phase_timer_ = constraints_.yellow_time;
            }

            else if (current_phase_ == SignalPhase::EAST_WEST_YELLOW) {
                current_phase_ = SignalPhase::NORTH_SOUTH_GREEN;
                phase_timer_ = constraints_.min_green;
            }

            else if (current_phase_ == SignalPhase::PEDESTRIAN_CROSSING) {
                current_phase_ = SignalPhase::NORTH_SOUTH_GREEN;
                phase_timer_ = constraints_.min_green;

                std::cout << "[Intersection " << id_
                          << "] Pedestrian phase finished. Returning to NS Green.\n";
            }
        }

        if (phase_timer_ > 0.0 && is_green_phase()) {
            process_vehicles(delta_time, current_time);
        }

        update_wait_times(delta_time);
    }

    void add_vehicle(std::shared_ptr<Vehicle> vehicle) {
        if (!vehicle) {
            return;
        }

        int dir_idx = static_cast<int>(vehicle->direction);

        if (dir_idx < 0 || dir_idx >= static_cast<int>(lanes_.size())) {
            return;
        }

        for (auto& lane : lanes_[dir_idx]) {
            for (TurnType allowed_turn : lane.allowed_turns) {
                if (vehicle->turn == allowed_turn) {
                    lane.vehicles.push_back(vehicle);
                    return;
                }
            }
        }
    }

    int get_vehicles_processed() const {
        return vehicles_processed_;
    }

    double get_total_wait_time() const {
        return total_wait_time_;
    }

    std::string get_id() const {
        return id_;
    }

    const TimingConstraints& get_constraints() const {
        return constraints_;
    }

    void print_state() const {
        auto state = get_state();

        std::cout << "Intersection " << id_ << "\n";
        std::cout << "  Phase: " << phase_to_string(state.current_phase)
                  << " (timer: " << state.phase_timer << "s)\n";

        std::cout << "  Queues - "
                  << "N:" << state.queue_lengths[Direction::NORTH]["total"]
                  << " S:" << state.queue_lengths[Direction::SOUTH]["total"]
                  << " E:" << state.queue_lengths[Direction::EAST]["total"]
                  << " W:" << state.queue_lengths[Direction::WEST]["total"]
                  << "\n";

        std::cout << "  Vehicles processed: " << vehicles_processed_ << "\n";
        std::cout << "  Total wait time: " << total_wait_time_ << "s\n";
    }
};

#endif // INTERSECTION_H