#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>

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

struct Vehicle {
    std::string id;
    VehicleType type;
    Direction direction;
    TurnType turn;
    double arrival_time;
    double wait_time;
    bool priority;
    
    Vehicle(const std::string& id, VehicleType type, Direction dir, 
            TurnType turn, double arrival_time)
        : id(id), type(type), direction(dir), turn(turn), 
          arrival_time(arrival_time), wait_time(0.0), 
          priority(type == VehicleType::EMERGENCY) {}
};

struct LaneState {
    Direction direction;
    std::vector<TurnType> allowed_turns;
    std::vector<std::shared_ptr<Vehicle>> vehicles;
    
    LaneState(Direction dir, std::vector<TurnType> turns)
        : direction(dir), allowed_turns(turns) {}
    
    size_t queue_length() const { return vehicles.size(); }
};

struct TimingConstraints {
    int min_green;      // Minimum green time (seconds)
    int max_green;      // Maximum green time (seconds)
    int yellow_time;    // Yellow light duration (seconds)
    int pedestrian_time;// Pedestrian crossing time (seconds)
    
    TimingConstraints() 
        : min_green(15), max_green(60), yellow_time(4), pedestrian_time(10) {}
    
    bool validate(SignalPhase phase, int duration) const {
        if (phase == SignalPhase::NORTH_SOUTH_GREEN || 
            phase == SignalPhase::EAST_WEST_GREEN) {
            return duration >= min_green && duration <= max_green;
        }
        else if (phase == SignalPhase::NORTH_SOUTH_YELLOW || 
                 phase == SignalPhase::EAST_WEST_YELLOW) {
            return duration == yellow_time;
        }
        else if (phase == SignalPhase::PEDESTRIAN_CROSSING) {
            return duration >= pedestrian_time;
        }
        return true;
    }
};

struct IntersectionState {
    std::string id;
    SignalPhase current_phase;
    double phase_timer;
    std::unordered_map<Direction, std::unordered_map<std::string, int>> queue_lengths;
    std::unordered_map<Direction, double> wait_times;
    int vehicles_processed;
};

class Intersection {
private:
    std::string id_;
    TimingConstraints constraints_;
    SignalPhase current_phase_;
    double phase_timer_;
    
    // Lanes: 2 lanes per direction [left-turn lane, straight/right lane]
    std::vector<std::vector<LaneState>> lanes_;
    
    // Metrics
    int vehicles_processed_;
    double total_wait_time_;
    
    void initialize_lanes() {
        lanes_.resize(4); // 4 dir
        
        // NORTH (index 0)
        lanes_[0].push_back(LaneState(Direction::NORTH, {TurnType::LEFT}));
        lanes_[0].push_back(LaneState(Direction::NORTH, {TurnType::STRAIGHT, TurnType::RIGHT}));
        
        // SOUTH (index 1)
        lanes_[1].push_back(LaneState(Direction::SOUTH, {TurnType::LEFT}));
        lanes_[1].push_back(LaneState(Direction::SOUTH, {TurnType::STRAIGHT, TurnType::RIGHT}));
        
        // EAST (index 2)
        lanes_[2].push_back(LaneState(Direction::EAST, {TurnType::LEFT}));
        lanes_[2].push_back(LaneState(Direction::EAST, {TurnType::STRAIGHT, TurnType::RIGHT}));
        
        // WEST (index 3)
        lanes_[3].push_back(LaneState(Direction::WEST, {TurnType::LEFT}));
        lanes_[3].push_back(LaneState(Direction::WEST, {TurnType::STRAIGHT, TurnType::RIGHT}));
    }
    
    bool is_green_phase() const {
        return current_phase_ == SignalPhase::NORTH_SOUTH_GREEN ||
               current_phase_ == SignalPhase::EAST_WEST_GREEN;
    }
    
    std::unordered_map<Direction, std::unordered_map<std::string, int>> compute_queue_lengths() const {
        std::unordered_map<Direction, std::unordered_map<std::string, int>> queues;
        
        for (int d = 0; d < 4; ++d) {
            Direction dir = static_cast<Direction>(d);
            queues[dir] = {{"left", 0}, {"straight", 0}, {"right", 0}, {"total", 0}};
            
            // Left lane
            for (const auto& v : lanes_[d][0].vehicles) {
                if (v->turn == TurnType::LEFT) {
                    queues[dir]["left"]++;
                    queues[dir]["total"]++;
                }
            }
            
            // Straight/right lane
            for (const auto& v : lanes_[d][1].vehicles) {
                if (v->turn == TurnType::STRAIGHT) queues[dir]["straight"]++;
                else if (v->turn == TurnType::RIGHT) queues[dir]["right"]++;
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
            
            for (int lane = 0; lane < 2; ++lane) {
                for (const auto& v : lanes_[d][lane].vehicles) {
                    total_wait += v->wait_time;
                    total_vehicles++;
                }
            }
            wait_times[dir] = total_vehicles > 0 ? total_wait / total_vehicles : 0.0;
        }
        return wait_times;
    }
    
    std::unordered_map<Direction, std::unordered_map<TurnType, bool>> get_movement_opportunities() const {
        std::unordered_map<Direction, std::unordered_map<TurnType, bool>> opportunities;
        
        // Init all false
        for (int d = 0; d < 4; ++d) {
            Direction dir = static_cast<Direction>(d);
            opportunities[dir] = {
                {TurnType::STRAIGHT, false},
                {TurnType::LEFT, false},
                {TurnType::RIGHT, false}
            };
        }
        
        // Set based on current phase
        if (current_phase_ == SignalPhase::NORTH_SOUTH_GREEN) {
            opportunities[Direction::NORTH][TurnType::STRAIGHT] = true;
            opportunities[Direction::NORTH][TurnType::RIGHT] = true;
            opportunities[Direction::SOUTH][TurnType::STRAIGHT] = true;
            opportunities[Direction::SOUTH][TurnType::RIGHT] = true;
        }
        else if (current_phase_ == SignalPhase::EAST_WEST_GREEN) {
            opportunities[Direction::EAST][TurnType::STRAIGHT] = true;
            opportunities[Direction::EAST][TurnType::RIGHT] = true;
            opportunities[Direction::WEST][TurnType::STRAIGHT] = true;
            opportunities[Direction::WEST][TurnType::RIGHT] = true;
        }
        
        return opportunities;
    }
    
    void process_vehicles(double delta_time, double current_time) {
        auto opportunities = get_movement_opportunities();
        
        for (int d = 0; d < 4; ++d) {
            for (int lane_idx = 0; lane_idx < 2; ++lane_idx) {
                auto& lane = lanes_[d][lane_idx];
                if (lane.vehicles.empty()) continue;
                
                // FIX: Use a counter to track actual removals
                int processed = 0;
                
                // Check if the front vehicle can move
                while (!lane.vehicles.empty() && processed < (10.0 * delta_time)) {
                    auto& vehicle = lane.vehicles[0]; // Always check the front
                    
                    bool can_move = opportunities[vehicle->direction][vehicle->turn];
                    
                    if (can_move) {
                        // Metrics
                        vehicles_processed_++;
                        total_wait_time_ += (current_time - vehicle->arrival_time);
                        
                        // Remove the front vehicle
                        lane.vehicles.erase(lane.vehicles.begin());
                        processed++;
                    } else {
                        break; // Front vehicle blocked, lane stops
                    }
                }
            }
        }
    }
    
    void update_wait_times(double delta_time) {
        for (int d = 0; d < 4; ++d) {
            for (int lane = 0; lane < 2; ++lane) {
                for (auto& vehicle : lanes_[d][lane].vehicles) {
                    vehicle->wait_time += delta_time;
                }
            }
        }
    }
    
public:
    Intersection(const std::string& id, const TimingConstraints& constraints = TimingConstraints())
        : id_(id), constraints_(constraints), 
          current_phase_(SignalPhase::NORTH_SOUTH_GREEN),
          phase_timer_(15.0), vehicles_processed_(0), total_wait_time_(0.0) {
        initialize_lanes();
    }
    
    // api
    
    /**
     * Set the signal phase and duration
     * phase; Which signal phase to activate
     * duration; How long it should last (seconds)
     * current_time; Current simulation time
     * true if phase change accepted, false if violates constraints
     */
    bool set_phase(SignalPhase phase, double duration, double current_time) {
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
        // rn for simplicity, putting this in update()
        // but could queue this request
    }
    
//simul methods
    /**
     * Update intersection state for one timestep
     * delta_time Time elapsed since last update (seconds)
     * current_time Current simulation time
     */
    void update(double delta_time, double current_time) {
        phase_timer_ -= delta_time;
        

        if (phase_timer_ <= 0) {
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
            // logic to recover from a Pedestrian phase
            else if (current_phase_ == SignalPhase::PEDESTRIAN_CROSSING) {
                // After pedestrians cross, return to North-South Green
                current_phase_ = SignalPhase::NORTH_SOUTH_GREEN;
                phase_timer_ = constraints_.min_green;
                std::cout << "[Intersection] Pedestrian phase finished. Returning to NS_GREEN.\n";
            }
        }
        
        // Process vehicles if green light
        if (phase_timer_ > 0 && is_green_phase()) {
            process_vehicles(delta_time, current_time);
        }
        
        // Update wait times for queued vehicles
        update_wait_times(delta_time);
    }
    
    void add_vehicle(std::shared_ptr<Vehicle> vehicle) {
        int dir_idx = static_cast<int>(vehicle->direction);
        for (auto& lane : lanes_[dir_idx]) {
            for (auto allowed : lane.allowed_turns) {
                if (vehicle->turn == allowed) {
                    lane.vehicles.push_back(vehicle);
                    return;
                }
            }
        }
    }

    int get_vehicles_processed() const { return vehicles_processed_; }
    double get_total_wait_time() const { return total_wait_time_; }
    std::string get_id() const { return id_; }
    const TimingConstraints& get_constraints() const { return constraints_; }  // ADDED

    void print_state() const {
        auto state = get_state();
        std::cout << "Intersection " << id_ << "\n";
        std::cout << "  Phase: ";
        if (state.current_phase == SignalPhase::NORTH_SOUTH_GREEN) std::cout << "NS Green";
        else if (state.current_phase == SignalPhase::EAST_WEST_GREEN) std::cout << "EW Green";
        else if (state.current_phase == SignalPhase::NORTH_SOUTH_YELLOW) std::cout << "NS Yellow";
        else std::cout << "EW Yellow";
        std::cout << " (timer: " << state.phase_timer << "s)\n";
        std::cout << "  Queues - N:" << state.queue_lengths[Direction::NORTH]["total"]
                  << " S:" << state.queue_lengths[Direction::SOUTH]["total"]
                  << " E:" << state.queue_lengths[Direction::EAST]["total"]
                  << " W:" << state.queue_lengths[Direction::WEST]["total"] << "\n";
    }
};

#endif // INTERSECTION_H