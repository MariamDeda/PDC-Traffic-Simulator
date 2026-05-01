#include "DecentralizedController.h"

#include <algorithm>
#include <cmath>
#include <sstream>

// for slow computation simulation (temporary)
// #include <thread>
// #include <chrono>

namespace {
int queue_for(const NeighborMessage& msg, Direction d) {
    const auto it = msg.directional_queues.find(d);
    return (it == msg.directional_queues.end()) ? 0 : it->second;
}

double wait_for(const NeighborMessage& msg, Direction d) {
    const auto it = msg.directional_waits.find(d);
    return (it == msg.directional_waits.end()) ? 0.0 : it->second;
}
}  // namespace

DecentralizedController::DecentralizedController(std::string intersection_id)
    : intersection_id_(std::move(intersection_id)), last_preferred_ns_(true), fairness_bias_(0.0),
    use_deadline_enforcement_(false)  // ADDED
    {
        deadline_enforcer_ = std::make_unique<DeadlineEnforcer>(50.0, false);
    }

NeighborMessage DecentralizedController::build_local_message(const Intersection& intersection,
                                                             double current_time) const {
    const IntersectionState state = intersection.get_state();
    NeighborMessage msg;
    msg.sender_id = intersection_id_;
    msg.timestamp = current_time;
    msg.current_phase = state.current_phase;
    msg.predicted_discharge_rate = 10.0;
    msg.prefers_ns = phase_is_ns(state.current_phase);

    for (int d = 0; d < 4; ++d) {
        const Direction dir = static_cast<Direction>(d);
        msg.directional_queues[dir] = state.queue_lengths.at(dir).at("total");
        msg.directional_waits[dir] = state.wait_times.at(dir);
    }
    return msg;
}

double DecentralizedController::queue_sum(const IntersectionState& state,
                                          Direction a,
                                          Direction b) const {
    return state.queue_lengths.at(a).at("total") + state.queue_lengths.at(b).at("total");
}

double DecentralizedController::wait_sum(const IntersectionState& state,
                                         Direction a,
                                         Direction b) const {
    return state.wait_times.at(a) + state.wait_times.at(b);
}

double DecentralizedController::upstream_pressure(
    Direction axis_a,
    Direction axis_b,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages) const {
    double pressure = 0.0;
    for (const auto& entry : neighbor_messages) {
        const NeighborLink& link = entry.first;
        const NeighborMessage& msg = entry.second;

        if (link.relative_position == axis_a) {
            const Direction toward_me = (axis_a == Direction::NORTH) ? Direction::SOUTH
                                      : (axis_a == Direction::SOUTH) ? Direction::NORTH
                                      : (axis_a == Direction::EAST)  ? Direction::WEST
                                                                     : Direction::EAST;
            pressure += queue_for(msg, toward_me) + 0.2 * wait_for(msg, toward_me);
        }
        if (link.relative_position == axis_b) {
            const Direction toward_me = (axis_b == Direction::NORTH) ? Direction::SOUTH
                                      : (axis_b == Direction::SOUTH) ? Direction::NORTH
                                      : (axis_b == Direction::EAST)  ? Direction::WEST
                                                                     : Direction::EAST;
            pressure += queue_for(msg, toward_me) + 0.2 * wait_for(msg, toward_me);
        }
    }
    return pressure;
}

double DecentralizedController::downstream_blocking(
    Direction axis_a,
    Direction axis_b,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages) const {
    double blocking = 0.0;
    for (const auto& entry : neighbor_messages) {
        const NeighborLink& link = entry.first;
        const NeighborMessage& msg = entry.second;

        if (link.relative_position == axis_a) {
            blocking += queue_for(msg, axis_a);
        }
        if (link.relative_position == axis_b) {
            blocking += queue_for(msg, axis_b);
        }
    }
    return blocking;
}

double DecentralizedController::consensus_bias(
    bool ns_candidate,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages) const {
    double bias = 0.0;
    for (const auto& entry : neighbor_messages) {
        const NeighborLink& link = entry.first;
        const NeighborMessage& msg = entry.second;
        const bool same_corridor = (ns_candidate &&
                                    (link.relative_position == Direction::NORTH ||
                                     link.relative_position == Direction::SOUTH)) ||
                                   (!ns_candidate &&
                                    (link.relative_position == Direction::EAST ||
                                     link.relative_position == Direction::WEST));
        if (!same_corridor) {
            continue;
        }
        bias += (msg.prefers_ns == ns_candidate) ? 1.0 : -0.7;
    }
    return bias;
}

bool DecentralizedController::phase_is_ns(SignalPhase phase) const {
    return phase == SignalPhase::NORTH_SOUTH_GREEN || phase == SignalPhase::NORTH_SOUTH_YELLOW;
}

PhaseDecision DecentralizedController::decide(
    const Intersection& intersection,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages,
    double current_time) {
    
    // If deadline enforcement is enabled, measure time
    if (use_deadline_enforcement_) {
        deadline_enforcer_->startDecision();

        // Simulate heavy computation (100ms - will miss 50ms deadline)
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // ===== YOUR ORIGINAL DECISION LOGIC GOES HERE =====
    // (Copy your existing decision code exactly as it was)
    
    const IntersectionState state = intersection.get_state();
    const TimingConstraints& constraints = intersection.get_constraints();
    
    // Get queue lengths using .at() (const-safe)
    int ns_q = state.queue_lengths.at(Direction::NORTH).at("total") + 
               state.queue_lengths.at(Direction::SOUTH).at("total");
    int ew_q = state.queue_lengths.at(Direction::EAST).at("total") + 
               state.queue_lengths.at(Direction::WEST).at("total");
    
    // Your existing scoring logic...
    double ns_score = ns_q;  // Replace with your actual scoring
    double ew_score = ew_q;  // Replace with your actual scoring
    
    bool currently_ns = (state.current_phase == SignalPhase::NORTH_SOUTH_GREEN ||
                         state.current_phase == SignalPhase::NORTH_SOUTH_YELLOW);
    
    bool choose_ns = currently_ns;
    if (ns_score > ew_score) {
        choose_ns = true;
    } else if (ew_score > ns_score) {
        choose_ns = false;
    }
    
    double duration = constraints.min_green + (ns_q + ew_q) / 100.0 * 
                      (constraints.max_green - constraints.min_green);
    duration = std::min(std::max(duration, (double)constraints.min_green), 
                        (double)constraints.max_green);
    
    PhaseDecision decision;
    decision.phase = choose_ns ? SignalPhase::NORTH_SOUTH_GREEN : SignalPhase::EAST_WEST_GREEN;
    decision.duration = duration;
    decision.ns_score = ns_score;
    decision.ew_score = ew_score;
    decision.reason = "Normal decision";
    
    // ===== CHECK DEADLINE =====
    if (use_deadline_enforcement_) {
        if (!deadline_enforcer_->checkDeadline()) {
            // Deadline missed - use simple fallback
            deadline_enforcer_->recordFallbackUsed();
            
            // Simple fallback: give green to direction with more cars
            PhaseDecision fallback;
            if (ns_q > ew_q * 1.5) {
                fallback.phase = SignalPhase::NORTH_SOUTH_GREEN;
            } else if (ew_q > ns_q * 1.5) {
                fallback.phase = SignalPhase::EAST_WEST_GREEN;
            } else {
                fallback.phase = decision.phase;  // Stay as is
            }
            
            double total_q = ns_q + ew_q;
            double ratio = std::min(1.0, total_q / 50.0);
            fallback.duration = constraints.min_green + ratio * (constraints.max_green - constraints.min_green);
            fallback.reason = "FALLBACK (deadline missed: " + 
                              std::to_string(deadline_enforcer_->getDeadlineMs()) + "ms)";
            fallback.ns_score = ns_score;
            fallback.ew_score = ew_score;
            
            return fallback;
        }
    }
    
    return decision;
}

void DecentralizedController::apply_decision(Intersection& intersection,
                                             const PhaseDecision& decision,
                                             double current_time) const {
    intersection.set_phase(decision.phase, decision.duration, current_time);
}

// Real-time deadline enforcement
void DecentralizedController::enableDeadlineEnforcement(bool enable, double deadline_ms) {
    use_deadline_enforcement_ = enable;
    deadline_enforcer_->enable(enable);
    if (enable) {
        deadline_enforcer_->setDeadlineMs(deadline_ms);
        deadline_enforcer_->resetStats();
        std::cout << "[Controller " << intersection_id_ 
                  << "] Deadline enforcement ENABLED (" << deadline_ms << "ms)\n";
    } else {
        std::cout << "[Controller " << intersection_id_ 
                  << "] Deadline enforcement DISABLED\n";
    }
}

const DeadlineStats& DecentralizedController::getDeadlineStats() const {
    return deadline_enforcer_->getStats();
}

void DecentralizedController::printDeadlineStats() const {
    if (use_deadline_enforcement_) {
        deadline_enforcer_->printStats();
    } else {
        std::cout << "  Deadline enforcement not enabled\n";
    }
}

void DecentralizedController::resetDeadlineStats() {
    deadline_enforcer_->resetStats();
}