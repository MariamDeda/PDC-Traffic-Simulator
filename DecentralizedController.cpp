#include "DecentralizedController.h"

#include <algorithm>
#include <cmath>
#include <sstream>

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
    : intersection_id_(std::move(intersection_id)), last_preferred_ns_(true), fairness_bias_(0.0) {}

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
    const IntersectionState state = intersection.get_state();
    const TimingConstraints& constraints = intersection.get_constraints();

    const double ns_local_q = queue_sum(state, Direction::NORTH, Direction::SOUTH);
    const double ew_local_q = queue_sum(state, Direction::EAST, Direction::WEST);
    const double ns_local_wait = wait_sum(state, Direction::NORTH, Direction::SOUTH);
    const double ew_local_wait = wait_sum(state, Direction::EAST, Direction::WEST);

    const double ns_upstream = upstream_pressure(Direction::NORTH, Direction::SOUTH, neighbor_messages);
    const double ew_upstream = upstream_pressure(Direction::EAST, Direction::WEST, neighbor_messages);
    const double ns_blocking = downstream_blocking(Direction::NORTH, Direction::SOUTH, neighbor_messages);
    const double ew_blocking = downstream_blocking(Direction::EAST, Direction::WEST, neighbor_messages);

    // Utility = local pressure relief + wait reduction + corridor consensus - downstream blocking.
    double ns_score = 1.20 * ns_local_q + 0.65 * ns_local_wait + 0.60 * ns_upstream - 0.35 * ns_blocking;
    double ew_score = 1.20 * ew_local_q + 0.65 * ew_local_wait + 0.60 * ew_upstream - 0.35 * ew_blocking;

    ns_score += 1.50 * consensus_bias(true, neighbor_messages);
    ew_score += 1.50 * consensus_bias(false, neighbor_messages);

    // Conflict resolution between competing directions: hysteresis + anti-starvation.
    const bool currently_ns = phase_is_ns(state.current_phase);
    const double switch_margin = 3.5;
    if (currently_ns) {
        ns_score += 2.0;  // discourage rapid oscillation
    } else {
        ew_score += 2.0;
    }

    const double ns_delay = ns_local_wait + fairness_bias_;
    const double ew_delay = ew_local_wait - fairness_bias_;
    if (ns_delay > ew_delay + 8.0) {
        ns_score += 4.0;
    } else if (ew_delay > ns_delay + 8.0) {
        ew_score += 4.0;
    }

    bool choose_ns = currently_ns;
    if (ns_score > ew_score + switch_margin) {
        choose_ns = true;
    } else if (ew_score > ns_score + switch_margin) {
        choose_ns = false;
    }

    // Optimization: choose green duration from utility and corridor pressure.
    const double chosen_load = choose_ns ? (ns_local_q + ns_upstream) : (ew_local_q + ew_upstream);
    const double total_load = ns_local_q + ew_local_q + ns_upstream + ew_upstream + 1.0;
    const double load_ratio = std::clamp(chosen_load / total_load, 0.0, 1.0);
    double duration = constraints.min_green + load_ratio * (constraints.max_green - constraints.min_green);

    const double alignment_bonus = std::max(0.0, consensus_bias(choose_ns, neighbor_messages));
    duration += 2.0 * alignment_bonus;
    duration = std::clamp(duration,
                          static_cast<double>(constraints.min_green),
                          static_cast<double>(constraints.max_green));

    // Update fairness memory so one axis cannot dominate forever.
    fairness_bias_ += choose_ns ? -0.5 : 0.5;
    fairness_bias_ = std::clamp(fairness_bias_, -10.0, 10.0);
    last_preferred_ns_ = choose_ns;

    std::ostringstream reason;
    reason << "local_q(NS=" << ns_local_q << ", EW=" << ew_local_q << ")"
           << ", upstream(NS=" << ns_upstream << ", EW=" << ew_upstream << ")"
           << ", chosen=" << (choose_ns ? "NS" : "EW")
           << ", t=" << current_time;

    PhaseDecision decision;
    decision.phase = choose_ns ? SignalPhase::NORTH_SOUTH_GREEN : SignalPhase::EAST_WEST_GREEN;
    decision.duration = duration;
    decision.ns_score = ns_score;
    decision.ew_score = ew_score;
    decision.reason = reason.str();
    return decision;
}

void DecentralizedController::apply_decision(Intersection& intersection,
                                             const PhaseDecision& decision,
                                             double current_time) const {
    intersection.set_phase(decision.phase, decision.duration, current_time);
}
