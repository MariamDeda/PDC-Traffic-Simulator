#include "DecentralizedController.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

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


// ===================== CONSTRUCTOR =====================

DecentralizedController::DecentralizedController(std::string intersection_id)
    : intersection_id_(std::move(intersection_id)),
      last_preferred_ns_(true),
      fairness_bias_(0.0),
      deadline_enforcer_(std::make_unique<DeadlineEnforcer>(50.0, false)),
      use_deadline_enforcement_(false) {}


// ===================== MESSAGE BUILDING =====================

NeighborMessage DecentralizedController::build_local_message(
    const Intersection& intersection,
    double current_time
) const {
    const IntersectionState state = intersection.get_state();

    NeighborMessage msg;
    msg.sender_id = intersection_id_;
    msg.timestamp = current_time;
    msg.current_phase = state.current_phase;
    msg.predicted_discharge_rate = 10.0;
    msg.prefers_ns = phase_is_ns(state.current_phase);

    for (int d = 0; d < 4; ++d) {
        Direction dir = static_cast<Direction>(d);
        msg.directional_queues[dir] = state.queue_lengths.at(dir).at("total");
        msg.directional_waits[dir] = state.wait_times.at(dir);
    }

    return msg;
}


// ===================== BASIC HELPERS =====================

double DecentralizedController::queue_sum(
    const IntersectionState& state,
    Direction a,
    Direction b
) const {
    return state.queue_lengths.at(a).at("total") +
           state.queue_lengths.at(b).at("total");
}

double DecentralizedController::wait_sum(
    const IntersectionState& state,
    Direction a,
    Direction b
) const {
    return state.wait_times.at(a) + state.wait_times.at(b);
}

bool DecentralizedController::phase_is_ns(SignalPhase phase) const {
    return phase == SignalPhase::NORTH_SOUTH_GREEN ||
           phase == SignalPhase::NORTH_SOUTH_YELLOW;
}


// ===================== NEIGHBOR-BASED HELPERS =====================

double DecentralizedController::upstream_pressure(
    Direction axis_a,
    Direction axis_b,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages
) const {
    double pressure = 0.0;

    for (const auto& entry : neighbor_messages) {
        const NeighborLink& link = entry.first;
        const NeighborMessage& msg = entry.second;

        if (link.relative_position == axis_a) {
            Direction toward_me =
                (axis_a == Direction::NORTH) ? Direction::SOUTH :
                (axis_a == Direction::SOUTH) ? Direction::NORTH :
                (axis_a == Direction::EAST)  ? Direction::WEST  :
                                               Direction::EAST;

            pressure += queue_for(msg, toward_me) + 0.2 * wait_for(msg, toward_me);
        }

        if (link.relative_position == axis_b) {
            Direction toward_me =
                (axis_b == Direction::NORTH) ? Direction::SOUTH :
                (axis_b == Direction::SOUTH) ? Direction::NORTH :
                (axis_b == Direction::EAST)  ? Direction::WEST  :
                                               Direction::EAST;

            pressure += queue_for(msg, toward_me) + 0.2 * wait_for(msg, toward_me);
        }
    }

    return pressure;
}

double DecentralizedController::downstream_blocking(
    Direction axis_a,
    Direction axis_b,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages
) const {
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
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages
) const {
    double bias = 0.0;

    for (const auto& entry : neighbor_messages) {
        const NeighborLink& link = entry.first;
        const NeighborMessage& msg = entry.second;

        bool same_corridor =
            (ns_candidate &&
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


// ===================== DECISION LOGIC =====================

PhaseDecision DecentralizedController::decide(
    const Intersection& intersection,
    const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages,
    double current_time
) {
    (void)current_time;

    if (use_deadline_enforcement_) {
        deadline_enforcer_->startDecision();
    }

    const IntersectionState state = intersection.get_state();
    const TimingConstraints& constraints = intersection.get_constraints();

    int ns_q = queue_sum(state, Direction::NORTH, Direction::SOUTH);
    int ew_q = queue_sum(state, Direction::EAST, Direction::WEST);

    double ns_wait = wait_sum(state, Direction::NORTH, Direction::SOUTH);
    double ew_wait = wait_sum(state, Direction::EAST, Direction::WEST);

    double ns_upstream = upstream_pressure(
        Direction::NORTH,
        Direction::SOUTH,
        neighbor_messages
    );

    double ew_upstream = upstream_pressure(
        Direction::EAST,
        Direction::WEST,
        neighbor_messages
    );

    double ns_downstream_block = downstream_blocking(
        Direction::NORTH,
        Direction::SOUTH,
        neighbor_messages
    );

    double ew_downstream_block = downstream_blocking(
        Direction::EAST,
        Direction::WEST,
        neighbor_messages
    );

    double ns_consensus = consensus_bias(true, neighbor_messages);
    double ew_consensus = consensus_bias(false, neighbor_messages);

    /*
        Score meaning:
        - local queue has strongest effect
        - local wait time matters
        - upstream neighbor pressure encourages corridor flow
        - downstream blocking discourages sending cars into congestion
        - consensus bias lightly encourages coordination with neighbors
    */
    double ns_score =
        static_cast<double>(ns_q) +
        0.5 * ns_wait +
        0.7 * ns_upstream -
        0.4 * ns_downstream_block +
        1.0 * ns_consensus;

    double ew_score =
        static_cast<double>(ew_q) +
        0.5 * ew_wait +
        0.7 * ew_upstream -
        0.4 * ew_downstream_block +
        1.0 * ew_consensus;

    bool currently_ns = phase_is_ns(state.current_phase);
    bool choose_ns = currently_ns;

    const double SWITCH_THRESHOLD = 1.0;

    if (ns_score > ew_score + SWITCH_THRESHOLD) {
        choose_ns = true;
    } else if (ew_score > ns_score + SWITCH_THRESHOLD) {
        choose_ns = false;
    }

    int total_q = ns_q + ew_q;
    double congestion_ratio = std::min(1.0, static_cast<double>(total_q) / 50.0);

    double duration =
        static_cast<double>(constraints.min_green) +
        congestion_ratio *
        static_cast<double>(constraints.max_green - constraints.min_green);

    duration = std::max(
        static_cast<double>(constraints.min_green),
        std::min(duration, static_cast<double>(constraints.max_green))
    );

    PhaseDecision decision;
    decision.phase = choose_ns
        ? SignalPhase::NORTH_SOUTH_GREEN
        : SignalPhase::EAST_WEST_GREEN;

    decision.duration = duration;
    decision.ns_score = ns_score;
    decision.ew_score = ew_score;

    std::ostringstream reason;
    reason << "Decentralized decision | "
           << "NS score=" << ns_score
           << ", EW score=" << ew_score
           << ", local NS q=" << ns_q
           << ", local EW q=" << ew_q;

    decision.reason = reason.str();

    if (use_deadline_enforcement_) {
        if (!deadline_enforcer_->checkDeadline()) {
            deadline_enforcer_->recordFallbackUsed();

            PhaseDecision fallback;

            if (ns_q > ew_q * 1.5) {
                fallback.phase = SignalPhase::NORTH_SOUTH_GREEN;
            } else if (ew_q > ns_q * 1.5) {
                fallback.phase = SignalPhase::EAST_WEST_GREEN;
            } else {
                fallback.phase = decision.phase;
            }

            double fallback_ratio =
                std::min(1.0, static_cast<double>(total_q) / 50.0);

            fallback.duration =
                static_cast<double>(constraints.min_green) +
                fallback_ratio *
                static_cast<double>(constraints.max_green - constraints.min_green);

            fallback.ns_score = ns_score;
            fallback.ew_score = ew_score;

            fallback.reason =
                "FALLBACK used because decision exceeded deadline of " +
                std::to_string(deadline_enforcer_->getDeadlineMs()) +
                " ms";

            return fallback;
        }
    }

    last_preferred_ns_ = choose_ns;

    return decision;
}


// ===================== APPLY DECISION =====================

void DecentralizedController::apply_decision(
    Intersection& intersection,
    const PhaseDecision& decision,
    double current_time
) const {
    IntersectionState state = intersection.get_state();

    /*
        IMPORTANT:
        Do not reset the phase timer every simulation step.

        The previous version called set_phase() every tick.
        That kept restarting the green timer, so yellow phases
        and natural phase cycling could fail.

        This version only applies a new decision when the current
        phase is basically finished.
    */
    if (state.phase_timer > 1.0) {
        return;
    }

    intersection.set_phase(decision.phase, decision.duration, current_time);
}


// ===================== DEADLINE ENFORCEMENT =====================

void DecentralizedController::enableDeadlineEnforcement(
    bool enable,
    double deadline_ms
) {
    use_deadline_enforcement_ = enable;
    deadline_enforcer_->enable(enable);

    if (enable) {
        deadline_enforcer_->setDeadlineMs(deadline_ms);
        deadline_enforcer_->resetStats();

        std::cout << "[Controller " << intersection_id_
                  << "] Deadline enforcement ENABLED ("
                  << deadline_ms << "ms)\n";
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