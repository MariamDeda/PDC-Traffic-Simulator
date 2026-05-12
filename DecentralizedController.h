#ifndef DECENTRALIZED_CONTROLLER_H
#define DECENTRALIZED_CONTROLLER_H

#include "Intersection.h"
#include "RealTimeDeadline.h"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <memory>

struct NeighborMessage {
    std::string sender_id;
    double timestamp;
    SignalPhase current_phase;
    std::unordered_map<Direction, int> directional_queues;
    std::unordered_map<Direction, double> directional_waits;
    double predicted_discharge_rate;
    bool prefers_ns;
};

struct NeighborLink {
    std::string neighbor_id;
    Direction relative_position;
};

struct PhaseDecision {
    SignalPhase phase;
    double duration;
    double ns_score;
    double ew_score;
    std::string reason;
};

class DecentralizedController {
public:
    explicit DecentralizedController(std::string intersection_id);

    NeighborMessage build_local_message(const Intersection& intersection, double current_time) const;

    PhaseDecision decide(const Intersection& intersection,
                         const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages,
                         double current_time);

    void apply_decision(Intersection& intersection,
                        const PhaseDecision& decision,
                        double current_time) const;
    
    void enableDeadlineEnforcement(bool enable, double deadline_ms = 50.0);
    const DeadlineStats& getDeadlineStats() const;
    void printDeadlineStats() const;
    void resetDeadlineStats();


private:
    std::string intersection_id_;
    bool last_preferred_ns_;
    double fairness_bias_;

    double queue_sum(const IntersectionState& state, Direction a, Direction b) const;
    double wait_sum(const IntersectionState& state, Direction a, Direction b) const;
    double upstream_pressure(Direction axis_a,
                             Direction axis_b,
                             const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages) const;
    double downstream_blocking(Direction axis_a,
                               Direction axis_b,
                               const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages) const;
    double consensus_bias(bool ns_candidate,
                          const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages) const;
    bool phase_is_ns(SignalPhase phase) const;

    // Deadline enforcement
    std::unique_ptr<DeadlineEnforcer> deadline_enforcer_;
    bool use_deadline_enforcement_;

    PhaseDecision computeDecision(
        const Intersection& intersection,
        const std::vector<std::pair<NeighborLink, NeighborMessage>>& neighbor_messages,
        double current_time);
};

#endif
