#include "Intersection.h"
#include "TrafficGenerator.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

namespace {
std::string phaseLabel(SignalPhase phase) {
    switch (phase) {
        case SignalPhase::PEDESTRIAN_CROSSING: return "[WALK] ";
        case SignalPhase::NORTH_SOUTH_GREEN:  return "NS-GRN ";
        case SignalPhase::EAST_WEST_GREEN:    return "EW-GRN ";
        case SignalPhase::NORTH_SOUTH_YELLOW: return "NS-YEL ";
        case SignalPhase::EAST_WEST_YELLOW:   return "EW-YEL ";
        default:                              return "UNKNOWN";
    }
}
}

int main() {
    const double DELTA_TIME = 1.0;
    const int TOTAL_STEPS = 300;
    double current_time = 0.0;

    // Pedestrian control variables
    bool ped_pending = false;
    double last_ped_end_time = -100.0;
    const double PED_COOLDOWN = 30.0;

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> percent_dist(0, 99);

    TrafficGenerator myGenerator(ScenarioType::RUSH_HOUR_NS);

    TimingConstraints config;
    config.min_green = 15;
    config.max_green = 60;
    config.yellow_time = 3;
    config.pedestrian_time = 10;

    Intersection myIntersection("IBA_Junction", config);

    int max_queue_observed = 0;

    std::cout << "Starting Simulation: " << myIntersection.get_id() << "\n";
    std::cout << "Scenario: RUSH_HOUR_NS | Duration: " << TOTAL_STEPS << "s\n\n";

    for (int step = 0; step < TOTAL_STEPS; ++step) {
        myGenerator.update(myIntersection, current_time);

        // Random pedestrian request, with cooldown.
        if (!ped_pending && (current_time - last_ped_end_time > PED_COOLDOWN)) {
            if (percent_dist(rng) < 5) {
                ped_pending = true;
            }
        }

        if (ped_pending) {
            bool accepted = myIntersection.set_phase(
                SignalPhase::PEDESTRIAN_CROSSING,
                config.pedestrian_time,
                current_time
            );

            if (accepted) {
                ped_pending = false;
                last_ped_end_time = current_time + config.pedestrian_time;
            }
        }

        myIntersection.update(DELTA_TIME, current_time);

        IntersectionState s = myIntersection.get_state();

        int current_max = 0;
        for (int i = 0; i < 4; ++i) {
            int q = s.queue_lengths.at(static_cast<Direction>(i)).at("total");
            current_max = std::max(current_max, q);
        }
        max_queue_observed = std::max(max_queue_observed, current_max);

        std::cout << "Time: " << std::fixed << std::setprecision(1)
                  << std::setw(5) << current_time << "s | "
                  << phaseLabel(s.current_phase)
                  << " | Queues N:" << std::setw(2) << s.queue_lengths.at(Direction::NORTH).at("total")
                  << " S:" << std::setw(2) << s.queue_lengths.at(Direction::SOUTH).at("total")
                  << " E:" << std::setw(2) << s.queue_lengths.at(Direction::EAST).at("total")
                  << " W:" << std::setw(2) << s.queue_lengths.at(Direction::WEST).at("total")
                  << " | Processed: " << std::setw(3) << myIntersection.get_vehicles_processed()
                  << "\n";

        current_time += DELTA_TIME;
    }

    int total_processed = myIntersection.get_vehicles_processed();
    double total_wait = myIntersection.get_total_wait_time();
    double avg_wait = (total_processed > 0) ? (total_wait / total_processed) : 0.0;
    double throughput = static_cast<double>(total_processed) / (TOTAL_STEPS / 60.0);

    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "             SIMULATION FINAL REPORT\n";
    std::cout << std::string(50, '=') << "\n";

    std::cout << std::left << std::setw(30) << "Total Simulation Time:" << TOTAL_STEPS << " seconds\n";
    std::cout << std::left << std::setw(30) << "Vehicles Processed:" << total_processed << " vehicles\n";
    std::cout << std::left << std::setw(30) << "Cumulative Wait Time:"
              << std::fixed << std::setprecision(2) << total_wait << " seconds\n";

    std::cout << "\n--- Performance Metrics ---\n";
    std::cout << std::left << std::setw(30) << "Average Wait Time:" << avg_wait << " s/vehicle\n";
    std::cout << std::left << std::setw(30) << "System Throughput:" << throughput << " vehicles/min\n";
    std::cout << std::left << std::setw(30) << "Max Queue Length:" << max_queue_observed << " vehicles\n";

    std::cout << std::left << std::setw(30) << "Intersection LOS:";
    if (avg_wait < 10) std::cout << "A (Excellent)\n";
    else if (avg_wait < 20) std::cout << "B (Very Good)\n";
    else if (avg_wait < 35) std::cout << "C (Good/Stable)\n";
    else if (avg_wait < 55) std::cout << "D (Fair/Unstable)\n";
    else std::cout << "F (Forced Flow/Congested)\n";

    std::cout << std::string(50, '=') << "\n";

    return 0;
}
