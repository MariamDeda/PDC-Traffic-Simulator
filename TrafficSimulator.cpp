#include "Intersection.h"
#include "TrafficGenerator.h"
#include <iostream>
#include <iomanip>
#include <windows.h> // sleep

int main() {
    // 1. Setup
    const double DELTA_TIME = 1.0;
    const int TOTAL_STEPS = 300; // Increased to 300 for better data collection
    double current_time = 0.0;
    
    // Pedestrian Control Variables
    bool ped_pending = false;
    double last_ped_end_time = -100.0; 
    const double PED_COOLDOWN = 30.0;  

    TrafficGenerator myGenerator(ScenarioType::RUSH_HOUR_NS);
    
    TimingConstraints config;
    config.min_green = 15;        
    config.yellow_time = 3;
    config.pedestrian_time = 10;
    
    Intersection myIntersection("IBA_Junction", config);

    // Trackers for evaluation metrics
    int max_queue_observed = 0;

    std::cout << "Starting Simulation: " << myIntersection.get_id() << "\n";
    std::cout << "Scenario: RUSH_HOUR_NS | Duration: " << TOTAL_STEPS << "s\n\n";

    for (int step = 0; step < TOTAL_STEPS; ++step) {
        myGenerator.update(myIntersection, current_time);

        // Pedestrian Logic
        if (!ped_pending && (current_time - last_ped_end_time > PED_COOLDOWN)) {
            if (rand() % 100 < 5) { 
                ped_pending = true;
            }
        }

        if (ped_pending) {
            bool accepted = myIntersection.set_phase(SignalPhase::PEDESTRIAN_CROSSING, 
                                                    config.pedestrian_time, current_time);
            if (accepted) {
                ped_pending = false;
                last_ped_end_time = current_time + config.pedestrian_time;
            }
        }

        myIntersection.update(DELTA_TIME, current_time);

        // live state
        IntersectionState s = myIntersection.get_state();
        
        // max queue for eval
        int current_max = 0;
        for (int i = 0; i < 4; ++i) {
            int q = s.queue_lengths.at(static_cast<Direction>(i)).at("total");
            if (q > current_max) current_max = q;
        }
        if (current_max > max_queue_observed) max_queue_observed = current_max;

        // Console Output
        std::cout << "Time: " << std::fixed << std::setprecision(1) << std::setw(5) << current_time << "s | ";
        if (s.current_phase == SignalPhase::PEDESTRIAN_CROSSING) std::cout << "[WALK]  ";
        else if (s.current_phase == SignalPhase::NORTH_SOUTH_GREEN) std::cout << "NS-GRN ";
        else if (s.current_phase == SignalPhase::EAST_WEST_GREEN) std::cout << "EW-GRN ";
        else std::cout << "YELLOW ";

        std::cout << "| Queues N:" << std::setw(2) << s.queue_lengths.at(Direction::NORTH).at("total")
                  << " S:" << std::setw(2) << s.queue_lengths.at(Direction::SOUTH).at("total")
                  << " | Processed: " << std::setw(3) << myIntersection.get_vehicles_processed() << "\n";

        current_time += DELTA_TIME;
        // Sleep(10); // Faster simulation
    }
    int total_processed = myIntersection.get_vehicles_processed();
    double total_wait = myIntersection.get_total_wait_time();
    double avg_wait = (total_processed > 0) ? (total_wait / total_processed) : 0.0;
    double throughput = (double)total_processed / (TOTAL_STEPS / 60.0); // Vehicles per minute

    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "             SIMULATION FINAL REPORT\n";
    std::cout << std::string(50, '=') << "\n";
    
    std::cout << std::left << std::setw(30) << "Total Simulation Time:" << TOTAL_STEPS << " seconds\n";
    std::cout << std::left << std::setw(30) << "Vehicles Processed:" << total_processed << " vehicles\n";
    std::cout << std::left << std::setw(30) << "Cumulative Wait Time:" << std::fixed << std::setprecision(2) << total_wait << " seconds\n";
    
    std::cout << "\n--- Performance Metrics ---\n";
    std::cout << std::left << std::setw(30) << "Average Wait Time:" << avg_wait << " s/vehicle\n";
    std::cout << std::left << std::setw(30) << "System Throughput:" << throughput << " vehicles/min\n";
    std::cout << std::left << std::setw(30) << "Max Queue Length:" << max_queue_observed << " vehicles\n";
    
    // los approx
    std::cout << std::left << std::setw(30) << "Intersection LOS:";
    if (avg_wait < 10) std::cout << "A (Excellent)\n";
    else if (avg_wait < 20) std::cout << "B (Very Good)\n";
    else if (avg_wait < 35) std::cout << "C (Good/Stable)\n";
    else if (avg_wait < 55) std::cout << "D (Fair/Unstable)\n";
    else std::cout << "F (Forced Flow/Congested)\n";

    std::cout << std::string(50, '=') << "\n";

    return 0;
}