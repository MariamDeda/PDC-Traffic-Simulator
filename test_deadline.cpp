// test_deadline.cpp
#include "DecentralizedController.h"
#include "Intersection.h"
#include "TrafficGenerator.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;

int main() {
    cout << "========================================\n";
    cout << "Real-Time Deadline Enforcement Test\n";
    cout << "========================================\n\n";
    
    // Simple test with a single intersection first
    cout << "Test: Creating intersection with deadline enforcement...\n";
    
    // Create an intersection
    TimingConstraints constraints;
    Intersection intersection("Test_Intersection", constraints);
    
    // Create a controller with deadline enforcement
    DecentralizedController controller("Test_Controller");
    
    // Enable deadline enforcement (50ms deadline)
    controller.enableDeadlineEnforcement(true, 50.0);
    
    // Add some vehicles
    cout << "Adding vehicles...\n";
    for (int i = 0; i < 10; i++) {
        auto vehicle = make_shared<Vehicle>(
            "car" + to_string(i),
            VehicleType::CAR,
            Direction::NORTH,
            TurnType::STRAIGHT,
            0.0
        );
        intersection.add_vehicle(vehicle);
    }
    
    // Run a few decision cycles
    cout << "Running decision cycles...\n";
    double current_time = 0.0;
    
    for (int cycle = 0; cycle < 5; cycle++) {
        // Build a dummy neighbor message (empty for this test)
        vector<pair<NeighborLink, NeighborMessage>> empty_neighbors;
        
        // Make a decision
        PhaseDecision decision = controller.decide(intersection, empty_neighbors, current_time);
        
        cout << "  Cycle " << cycle + 1 << ": " << decision.reason << "\n";
        cout << "    Phase: " << (decision.phase == SignalPhase::NORTH_SOUTH_GREEN ? "NS_GREEN" : "EW_GREEN")
             << ", Duration: " << decision.duration << "s\n";
        
        current_time += 1.0;
    }
    
    // Print statistics
    cout << "\n";
    controller.printDeadlineStats();
    
    cout << "\n✅ Deadline enforcement test complete!\n";
    
    return 0;
}