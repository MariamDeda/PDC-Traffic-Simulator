// test_deadline.cpp - Basic real-time deadline enforcement smoke test.
#include "DecentralizedController.h"
#include "Intersection.h"

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

using namespace std;

int main() {
    cout << "========================================\n";
    cout << "Real-Time Deadline Enforcement Test\n";
    cout << "========================================\n\n";

    TimingConstraints constraints;
    Intersection intersection("Test_Intersection", constraints);

    DecentralizedController controller("Test_Controller");
    controller.enableDeadlineEnforcement(true, 50.0);

    cout << "Adding vehicles...\n";
    for (int i = 0; i < 10; ++i) {
        auto vehicle = make_shared<Vehicle>(
            "car" + to_string(i),
            VehicleType::CAR,
            Direction::NORTH,
            TurnType::STRAIGHT,
            0.0
        );
        intersection.add_vehicle(vehicle);
    }

    cout << "Running decision cycles...\n";
    double current_time = 0.0;
    const double DELTA_TIME = 1.0;

    vector<pair<NeighborLink, NeighborMessage>> empty_neighbors;

    for (int cycle = 0; cycle < 5; ++cycle) {
        PhaseDecision decision = controller.decide(
            intersection,
            empty_neighbors,
            current_time
        );

        controller.apply_decision(intersection, decision, current_time);
        intersection.update(DELTA_TIME, current_time);

        cout << "  Cycle " << cycle + 1 << ": " << decision.reason << "\n";
        cout << "    Phase: "
             << (decision.phase == SignalPhase::NORTH_SOUTH_GREEN ? "NS_GREEN" : "EW_GREEN")
             << ", Duration: " << decision.duration << "s\n";

        current_time += DELTA_TIME;
    }

    cout << "\n";
    controller.printDeadlineStats();

    cout << "\nDeadline enforcement smoke test complete.\n";

    return 0;
}
