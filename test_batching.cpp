// test_batching.cpp - Compare decentralized simulation with batching enabled vs disabled.
#include "NxNMeshNetwork.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

struct BatchingComparison {
    string test_name;
    int vehicles_processed = 0;
    double avg_wait = 0.0;
    double throughput = 0.0;
    double batching_reduction_percent = 0.0;
    long long simulation_time_ms = 0;
};

BatchingComparison runScenario(const string& name,
                               bool batching_enabled,
                               double batch_interval_seconds,
                               double reduction_estimate_percent) {
    NxNMeshNetwork network(3, ScenarioType::RUSH_HOUR_NS);
    network.enableBatching(batching_enabled, batch_interval_seconds);

    const double SIM_DURATION = 60.0;
    const double TIME_STEP = 1.0;
    double current_time = 0.0;

    auto start = chrono::high_resolution_clock::now();

    for (int step = 0; step < static_cast<int>(SIM_DURATION / TIME_STEP); ++step) {
        network.step_decentralized(current_time, TIME_STEP);
        current_time += TIME_STEP;
    }

    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);

    auto stats = network.collect_stats(SIM_DURATION);

    BatchingComparison result;
    result.test_name = name;
    result.vehicles_processed = stats.total_processed;
    result.avg_wait = stats.avg_wait;
    result.throughput = stats.throughput_vpm;
    result.batching_reduction_percent = reduction_estimate_percent;
    result.simulation_time_ms = duration.count();

    cout << "   Completed: " << stats.total_processed << " vehicles processed\n";

    if (batching_enabled) {
        network.printBatchingStats();
    }

    return result;
}

void printComparisonTable(const vector<BatchingComparison>& results) {
    cout << "\n" << string(90, '=') << "\n";
    cout << "  BATCHING vs NO BATCHING COMPARISON\n";
    cout << string(90, '=') << "\n\n";

    cout << left
         << setw(24) << "Configuration"
         << setw(14) << "Vehicles"
         << setw(16) << "Avg Wait (s)"
         << setw(16) << "Throughput"
         << setw(18) << "Msg Reduction"
         << setw(12) << "Time (ms)"
         << "\n";

    cout << string(90, '-') << "\n";

    for (const auto& r : results) {
        string reduction = (r.batching_reduction_percent > 0.0)
            ? to_string(static_cast<int>(r.batching_reduction_percent)) + "% est."
            : "N/A";

        cout << left
             << setw(24) << r.test_name
             << setw(14) << r.vehicles_processed
             << setw(16) << fixed << setprecision(2) << r.avg_wait
             << setw(16) << fixed << setprecision(1) << r.throughput
             << setw(18) << reduction
             << setw(12) << r.simulation_time_ms
             << "\n";
    }

    cout << string(90, '=') << "\n";
}

int main() {
    cout << "========================================\n";
    cout << "Message Batching Performance Test\n";
    cout << "Milestone 3 - Task: Message Batching\n";
    cout << "========================================\n\n";

    vector<BatchingComparison> results;

    cout << "Test 1: Running WITHOUT batching...\n";
    results.push_back(runScenario("No batching", false, 5.0, 0.0));

    cout << "\nTest 2: Running WITH batching, 5 second interval...\n";
    results.push_back(runScenario("Batching 5s", true, 5.0, 80.0));

    cout << "\nTest 3: Running WITH batching, 10 second interval...\n";
    results.push_back(runScenario("Batching 10s", true, 10.0, 90.0));

    printComparisonTable(results);

    cout << "\nCONCLUSIONS:\n";
    cout << "   Message batching reduces communication overhead by grouping neighbor updates.\n";
    cout << "   Vehicle throughput should remain in the same general range across runs.\n";
    cout << "   Exact vehicle counts vary because the traffic generator uses randomness.\n";
    cout << "\nMessage batching implementation test complete.\n";

    return 0;
}
