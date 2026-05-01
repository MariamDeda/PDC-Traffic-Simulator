// quick_m2_test.cpp
#include "NxNMeshNetwork.h"
#include <iostream>

int main() {
    std::cout << "Testing M2 components...\n";
    
    // Create 2x2 mesh network
    NxNMeshNetwork network(2, ScenarioType::NORMAL);
    
    std::cout << "✅ Network created with " << network.size() << " intersections\n";
    
    // Run one decentralized step
    network.step_decentralized(0.0, 1.0);
    
    std::cout << "✅ Decentralized step executed\n";
    
    // Collect stats
    auto stats = network.collect_stats(10.0);
    std::cout << "✅ Stats collected: " << stats.total_processed << " vehicles processed\n";
    
    return 0;
}