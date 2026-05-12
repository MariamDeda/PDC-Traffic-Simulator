#ifndef REALTIME_DEADLINE_H
#define REALTIME_DEADLINE_H

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>

// Forward declarations (these are defined in your other headers)
enum class SignalPhase;
enum class Direction;
struct TimingConstraints;
struct IntersectionState;

// ============== DEADLINE TRACKING STATISTICS ==============
struct DeadlineStats {
    int total_decisions = 0;
    int deadline_misses = 0;
    int fallback_used = 0;
    double max_decision_time_ms = 0.0;
    double avg_decision_time_ms = 0.0;
    
    void reset() {
        total_decisions = 0;
        deadline_misses = 0;
        fallback_used = 0;
        max_decision_time_ms = 0.0;
        avg_decision_time_ms = 0.0;
    }
    
    void print() const {
        std::cout << "\n   === DEADLINE STATISTICS ===\n";
        std::cout << "   Total decisions:     " << total_decisions << "\n";
        std::cout << "   Deadline misses:     " << deadline_misses;
        if (total_decisions > 0) {
            std::cout << " (" << (100.0 * deadline_misses / total_decisions) << "%)";
        }
        std::cout << "\n";
        std::cout << "   Fallback used:       " << fallback_used << "\n";
        std::cout << "   Max decision time:   " << max_decision_time_ms << " ms\n";
        std::cout << "   Avg decision time:   " << avg_decision_time_ms << " ms\n";
    }
};

// ============== DEADLINE ENFORCER ==============
class DeadlineEnforcer {
private:
    double deadline_ms_;
    bool deadline_enabled_;
    DeadlineStats stats_;
    std::chrono::high_resolution_clock::time_point start_time_;
    
public:
    DeadlineEnforcer(double deadline_ms = 50.0, bool enabled = true)
        : deadline_ms_(deadline_ms), deadline_enabled_(enabled) {}
    
    void startDecision() {
        if (!deadline_enabled_) return;
        start_time_ = std::chrono::high_resolution_clock::now();
    }
    
    bool checkDeadline() {
        if (!deadline_enabled_) return true;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
        double duration_ms = duration.count() / 1000.0;
        
        stats_.total_decisions++;
        stats_.avg_decision_time_ms = (stats_.avg_decision_time_ms * (stats_.total_decisions - 1) + duration_ms) 
                                      / stats_.total_decisions;
        stats_.max_decision_time_ms = std::max(stats_.max_decision_time_ms, duration_ms);
        
        if (duration_ms > deadline_ms_) {
            stats_.deadline_misses++;
            return false;
        }
        return true;
    }
    
    void recordFallbackUsed() {
        stats_.fallback_used++;
    }
    
    bool isEnabled() const { return deadline_enabled_; }
    double getDeadlineMs() const { return deadline_ms_; }
    void setDeadlineMs(double ms) { deadline_ms_ = ms; }
    void enable(bool enable) { deadline_enabled_ = enable; }
    const DeadlineStats& getStats() const { return stats_; }
    void printStats() const { stats_.print(); }
    void resetStats() { stats_.reset(); }
};

// ============== SIMPLE FALLBACK HELPER (using .at() instead of []) ==========
struct SimpleFallbackHelper {
    // Helper to safely get queue length from const state
    static int getQueueTotal(const IntersectionState& state, Direction dir);
    static int getNorthSouthTotal(const IntersectionState& state);
    static int getEastWestTotal(const IntersectionState& state);
};

#endif // REALTIME_DEADLINE_H