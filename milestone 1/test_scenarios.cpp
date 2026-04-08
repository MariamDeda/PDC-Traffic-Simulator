#include "Intersection.h"
#include "CONTROLLER.h"
#include "TrafficGenerator.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>       

using Clock     = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

static TimePoint now() { return Clock::now(); }

// elapsed microseconds between two time points
static long long us(TimePoint start, TimePoint end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

struct ScenarioResult {
    std::string name;
    int         vehicles_spawned;
    int         vehicles_processed;
    double      avg_wait;
    double      throughput;
    int         peak_queue;
    std::string los;


    long long   time_generator_us;    // total time spent in generator.update()
    long long   time_controller_us;   // total time spent in controller.update()
    long long   time_intersection_us; // total time spent in intersection.update()
    long long   time_total_us;        // total wall-clock for the whole run
};


static std::string compute_los(double avg_wait) {
    if (avg_wait < 10) return "A";
    if (avg_wait < 20) return "B";
    if (avg_wait < 35) return "C";
    if (avg_wait < 55) return "D";
    if (avg_wait < 80) return "E";
    return "F";
}

static ScenarioResult run_scenario(const std::string& name,
                                   ScenarioType       scenario,
                                   int                total_steps = 300,
                                   double             delta       = 1.0)
{
    Intersection     intersection("Junction");
    Controller       controller;
    TrafficGenerator generator(scenario);

    int    spawned      = 0;
    int    peak_queue   = 0;
    double current_time = 0.0;

    long long gen_us  = 0;
    long long ctrl_us = 0;
    long long isec_us = 0;

    TimePoint run_start = now();

    for (int step = 0; step < total_steps; ++step) {

        //Track spawned: measure queue before generator 
        IntersectionState pre = intersection.get_state();
        int q_before = pre.queue_lengths.at(Direction::NORTH).at("total")
                     + pre.queue_lengths.at(Direction::SOUTH).at("total")
                     + pre.queue_lengths.at(Direction::EAST).at("total")
                     + pre.queue_lengths.at(Direction::WEST).at("total");

        // gen
        {
            TimePoint t0 = now();
            generator.update(intersection, current_time);
            gen_us += us(t0, now());
        }

        // spawn: queue increase = new arrivals 
        IntersectionState post_spawn = intersection.get_state();
        int q_after = post_spawn.queue_lengths.at(Direction::NORTH).at("total")
                    + post_spawn.queue_lengths.at(Direction::SOUTH).at("total")
                    + post_spawn.queue_lengths.at(Direction::EAST).at("total")
                    + post_spawn.queue_lengths.at(Direction::WEST).at("total");
        spawned += (q_after - q_before);

        // peak queu after spawn b4 processing
        if (q_after > peak_queue) peak_queue = q_after;

        // controller
        {
            TimePoint t0 = now();
            controller.update(intersection, current_time);
            ctrl_us += us(t0, now());
        }

        // intersection
        {
            TimePoint t0 = now();
            intersection.update(delta, current_time + delta);
            isec_us += us(t0, now());
        }

        current_time += delta;
    }

    long long total_us = us(run_start, now());

    int    processed  = intersection.get_vehicles_processed();
    double total_wait = intersection.get_total_wait_time();
    double avg_wait   = processed > 0 ? total_wait / processed : 0.0;
    double throughput = processed / (total_steps * delta / 60.0);

    return { name, spawned, processed, avg_wait, throughput,
             peak_queue, compute_los(avg_wait),
             gen_us, ctrl_us, isec_us, total_us };
}

//print data
static void print_result(const ScenarioResult& r) {
    std::cout << "\n  Scenario           : " << r.name << "\n";
    std::cout << "  Vehicles spawned   : " << r.vehicles_spawned   << "\n";
    std::cout << "  Vehicles processed : " << r.vehicles_processed  << "\n";
    std::cout << "  Avg wait time      : " << std::fixed << std::setprecision(2)
              << r.avg_wait << " s/vehicle\n";
    std::cout << "  Throughput         : " << std::setprecision(1)
              << r.throughput << " veh/min\n";
    std::cout << "  Peak queue         : " << r.peak_queue << " vehicles\n";
    std::cout << "  Level of Service   : " << r.los << "\n";
    std::cout << "  --- Wall-clock timing ---\n";
    std::cout << "  Generator time     : " << std::setw(8) << r.time_generator_us    << " us\n";
    std::cout << "  Controller time    : " << std::setw(8) << r.time_controller_us   << " us\n";
    std::cout << "  Intersection time  : " << std::setw(8) << r.time_intersection_us << " us\n";
    std::cout << "  Total run time     : " << std::setw(8) << r.time_total_us        << " us"
              << "  (" << std::setprecision(3) << r.time_total_us / 1000.0 << " ms)\n";
}

//print table
static void print_metrics_table(const std::vector<ScenarioResult>& results) {
    const int W1=16, W2=9, W3=11, W4=11, W5=12, W6=8, W7=5;
    std::string line(W1+W2+W3+W4+W5+W6+W7+2, '-');

    std::cout << "\n" << line << "\n";
    std::cout << std::left
              << std::setw(W1) << "Scenario"
              << std::setw(W2) << "Spawned"
              << std::setw(W3) << "Processed"
              << std::setw(W4) << "Avg Wait"
              << std::setw(W5) << "Throughput"
              << std::setw(W6) << "PeakQ"
              << std::setw(W7) << "LOS"
              << "\n";
    std::cout << std::left
              << std::setw(W1) << ""
              << std::setw(W2) << "(veh)"
              << std::setw(W3) << "(veh)"
              << std::setw(W4) << "(s/veh)"
              << std::setw(W5) << "(veh/min)"
              << std::setw(W6) << "(veh)"
              << "" << "\n";
    std::cout << line << "\n";

    for (const auto& r : results) {
        std::cout << std::left
                  << std::setw(W1) << r.name
                  << std::setw(W2) << r.vehicles_spawned
                  << std::setw(W3) << r.vehicles_processed
                  << std::setw(W4) << std::fixed << std::setprecision(2) << r.avg_wait
                  << std::setw(W5) << std::setprecision(1)               << r.throughput
                  << std::setw(W6) << r.peak_queue
                  << std::setw(W7) << r.los
                  << "\n";
    }
    std::cout << line << "\n";
}


static void print_timing_table(const std::vector<ScenarioResult>& results) {
    const int W1=16, W2=10, W3=10, W4=10, W5=10, W6=8;
    std::string line(W1+W2+W3+W4+W5+W6+2, '-');

    std::cout << "\n" << line << "\n";
    std::cout << std::left
              << std::setw(W1) << "Scenario"
              << std::setw(W2) << "Gen (us)"
              << std::setw(W3) << "Ctrl (us)"
              << std::setw(W4) << "Isec (us)"
              << std::setw(W5) << "Total (ms)"
              << std::setw(W6) << "Ctrl %"
              << "\n";
    std::cout << line << "\n";

    for (const auto& r : results) {
        double ctrl_pct = r.time_total_us > 0
            ? 100.0 * r.time_controller_us / r.time_total_us
            : 0.0;
        std::cout << std::left
                  << std::setw(W1) << r.name
                  << std::setw(W2) << r.time_generator_us
                  << std::setw(W3) << r.time_controller_us
                  << std::setw(W4) << r.time_intersection_us
                  << std::setw(W5) << std::fixed << std::setprecision(3)
                                   << r.time_total_us / 1000.0
                  << std::setw(W6) << std::setprecision(1) << ctrl_pct << "%"
                  << "\n";
    }
    std::cout << line << "\n";

    // Aggregate totals
    long long sum_gen=0, sum_ctrl=0, sum_isec=0, sum_total=0;
    for (const auto& r : results) {
        sum_gen   += r.time_generator_us;
        sum_ctrl  += r.time_controller_us;
        sum_isec  += r.time_intersection_us;
        sum_total += r.time_total_us;
    }
    double total_ctrl_pct = sum_total > 0
        ? 100.0 * sum_ctrl / sum_total : 0.0;

    std::cout << line << "\n";
    std::cout << std::left
              << std::setw(W1) << "ALL SCENARIOS"
              << std::setw(W2) << sum_gen
              << std::setw(W3) << sum_ctrl
              << std::setw(W4) << sum_isec
              << std::setw(W5) << std::fixed << std::setprecision(3)
                               << sum_total / 1000.0
              << std::setw(W6) << std::setprecision(1) << total_ctrl_pct << "%"
              << "\n";
    std::cout << line << "\n";

    std::cout << "\n  NOTE FOR M2: Controller::update() ("
              << std::setprecision(1) << total_ctrl_pct
              << "% of runtime) is the\n"
              << "  parallelizable boundary. Each intersection's controller\n"
              << "  runs independently — replace the sequential loop with\n"
              << "  std::for_each(std::execution::par_unseq, ...) and\n"
              << "  re-run this file to measure speedup.\n";
}

int main() {
    const int    SIM_STEPS = 300;
    const double DELTA     = 1.0;

    std::cout << "========================================\n";
    std::cout << "  Traffic Scenario Benchmark\n";
    std::cout << "  Duration: " << SIM_STEPS << "s | Timestep: " << DELTA << "s\n";
    std::cout << "  Timing: std::chrono::steady_clock (us)\n";
    std::cout << "========================================\n";

    std::vector<ScenarioResult> results;
    std::cout << "\nRunning scenarios...\n";

    auto run = [&](const std::string& label, ScenarioType sc) {
        std::cout << "  " << std::left << std::setw(18) << label;
        auto r = run_scenario(label, sc, SIM_STEPS, DELTA);
        results.push_back(r);
        std::cout << "done  ("
                  << "processed: " << std::setw(3) << r.vehicles_processed
                  << "  total: " << std::setw(6) << r.time_total_us << " us)\n";
    };

    run("Low Traffic",  ScenarioType::LOW_TRAFFIC);
    run("Normal",       ScenarioType::NORMAL);
    run("Rush Hour NS", ScenarioType::RUSH_HOUR_NS);
    run("Rush Hour EW", ScenarioType::RUSH_HOUR_EW);
    run("Peak Switch",  ScenarioType::PEAK_SWITCH);
    run("Accident",     ScenarioType::ACCIDENT);

    // blocks
    std::cout << "\n========================================\n";
    std::cout << "  INDIVIDUAL SCENARIO RESULTS\n";
    std::cout << "========================================\n";
    for (const auto& r : results) {
        std::cout << "----------------------------------------";
        print_result(r);
    }

    // table
    std::cout << "\n========================================\n";
    std::cout << "  TRAFFIC METRICS SUMMARY\n";
    std::cout << "========================================\n";
    print_metrics_table(results);

    // breakdown
    std::cout << "\n========================================\n";
    std::cout << "  RUNTIME BREAKDOWN  (M1 sequential baseline)\n";
    std::cout << "========================================\n";
    print_timing_table(results);

    std::cout << "\nLOS Key: A<10s  B 10-20s  C 20-35s  D 35-55s  E 55-80s  F>=80s\n";
    std::cout << "\n========================================\n";
    std::cout << "  Benchmark Complete\n";
    std::cout << "========================================\n";
    return 0;
}