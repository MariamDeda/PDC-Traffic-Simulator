#ifndef NXN_MESH_NETWORK_H
#define NXN_MESH_NETWORK_H

/*
 * NxNMeshNetwork.h
 *
 * Supports:
 *  - sequential centralized simulation
 *  - sequential decentralized simulation
 *  - parallel epoch simulation
 *  - fully asynchronous per-node simulation
 *  - message batching statistics
 *  - deadline enforcement statistics
 */

#include "Intersection.h"
#include "CONTROLLER.h"
#include "DecentralizedController.h"
#include "TrafficGenerator.h"
#include "MessageBuffer.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

// ============================================================
// Thread-safe wrapper around Intersection
// ============================================================
class ThreadSafeIntersection {
public:
    explicit ThreadSafeIntersection(
        const std::string& id,
        const TimingConstraints& constraints = TimingConstraints())
        : intersection_(id, constraints) {}

    bool set_phase(SignalPhase phase, double duration, double current_time) {
        std::lock_guard<std::mutex> lock(mtx_);
        return intersection_.set_phase(phase, duration, current_time);
    }

    void update(double delta_time, double current_time) {
        std::lock_guard<std::mutex> lock(mtx_);
        intersection_.update(delta_time, current_time);
    }

    void add_vehicle(std::shared_ptr<Vehicle> vehicle) {
        std::lock_guard<std::mutex> lock(mtx_);
        intersection_.add_vehicle(std::move(vehicle));
    }

    IntersectionState get_state() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return intersection_.get_state();
    }

    int get_vehicles_processed() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return intersection_.get_vehicles_processed();
    }

    double get_total_wait_time() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return intersection_.get_total_wait_time();
    }

    std::string get_id() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return intersection_.get_id();
    }

    const TimingConstraints& get_constraints() const {
        return intersection_.get_constraints();
    }

    // Raw access is intended for sequential code or for the owning worker thread.
    Intersection& raw() {
        return intersection_;
    }

    const Intersection& raw() const {
        return intersection_;
    }

private:
    mutable std::mutex mtx_;
    Intersection intersection_;
};

// ============================================================
// Latest-message mailbox for one node
// ============================================================
class NodeMailbox {
public:
    NodeMailbox()
        : slot_(std::make_shared<NeighborMessage>()) {}

    void publish(const NeighborMessage& message) {
        auto ptr = std::make_shared<NeighborMessage>(message);
        std::atomic_store_explicit(&slot_, ptr, std::memory_order_release);
    }

    std::shared_ptr<NeighborMessage> read() const {
        return std::atomic_load_explicit(&slot_, std::memory_order_acquire);
    }

private:
    std::shared_ptr<NeighborMessage> slot_;
};

// ============================================================
// NxNMeshNetwork
// ============================================================
class NxNMeshNetwork {
public:
    struct Node {
        int row = 0;
        int col = 0;

        std::shared_ptr<ThreadSafeIntersection> ts_intersection;
        std::shared_ptr<DecentralizedController> dc_controller;
        std::shared_ptr<TrafficGenerator> generator;

        std::vector<NeighborLink> neighbors;

        Intersection& intersection() {
            return ts_intersection->raw();
        }

        const Intersection& intersection() const {
            return ts_intersection->raw();
        }
    };

    struct NetworkStats {
        int total_processed = 0;
        double total_wait = 0.0;
        double avg_wait = 0.0;
        int peak_queue = 0;
        double throughput_vpm = 0.0;
    };

    struct AsyncStats {
        long long wall_time_us = 0;
        std::vector<long long> node_runtime_us;
        std::vector<int> node_steps_completed;
        int total_node_steps = 0;
        double avg_node_steps = 0.0;
        int min_node_steps = 0;
        int max_node_steps = 0;
    };

    explicit NxNMeshNetwork(
        int n,
        ScenarioType default_scenario = ScenarioType::NORMAL,
        const TimingConstraints& constraints = TimingConstraints(),
        double batch_interval_seconds = 5.0,
        bool enable_batching = true)
        : n_(n),
          batch_interval_seconds_(batch_interval_seconds),
          batching_enabled_(enable_batching) {
        if (n_ < 1) {
            throw std::invalid_argument("Grid size n must be >= 1");
        }

        nodes_.resize(size());
        mailboxes_.resize(size());
        buffers_.resize(size(), MessageBuffer(batch_interval_seconds_, 50));

        reset_async_stats();

        for (int r = 0; r < n_; ++r) {
            for (int c = 0; c < n_; ++c) {
                const int idx = index(r, c);
                Node& nd = nodes_[idx];

                nd.row = r;
                nd.col = c;

                const std::string id = make_id(r, c);

                nd.ts_intersection =
                    std::make_shared<ThreadSafeIntersection>(id, constraints);

                nd.dc_controller =
                    std::make_shared<DecentralizedController>(id);

                nd.generator =
                    std::make_shared<TrafficGenerator>(default_scenario);
            }
        }

        wire_neighbors();
        initialize_mailboxes(0.0);
    }

    // ========================================================
    // Basic accessors
    // ========================================================
    int n() const {
        return n_;
    }

    int size() const {
        return n_ * n_;
    }

    std::string node_id(int row, int col) const {
        check_bounds(row, col);
        return make_id(row, col);
    }

    Node& node(int row, int col) {
        check_bounds(row, col);
        return nodes_[index(row, col)];
    }

    const Node& node(int row, int col) const {
        check_bounds(row, col);
        return nodes_[index(row, col)];
    }

    const std::vector<Node>& nodes() const {
        return nodes_;
    }

    void set_scenario(int row, int col, ScenarioType scenario) {
        check_bounds(row, col);
        nodes_[index(row, col)].generator =
            std::make_shared<TrafficGenerator>(scenario);
    }

    // ========================================================
    // Feature controls
    // ========================================================
    void enableBatching(bool enable, double interval_seconds = 5.0) {
        batching_enabled_ = enable;
        batch_interval_seconds_ = interval_seconds;

        for (auto& buffer : buffers_) {
            buffer = MessageBuffer(batch_interval_seconds_, 50);
        }

        std::cout << "[NxNMeshNetwork] Batching "
                  << (batching_enabled_ ? "ENABLED" : "DISABLED")
                  << " | interval=" << batch_interval_seconds_ << "s\n";
    }

    void enableDeadlineEnforcementForAll(bool enable,
                                         double deadline_ms = 50.0) {
        for (auto& nd : nodes_) {
            nd.dc_controller->enableDeadlineEnforcement(enable, deadline_ms);
        }
    }

    // ========================================================
    // Sequential centralized step
    // ========================================================
    void step_centralized(std::vector<Controller>& controllers,
                          double current_time,
                          double delta_time) {
        assert(static_cast<int>(controllers.size()) == size());

        for (auto& nd : nodes_) {
            nd.generator->update(nd.intersection(), current_time);
        }

        for (int i = 0; i < size(); ++i) {
            controllers[i].update(nodes_[i].intersection(), current_time);
        }

        for (auto& nd : nodes_) {
            nd.intersection().update(delta_time, current_time + delta_time);
        }

        initialize_mailboxes(current_time + delta_time);
    }

    // ========================================================
    // Sequential decentralized step
    // ========================================================
    void step_decentralized(double current_time, double delta_time) {
        for (auto& nd : nodes_) {
            nd.generator->update(nd.intersection(), current_time);
        }

        std::vector<NeighborMessage> fresh(size());

        for (int i = 0; i < size(); ++i) {
            fresh[i] = nodes_[i].dc_controller->build_local_message(
                nodes_[i].intersection(), current_time);

            mailboxes_[i].publish(fresh[i]);
        }

        if (batching_enabled_) {
            for (int i = 0; i < size(); ++i) {
                for (const NeighborLink& link : nodes_[i].neighbors) {
                    buffers_[i].addMessage(link.neighbor_id, fresh[i]);
                }
            }

            bool should_flush = false;
            for (const auto& buffer : buffers_) {
                if (buffer.shouldFlush(current_time)) {
                    should_flush = true;
                    break;
                }
            }

            if (should_flush) {
                for (auto& buffer : buffers_) {
                    buffer.flush(current_time);
                }
            }
        } else {
            for (int i = 0; i < size(); ++i) {
                for (const NeighborLink& link : nodes_[i].neighbors) {
                    buffers_[i].sendImmediate(link.neighbor_id, fresh[i]);
                }
            }
        }

        for (int i = 0; i < size(); ++i) {
            Node& nd = nodes_[i];

            std::vector<std::pair<NeighborLink, NeighborMessage>> inbox =
                build_latest_neighbor_inbox(i);

            PhaseDecision decision =
                nd.dc_controller->decide(nd.intersection(), inbox, current_time);

            nd.dc_controller->apply_decision(
                nd.intersection(), decision, current_time);
        }

        for (auto& nd : nodes_) {
            nd.intersection().update(delta_time, current_time + delta_time);
        }
    }

    // ========================================================
    // Parallel epoch mode
    // ========================================================
    long long run_parallel_epoch(int steps, double delta_time) {
        using Clock = std::chrono::steady_clock;
        using Micros = std::chrono::microseconds;

        if (steps <= 0) {
            return 0;
        }

        initialize_mailboxes(0.0);
        reset_async_stats();

        epoch_stop_flag_.store(false, std::memory_order_release);
        epoch_generation_.store(0, std::memory_order_release);
        epoch_done_count_.store(0, std::memory_order_release);

        epoch_current_time_ = 0.0;
        epoch_delta_time_ = delta_time;

        std::vector<std::thread> workers;
        workers.reserve(size());

        for (int i = 0; i < size(); ++i) {
            workers.emplace_back([this, i]() {
                epoch_worker_loop(i);
            });
        }

        const auto wall_start = Clock::now();

        for (int ep = 0; ep < steps; ++ep) {
            epoch_done_count_.store(0, std::memory_order_release);

            epoch_generation_.fetch_add(1, std::memory_order_acq_rel);
            epoch_cv_.notify_all();

            {
                std::unique_lock<std::mutex> lock(epoch_done_mtx_);
                epoch_done_cv_.wait(lock, [this]() {
                    return epoch_done_count_.load(std::memory_order_acquire) ==
                           size();
                });
            }

            epoch_current_time_ += delta_time;
        }

        epoch_stop_flag_.store(true, std::memory_order_release);
        epoch_generation_.fetch_add(1, std::memory_order_acq_rel);
        epoch_cv_.notify_all();

        for (auto& t : workers) {
            if (t.joinable()) {
                t.join();
            }
        }

        const long long wall_us =
            std::chrono::duration_cast<Micros>(Clock::now() - wall_start).count();

        last_async_wall_time_us_ = wall_us;
        return wall_us;
    }

    long long run_parallel(int steps, double delta_time) {
        return run_parallel_epoch(steps, delta_time);
    }

    // ========================================================
    // Fully asynchronous mode
    // ========================================================
    long long run_async(int local_steps_per_node,
                        double delta_time,
                        int optional_sleep_ms_per_local_step = 0) {
        using Clock = std::chrono::steady_clock;
        using Micros = std::chrono::microseconds;

        if (local_steps_per_node <= 0) {
            return 0;
        }

        initialize_mailboxes(0.0);
        reset_async_stats();

        async_stop_flag_.store(false, std::memory_order_release);

        std::vector<std::thread> workers;
        workers.reserve(size());

        const auto wall_start = Clock::now();

        for (int i = 0; i < size(); ++i) {
            workers.emplace_back([this,
                                  i,
                                  local_steps_per_node,
                                  delta_time,
                                  optional_sleep_ms_per_local_step]() {
                async_worker_loop(i,
                                  local_steps_per_node,
                                  delta_time,
                                  optional_sleep_ms_per_local_step);
            });
        }

        for (auto& t : workers) {
            if (t.joinable()) {
                t.join();
            }
        }

        async_stop_flag_.store(true, std::memory_order_release);

        const long long wall_us =
            std::chrono::duration_cast<Micros>(Clock::now() - wall_start).count();

        last_async_wall_time_us_ = wall_us;
        return wall_us;
    }

    std::vector<long long> parallel_node_us() const {
        std::vector<long long> result;
        result.reserve(async_node_runtime_us_.size());

        for (const auto& value : async_node_runtime_us_) {
            result.push_back(value.load(std::memory_order_relaxed));
        }

        return result;
    }

    AsyncStats getAsyncStats() const {
        AsyncStats stats;
        stats.wall_time_us = last_async_wall_time_us_;

        stats.node_runtime_us.reserve(async_node_runtime_us_.size());
        for (const auto& value : async_node_runtime_us_) {
            stats.node_runtime_us.push_back(
                value.load(std::memory_order_relaxed));
        }

        stats.node_steps_completed.reserve(async_node_steps_.size());
        for (const auto& value : async_node_steps_) {
            stats.node_steps_completed.push_back(
                value.load(std::memory_order_relaxed));
        }

        if (stats.node_steps_completed.empty()) {
            return stats;
        }

        stats.min_node_steps = stats.node_steps_completed[0];
        stats.max_node_steps = stats.node_steps_completed[0];

        for (int steps : stats.node_steps_completed) {
            stats.total_node_steps += steps;
            stats.min_node_steps = std::min(stats.min_node_steps, steps);
            stats.max_node_steps = std::max(stats.max_node_steps, steps);
        }

        stats.avg_node_steps =
            static_cast<double>(stats.total_node_steps) /
            static_cast<double>(stats.node_steps_completed.size());

        return stats;
    }

    void printAsyncStats() const {
        AsyncStats stats = getAsyncStats();

        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "  ASYNCHRONOUS COORDINATION STATS\n";
        std::cout << std::string(70, '=') << "\n";

        std::cout << "Wall time:          "
                  << stats.wall_time_us << " us\n";
        std::cout << "Total node steps:   "
                  << stats.total_node_steps << "\n";
        std::cout << "Avg steps/node:     "
                  << std::fixed << std::setprecision(2)
                  << stats.avg_node_steps << "\n";
        std::cout << "Min steps/node:     "
                  << stats.min_node_steps << "\n";
        std::cout << "Max steps/node:     "
                  << stats.max_node_steps << "\n";

        std::cout << "\nPer-node runtime:\n";
        for (int i = 0; i < size(); ++i) {
            const int steps =
                (i < static_cast<int>(stats.node_steps_completed.size()))
                    ? stats.node_steps_completed[i]
                    : 0;

            const long long runtime =
                (i < static_cast<int>(stats.node_runtime_us.size()))
                    ? stats.node_runtime_us[i]
                    : 0;

            std::cout << "  " << nodes_[i].ts_intersection->get_id()
                      << " | steps=" << std::setw(5) << steps
                      << " | runtime=" << std::setw(10)
                      << runtime << " us\n";
        }

        std::cout << std::string(70, '=') << "\n";
    }

    // ========================================================
    // Metrics and printing
    // ========================================================
    NetworkStats collect_stats(double simulation_duration_seconds) const {
        NetworkStats s;

        for (const auto& nd : nodes_) {
            s.total_processed += nd.ts_intersection->get_vehicles_processed();
            s.total_wait += nd.ts_intersection->get_total_wait_time();

            IntersectionState st = nd.ts_intersection->get_state();

            const int q =
                st.queue_lengths.at(Direction::NORTH).at("total") +
                st.queue_lengths.at(Direction::SOUTH).at("total") +
                st.queue_lengths.at(Direction::EAST).at("total") +
                st.queue_lengths.at(Direction::WEST).at("total");

            s.peak_queue = std::max(s.peak_queue, q);
        }

        s.avg_wait =
            s.total_processed > 0
                ? s.total_wait / static_cast<double>(s.total_processed)
                : 0.0;

        s.throughput_vpm =
            simulation_duration_seconds > 0.0
                ? s.total_processed / (simulation_duration_seconds / 60.0)
                : 0.0;

        return s;
    }

    void print_grid(double current_time) const {
        std::cout << "\n  Grid snapshot @ t="
                  << std::fixed << std::setprecision(1)
                  << current_time << "s\n";

        std::cout << "  ";
        for (int c = 0; c < n_; ++c) {
            std::cout << std::setw(12)
                      << ("col" + std::to_string(c));
        }
        std::cout << "\n";

        for (int r = 0; r < n_; ++r) {
            std::cout << "  row" << r;

            for (int c = 0; c < n_; ++c) {
                const int idx = index(r, c);
                IntersectionState st = nodes_[idx].ts_intersection->get_state();

                const int q =
                    st.queue_lengths.at(Direction::NORTH).at("total") +
                    st.queue_lengths.at(Direction::SOUTH).at("total") +
                    st.queue_lengths.at(Direction::EAST).at("total") +
                    st.queue_lengths.at(Direction::WEST).at("total");

                std::string phase = phase_short_name(st.current_phase);

                std::ostringstream cell;
                cell << phase << "/Q" << q;

                std::cout << std::setw(12) << cell.str();
            }

            std::cout << "\n";
        }
    }

    void printBatchingStats() const {
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "  MESSAGE BATCHING STATISTICS\n";
        std::cout << std::string(70, '=') << "\n";

        for (int i = 0; i < size(); ++i) {
            std::cout << "\nNode "
                      << nodes_[i].ts_intersection->get_id()
                      << ":\n";
            buffers_[i].printStats();
        }
    }

    void printAllDeadlineStats() const {
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "  DEADLINE ENFORCEMENT STATISTICS\n";
        std::cout << std::string(70, '=') << "\n";

        for (const auto& nd : nodes_) {
            std::cout << "\nNode "
                      << nd.ts_intersection->get_id()
                      << ":\n";
            nd.dc_controller->printDeadlineStats();
        }
    }

private:
    // ========================================================
    // Worker implementation: epoch-parallel mode
    // ========================================================
    void epoch_worker_loop(int idx) {
        int observed_generation = 0;

        while (true) {
            {
                std::unique_lock<std::mutex> lock(epoch_mtx_);

                epoch_cv_.wait(lock, [this, &observed_generation]() {
                    return epoch_generation_.load(std::memory_order_acquire) >
                               observed_generation ||
                           epoch_stop_flag_.load(std::memory_order_acquire);
                });

                observed_generation =
                    epoch_generation_.load(std::memory_order_acquire);
            }

            if (epoch_stop_flag_.load(std::memory_order_acquire)) {
                break;
            }

            const double local_time = epoch_current_time_;
            execute_one_node_step(idx, local_time, epoch_delta_time_);

            if (epoch_done_count_.fetch_add(1, std::memory_order_acq_rel) + 1 ==
                size()) {
                std::lock_guard<std::mutex> lock(epoch_done_mtx_);
                epoch_done_cv_.notify_one();
            }
        }
    }

    // ========================================================
    // Worker implementation: fully async mode
    // ========================================================
    void async_worker_loop(int idx,
                           int local_steps,
                           double delta_time,
                           int optional_sleep_ms) {
        double local_time = 0.0;

        for (int step = 0; step < local_steps; ++step) {
            if (async_stop_flag_.load(std::memory_order_acquire)) {
                break;
            }

            execute_one_node_step(idx, local_time, delta_time);
            local_time += delta_time;

            if (optional_sleep_ms > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(optional_sleep_ms));
            }
        }
    }

    // ========================================================
    // One local node decision/update cycle
    // ========================================================
    void execute_one_node_step(int idx,
                               double local_time,
                               double delta_time) {
        using Clock = std::chrono::steady_clock;
        using Micros = std::chrono::microseconds;

        const auto start = Clock::now();

        Node& nd = nodes_[idx];

        nd.generator->update(nd.intersection(), local_time);

        NeighborMessage local_message =
            nd.dc_controller->build_local_message(nd.intersection(), local_time);

        mailboxes_[idx].publish(local_message);

        if (batching_enabled_) {
            for (const NeighborLink& link : nd.neighbors) {
                buffers_[idx].addMessage(link.neighbor_id, local_message);
            }

            if (buffers_[idx].shouldFlush(local_time)) {
                buffers_[idx].flush(local_time);
            }
        } else {
            for (const NeighborLink& link : nd.neighbors) {
                buffers_[idx].sendImmediate(link.neighbor_id, local_message);
            }
        }

        std::vector<std::pair<NeighborLink, NeighborMessage>> inbox =
            build_latest_neighbor_inbox(idx);

        PhaseDecision decision =
            nd.dc_controller->decide(nd.intersection(), inbox, local_time);

        nd.dc_controller->apply_decision(
            nd.intersection(), decision, local_time);

        nd.intersection().update(delta_time, local_time + delta_time);

        const long long elapsed =
            std::chrono::duration_cast<Micros>(Clock::now() - start).count();

        async_node_runtime_us_[idx].fetch_add(
            elapsed, std::memory_order_relaxed);

        async_node_steps_[idx].fetch_add(
            1, std::memory_order_relaxed);
    }

    std::vector<std::pair<NeighborLink, NeighborMessage>>
    build_latest_neighbor_inbox(int idx) const {
        std::vector<std::pair<NeighborLink, NeighborMessage>> inbox;

        const Node& nd = nodes_[idx];
        inbox.reserve(nd.neighbors.size());

        for (const NeighborLink& link : nd.neighbors) {
            const int neighbor_idx = node_index_by_id(link.neighbor_id);

            std::shared_ptr<NeighborMessage> msg_ptr =
                mailboxes_[neighbor_idx].read();

            if (msg_ptr) {
                inbox.push_back({link, *msg_ptr});
            }
        }

        return inbox;
    }

    void initialize_mailboxes(double current_time) {
        for (int i = 0; i < size(); ++i) {
            NeighborMessage msg =
                nodes_[i].dc_controller->build_local_message(
                    nodes_[i].intersection(), current_time);

            mailboxes_[i].publish(msg);
        }
    }

    void reset_async_stats() {
        last_async_wall_time_us_ = 0;

        async_node_runtime_us_ =
            std::vector<std::atomic<long long>>(size());

        async_node_steps_ =
            std::vector<std::atomic<int>>(size());

        for (int i = 0; i < size(); ++i) {
            async_node_runtime_us_[i].store(0, std::memory_order_relaxed);
            async_node_steps_[i].store(0, std::memory_order_relaxed);
        }
    }

    // ========================================================
    // Mesh helpers
    // ========================================================
    int index(int row, int col) const {
        return row * n_ + col;
    }

    static std::string make_id(int row, int col) {
        return "I[" + std::to_string(row) + "," +
               std::to_string(col) + "]";
    }

    void check_bounds(int row, int col) const {
        if (row < 0 || row >= n_ || col < 0 || col >= n_) {
            throw std::out_of_range(
                "Node (" + std::to_string(row) + "," +
                std::to_string(col) + ") out of range");
        }
    }

    int node_index_by_id(const std::string& id) const {
        auto it = id_to_index_.find(id);
        if (it == id_to_index_.end()) {
            throw std::runtime_error("Unknown node id: " + id);
        }

        return it->second;
    }

    void wire_neighbors() {
        id_to_index_.clear();

        for (int r = 0; r < n_; ++r) {
            for (int c = 0; c < n_; ++c) {
                id_to_index_[make_id(r, c)] = index(r, c);
            }
        }

        for (int r = 0; r < n_; ++r) {
            for (int c = 0; c < n_; ++c) {
                Node& nd = nodes_[index(r, c)];
                nd.neighbors.clear();

                if (r > 0) {
                    nd.neighbors.push_back(
                        {make_id(r - 1, c), Direction::NORTH});
                }

                if (r < n_ - 1) {
                    nd.neighbors.push_back(
                        {make_id(r + 1, c), Direction::SOUTH});
                }

                if (c > 0) {
                    nd.neighbors.push_back(
                        {make_id(r, c - 1), Direction::WEST});
                }

                if (c < n_ - 1) {
                    nd.neighbors.push_back(
                        {make_id(r, c + 1), Direction::EAST});
                }
            }
        }
    }

    static std::string phase_short_name(SignalPhase phase) {
        switch (phase) {
            case SignalPhase::NORTH_SOUTH_GREEN:
                return "NS_G";
            case SignalPhase::NORTH_SOUTH_YELLOW:
                return "NS_Y";
            case SignalPhase::EAST_WEST_GREEN:
                return "EW_G";
            case SignalPhase::EAST_WEST_YELLOW:
                return "EW_Y";
            case SignalPhase::PEDESTRIAN_CROSSING:
                return "PED";
            default:
                return "UNK";
        }
    }

private:
    int n_;

    std::vector<Node> nodes_;
    std::vector<NodeMailbox> mailboxes_;
    std::vector<MessageBuffer> buffers_;

    std::unordered_map<std::string, int> id_to_index_;

    double batch_interval_seconds_;
    bool batching_enabled_;

    // Epoch-parallel control.
    std::atomic<bool> epoch_stop_flag_{false};
    std::atomic<int> epoch_generation_{0};
    std::mutex epoch_mtx_;
    std::condition_variable epoch_cv_;

    std::atomic<int> epoch_done_count_{0};
    std::mutex epoch_done_mtx_;
    std::condition_variable epoch_done_cv_;

    double epoch_current_time_ = 0.0;
    double epoch_delta_time_ = 1.0;

    // Fully async control.
    std::atomic<bool> async_stop_flag_{false};

    // Runtime stats.
    long long last_async_wall_time_us_ = 0;
    std::vector<std::atomic<long long>> async_node_runtime_us_;
    std::vector<std::atomic<int>> async_node_steps_;
};

#endif // NXN_MESH_NETWORK_H
