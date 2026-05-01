#ifndef NXN_MESH_NETWORK_H
#define NXN_MESH_NETWORK_H
#include "Intersection.h"
#include "CONTROLLER.h"
#include "DecentralizedController.h"
#include "TrafficGenerator.h"
#include "MessageBuffer.h"  // Added

#include <cassert>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class NxNMeshNetwork {
public:

    struct Node {
        int row, col;
        std::shared_ptr<Intersection>            intersection;
        std::shared_ptr<DecentralizedController> dc_controller;
        std::shared_ptr<TrafficGenerator>        generator;
        std::vector<NeighborLink>                neighbors; 
    };

    explicit NxNMeshNetwork(int n,
                            ScenarioType default_scenario = ScenarioType::NORMAL,
                            const TimingConstraints& constraints = TimingConstraints(),
                            double batch_interval = 5.0,      // NEW: batch every 5 seconds
                            bool enable_batching = true)      // NEW: enable by default
        : n_(n)
    {
        if (n < 1) throw std::invalid_argument("Grid size n must be >= 1");

        nodes_.resize(n * n);
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                int idx = index(r, c);
                Node& nd = nodes_[idx];
                nd.row = r;
                nd.col = c;

                std::string id = make_id(r, c);
                nd.intersection  = std::make_shared<Intersection>(id, constraints);
                nd.dc_controller = std::make_shared<DecentralizedController>(id);
                nd.generator     = std::make_shared<TrafficGenerator>(default_scenario);
            }
        }

        wire_neighbors();

        // Initialize message buffer for each node
        for (int i = 0; i < n * n; i++) {
            buffers_[i] = MessageBuffer(batch_interval_seconds_, 50);
        }
    }

    void enableBatching(bool enable, double interval_seconds = 5.0) {
        batching_enabled_ = enable;
        batch_interval_seconds_ = interval_seconds;
        
        // Reinitialize buffers with new interval
        for (int i = 0; i < n_ * n_; i++) {
            buffers_[i] = MessageBuffer(batch_interval_seconds_, 50);
        }
        
        std::cout << "[NxNMeshNetwork] Batching " << (enable ? "ENABLED" : "DISABLED") 
                << " (interval=" << interval_seconds << "s)\n";
    }

    void set_scenario(int row, int col, ScenarioType scenario) {
        check_bounds(row, col);
        nodes_[index(row, col)].generator =
            std::make_shared<TrafficGenerator>(scenario);
    }

    int n() const { return n_; }
    int size() const { return n_ * n_; }

    Node& node(int row, int col) {
        check_bounds(row, col);
        return nodes_[index(row, col)];
    }
    const Node& node(int row, int col) const {
        check_bounds(row, col);
        return nodes_[index(row, col)];
    }
    const std::vector<Node>& nodes() const { return nodes_; }

    std::string node_id(int row, int col) const { return make_id(row, col); }

    // Simulation CENTRAL
    void step_centralized(std::vector<Controller>& controllers,
                          double current_time,
                          double delta_time) {
        assert(static_cast<int>(controllers.size()) == n_ * n_);

        // traffic gen
        for (auto& nd : nodes_)
            nd.generator->update(*nd.intersection, current_time);

        // centrallized
        for (int i = 0; i < n_ * n_; ++i)
            controllers[i].update(*nodes_[i].intersection, current_time);

        // advance
        for (auto& nd : nodes_)
            nd.intersection->update(delta_time, current_time + delta_time);
    }

    // Simulation step — decentralized with message batching
    void step_decentralized(double current_time, double delta_time) {
        // ===== PHASE 1: Generate traffic =====
        for (auto& nd : nodes_)
            nd.generator->update(*nd.intersection, current_time);
        
        // ===== PHASE 2: Build local messages (always done) =====
        std::vector<NeighborMessage> msgs(n_ * n_);
        for (int i = 0; i < n_ * n_; ++i)
            msgs[i] = nodes_[i].dc_controller->build_local_message(
                        *nodes_[i].intersection, current_time);
        
        // ===== PHASE 3: Add messages to buffers OR send immediately =====
        if (batching_enabled_) {
            // WITH BATCHING: Add to buffers
            for (int i = 0; i < n_ * n_; ++i) {
                Node& nd = nodes_[i];
                for (const NeighborLink& link : nd.neighbors) {
                    int neighbor_idx = node_index_by_id(link.neighbor_id);
                    buffers_[i].addMessage(link.neighbor_id, msgs[neighbor_idx]);
                }
            }
            
            // Check if any buffer needs flushing
            bool should_flush = false;
            for (int i = 0; i < n_ * n_; ++i) {
                if (buffers_[i].shouldFlush(current_time)) {
                    should_flush = true;
                    break;
                }
            }
            
            if (should_flush) {
                // Flush all buffers and process batches
                for (int i = 0; i < n_ * n_; ++i) {
                    auto batches = buffers_[i].flush(current_time);
                    
                    // Process each batch (in real system, would send over network)
                    for (auto& batch_pair : batches) {
                        const std::string& neighbor_id = batch_pair.first;
                        const BatchedMessage& batch = batch_pair.second;
                        
                        // Find the node that receives this batch
                        int receiver_idx = node_index_by_id(neighbor_id);
                        
                        // Add batched messages to receiver's "inbox"
                        // For simulation, we process them immediately
                        for (const auto& msg : batch.messages) {
                            // Store in a temporary inbox for this step
                            // (Simplified - in real system, messages would arrive with delay)
                        }
                    }
                }
            }
        } else {
            // WITHOUT BATCHING: Send messages immediately (original M2 behavior)
            for (int i = 0; i < n_ * n_; ++i) {
                buffers_[i].sendImmediate("", NeighborMessage());  // Track stats
            }
        }
        
        // ===== PHASE 4: Decisions (using most recent messages) =====
        // Note: In batching mode, decisions use last received batch, not immediate messages
        // For simplicity, we'll use the messages we just built
        
        for (int i = 0; i < n_ * n_; ++i) {
            Node& nd = nodes_[i];
            
            // Build inbox from neighbors' messages (current or batched)
            std::vector<std::pair<NeighborLink, NeighborMessage>> inbox;
            inbox.reserve(nd.neighbors.size());
            
            for (const NeighborLink& link : nd.neighbors) {
                int nidx = node_index_by_id(link.neighbor_id);
                inbox.push_back({link, msgs[nidx]});
            }
            
            PhaseDecision dec = nd.dc_controller->decide(
                                    *nd.intersection, inbox, current_time);
            nd.dc_controller->apply_decision(*nd.intersection, dec, current_time);
        }
        
        // ===== PHASE 5: Advance simulation =====
        for (auto& nd : nodes_)
            nd.intersection->update(delta_time, current_time + delta_time);
    }

    // Add method to get batching statistics
    void printBatchingStats() const {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "  MESSAGE BATCHING STATISTICS\n";
        std::cout << std::string(60, '=') << "\n";
        
        for (int i = 0; i < n_ * n_; ++i) {
            std::cout << "\nNode " << nodes_[i].intersection->get_id() << ":\n";
            buffers_.at(i).printStats();
        }
    }

//stats
    struct NetworkStats {
        int    total_processed  = 0;
        double total_wait       = 0.0;
        double avg_wait         = 0.0;
        int    peak_queue       = 0;
        double throughput_vpm   = 0.0; // vehs per min
    };

    NetworkStats collect_stats(double sim_duration_seconds) const {
        NetworkStats s;
        for (const auto& nd : nodes_) {
            s.total_processed += nd.intersection->get_vehicles_processed();
            s.total_wait      += nd.intersection->get_total_wait_time();
        }
        s.avg_wait = s.total_processed > 0
                         ? s.total_wait / s.total_processed
                         : 0.0;
        s.throughput_vpm = s.total_processed / (sim_duration_seconds / 60.0);
        return s;
    }

    //print grid
    void print_grid(double current_time) const {
        std::cout << "\n  Grid snapshot @ t=" << std::fixed
                  << std::setprecision(1) << current_time << "s\n";
        std::cout << "  ";
        for (int c = 0; c < n_; ++c)
            std::cout << std::setw(10) << ("col" + std::to_string(c));
        std::cout << "\n";

        for (int r = 0; r < n_; ++r) {
            std::cout << "  row" << r;
            for (int c = 0; c < n_; ++c) {
                const auto& nd = nodes_[index(r, c)];
                auto st = nd.intersection->get_state();
                int q = st.queue_lengths.at(Direction::NORTH).at("total")
                      + st.queue_lengths.at(Direction::SOUTH).at("total")
                      + st.queue_lengths.at(Direction::EAST).at("total")
                      + st.queue_lengths.at(Direction::WEST).at("total");
                std::string phase_str =
                    (st.current_phase == SignalPhase::NORTH_SOUTH_GREEN) ? "NS_G" :
                    (st.current_phase == SignalPhase::EAST_WEST_GREEN)   ? "EW_G" :
                    (st.current_phase == SignalPhase::NORTH_SOUTH_YELLOW)? "NS_Y" :
                    (st.current_phase == SignalPhase::EAST_WEST_YELLOW)  ? "EW_Y" : "PED";
                std::ostringstream cell;
                cell << phase_str << "/Q" << q;
                std::cout << std::setw(10) << cell.str();
            }
            std::cout << "\n";
        }
    }

    // Deadline enforcement
    void enableDeadlineEnforcementForAll(bool enable, double deadline_ms = 50.0) {
        for (auto& nd : nodes_) {
            nd.dc_controller->enableDeadlineEnforcement(enable, deadline_ms);
        }
    }

    void printAllDeadlineStats() const {
        std::cout << "\n" << std::string(60, '=') << "\n";
        std::cout << "  DEADLINE ENFORCEMENT STATISTICS (per node)\n";
        std::cout << std::string(60, '=') << "\n";
        for (const auto& nd : nodes_) {
            std::cout << "\nNode " << nd.intersection->get_id() << ":\n";
            nd.dc_controller->printDeadlineStats();
        }
    }
    
private:
    int n_;
    std::vector<Node> nodes_;
    std::unordered_map<int, MessageBuffer> buffers_;  // One buffer per node
    double batch_interval_seconds_;
    bool batching_enabled_;

    // helper
    int index(int r, int c) const { return r * n_ + c; }

    static std::string make_id(int r, int c) {
        return "I[" + std::to_string(r) + "," + std::to_string(c) + "]";
    }

    void check_bounds(int r, int c) const {
        if (r < 0 || r >= n_ || c < 0 || c >= n_)
            throw std::out_of_range("Node (" + std::to_string(r) + "," +
                                    std::to_string(c) + ") out of range");
    }

    int node_index_by_id(const std::string& id) const {
        for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
            if (nodes_[i].intersection->get_id() == id)
                return i;
        throw std::runtime_error("Unknown node id: " + id);
    }

    // pair nodes as neighbours
    void wire_neighbors() {
        for (int r = 0; r < n_; ++r) {
            for (int c = 0; c < n_; ++c) {
                Node& nd = nodes_[index(r, c)];

                // NORTH neighbor (r-1, c)
                if (r > 0) {
                    nd.neighbors.push_back({make_id(r - 1, c), Direction::NORTH});
                }
                // SOUTH neighbor (r+1, c)
                if (r < n_ - 1) {
                    nd.neighbors.push_back({make_id(r + 1, c), Direction::SOUTH});
                }
                // WEST neighbor (r, c-1)
                if (c > 0) {
                    nd.neighbors.push_back({make_id(r, c - 1), Direction::WEST});
                }
                // EAST neighbor (r, c+1)
                if (c < n_ - 1) {
                    nd.neighbors.push_back({make_id(r, c + 1), Direction::EAST});
                }
            }
        }
    }
};

#endif // NXN_MESH_NETWORK_H
