#ifndef NXN_MESH_NETWORK_H
#define NXN_MESH_NETWORK_H

/*
 *  Coordinate layout (row 0 = top, col 0 = left):
 *
 *      col0   col1  ... col(N-1)
 *  row0  [0,0] [0,1] ... [0,N-1]
 *  row1  [1,0] [1,1] ...
 *  ...
 *  row(N-1)
 *
 *  Horizontal edges: [r,c] --EAST--> [r,c+1]  /  [r,c+1] --WEST--> [r,c]
 *  Vertical   edges: [r,c] --SOUTH-> [r+1,c]  /  [r+1,c] --NORTH-> [r,c]
 *
 * Each node owns an Intersection, a DecentralizedController, and a
 * TrafficGenerator (scenario is set per-node, defaults to NORMAL).
 *
 * The class exposes two step() overloads:
 *   step_centralized(controllers, delta, t)  — caller passes one Controller per node
 *   step_decentralized(delta, t)             — uses built-in DecentralizedControllers
 */

#include "Intersection.h"
#include "CONTROLLER.h"
#include "DecentralizedController.h"
#include "TrafficGenerator.h"

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
                            const TimingConstraints& constraints = TimingConstraints())
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

    // Simulation step — msg exchange decentral

    void step_decentralized(double current_time, double delta_time) {
        // traffic gen
        for (auto& nd : nodes_)
            nd.generator->update(*nd.intersection, current_time);

        // message
        std::vector<NeighborMessage> msgs(n_ * n_);
        for (int i = 0; i < n_ * n_; ++i)
            msgs[i] = nodes_[i].dc_controller->build_local_message(
                          *nodes_[i].intersection, current_time);

        // 
        for (int i = 0; i < n_ * n_; ++i) {
            Node& nd = nodes_[i];

            // neigbours recieve
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

        // move on
        for (auto& nd : nodes_)
            nd.intersection->update(delta_time, current_time + delta_time);
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

private:
    int n_;
    std::vector<Node> nodes_;

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
