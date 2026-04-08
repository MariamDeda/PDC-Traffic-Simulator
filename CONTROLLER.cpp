#include "CONTROLLER.h"

Controller::Controller() {}


double Controller::compute_NS_demand(const IntersectionState& state) {
    return state.queue_lengths.at(Direction::NORTH).at("total") +
           state.queue_lengths.at(Direction::SOUTH).at("total");
}

double Controller::compute_EW_demand(const IntersectionState& state) {
    return state.queue_lengths.at(Direction::EAST).at("total") +
           state.queue_lengths.at(Direction::WEST).at("total");
}


double Controller::compute_cycle_length(double total_demand) {
    const double LOST_TIME = 8.0;

    double Y = total_demand / (total_demand + 20.0);

    double C = (1.5 * LOST_TIME + 5.0) / (1.0 - Y);

    if (C < 30.0) C = 30.0;
    if (C > 120.0) C = 120.0;

    return C;
}



void Controller::update(Intersection& intersection, double current_time) {

    IntersectionState state = intersection.get_state();

    double NS = compute_NS_demand(state);
    double EW = compute_EW_demand(state);

    double total = NS + EW;

    if (total < 1.0) {
        intersection.set_phase(SignalPhase::NORTH_SOUTH_GREEN, 15.0, current_time);
        return;
    }

    double cycle = compute_cycle_length(total);

    double NS_ratio = NS / total;
    double EW_ratio = EW / total;

    double NS_green = NS_ratio * cycle;
    double EW_green = EW_ratio * cycle;

    const double MIN_GREEN = 15.0;
    const double MAX_GREEN = 60.0;

    if (NS_green < MIN_GREEN) NS_green = MIN_GREEN;
    if (EW_green < MIN_GREEN) EW_green = MIN_GREEN;

    if (NS_green > MAX_GREEN) NS_green = MAX_GREEN;
    if (EW_green > MAX_GREEN) EW_green = MAX_GREEN;

    if (NS >= EW) {
        intersection.set_phase(SignalPhase::NORTH_SOUTH_GREEN, NS_green, current_time);
    } else {
        intersection.set_phase(SignalPhase::EAST_WEST_GREEN, EW_green, current_time);
    }
}