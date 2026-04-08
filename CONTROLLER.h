#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Intersection.h"

class Controller {
public:
    Controller();

    void update(Intersection& intersection, double current_time);

private:
    double compute_NS_demand(const IntersectionState& state);
    double compute_EW_demand(const IntersectionState& state);

    double compute_cycle_length(double total_demand);
};

#endif