#pragma once

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Eigen"

class Controller{
    public:
        // Controller output. {torque0, torque1}
        using U = std::array<float,2>;

        // Observed state. {x, theta, x_dot, theta_dot}
        using State = std::array<float,4>;

        enum StateIndex{
            X = 0,
            THETA = 1,
            X_DOT = 2,
            THETA_DOT = 3
        };

        U u;
        State state;

        virtual void iterate(U& u, const State& state);

};

#endif /* _CONTROLLER_H_ */