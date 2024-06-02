#pragma once

#ifndef _LQR_H_
#define _LQR_H_

#include "Controller.h"
#include "Eigen"
#include "TemporalFilter.h"

class LQR : public Controller{
    public:
        LQR();
        void iterate(U& u, const State& state) override;

    private:
        TemporalFilter<float, 1u> m_filter; // One channel filter for the torque output
        std::array<float,4> m_K;
};
#endif /* _LQR_H_ */