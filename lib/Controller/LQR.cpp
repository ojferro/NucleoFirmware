#include "LQR.h"


LQR::LQR(){

    // Filter for the torque output
    m_filter =  TemporalFilter<float, 1u>();
    m_filter.Initialize(0.9f, {0.0f});

    // LQR gains
    // m_K = {-100, 371.94740521, -84.40802539, 117.13209978};
    // m_K = {-10.0 , 105.74833494, -13.03282748,  32.47217271};
    // m_K = {-3.16227766, 60.99674706, -5.29600073, 18.37821378};
    // m_K = {-3.16227766, 42.12537449, -4.50120282, 12.35963449};

    // all of them positive
    // m_K = {3.16227766, 42.12537449, 4.50120282, 12.35963449};
    // m_K = {1.0, 31.54632155, 2.06466836, 9.02291656};
    // m_K = {1.0, 31.54632155, 2.06466836, 9.02291656};


    // Sort of working
    // m_K = {0.31622777, 26.75087242, 1.02286157,  7.49577499};

    // This one works almost perfectly!
    // m_K = {0.4472136 , 24.84478241, 1.13187714,  3.78118461};

    // This one works great! The cable pulling it makes it not want to stay at x=0, but it stabilizes at another nonzero point.
    m_K = {0.70710678, 25.75388822,  1.46730009,  3.93888074};
    

}
void LQR::iterate(U& u, const State& state){
    
    float totalForce = 0.f;

    for (uint8_t i = 0; i < state.size(); i++)
        totalForce += m_K[i] * state[i];

    const float wheelRadius = 0.03f; // meters
    float totalTorque = totalForce * wheelRadius;

    // Filter the torque output
    auto t = std::array<float, 1u>{totalTorque};
    m_filter.Apply(t);

    totalTorque = t[0];

    // Send half the torque to each wheel
    u[0] = totalTorque * 0.5f;
    u[1] = totalTorque * 0.5f;
}