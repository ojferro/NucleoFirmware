#ifndef _FILTER_H_
#define _FILTER_H_

#include <stdint.h>

// First order filter for N channels
template <uint32_t N>
class Filter{
    public:
        Filter(){};

        void Initialize(float alpha, const std::array<float, N>& sensorBiases)
        {
            m_alpha = alpha;
            m_prevValues = std::array<float, N>{0.0f};
            m_sensorBiases = sensorBiases;
        };

        std::array<float, N> Apply(const std::array<float, N>& values)
        {
            auto filteredValues = std::array<float, N>{};
            for (auto i = 0u; i < N; i++)
            {
                // Remove bias from the sensor reading
                const auto value = values[i] - m_sensorBiases[i];

                const auto filteredValue = m_alpha * value + (1.0f - m_alpha) * m_prevValues[i];
                m_prevValues[i] = value;

                filteredValues[i] = filteredValue;
            }

            return filteredValues;
        };

    private:
        // Blending param, [1.0,0.0]   1 means unfiltered, 0.5 means equal weight to current and previous samples 
        float m_alpha;
        std::array<float, N> m_prevValues;

        // Calibration params
        std::array<float, N> m_sensorBiases;
};

# endif /* _FILTER_H_ */