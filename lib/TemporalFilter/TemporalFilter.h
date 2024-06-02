#ifndef _TEMP_FILTER_H_
#define _TEMP_FILTER_H_

#include <stdint.h>

// First order filter for N channels
template <typename T, uint32_t N>
class TemporalFilter{
    public:
        TemporalFilter(){};

        void Initialize(T alpha, const std::array<T, N>& sensorBiases)
        {
            m_alpha = alpha;
            m_prevValues = std::array<T, N>{0.0f};
            m_sensorBiases = sensorBiases;
        };

        // Modifies the input values in place
        void Apply(std::array<T, N>& values)
        {
            for (auto i = 0u; i < N; i++)
            {
                // Remove bias from the sensor reading
                const auto value = values[i] - m_sensorBiases[i];

                // Compute filtered value
                const auto filteredValue = m_alpha * value + (1.0f - m_alpha) * m_prevValues[i];

                // Update previous value
                m_prevValues[i] = value;

                // Populate array with filtered values
                values[i] = filteredValue;
            }
        };

    private:
        // Blending param, [1.0,0.0]   1 means unfiltered, 0.5 means equal weight to current and previous samples 
        T m_alpha;
        std::array<T, N> m_prevValues;

        // Calibration params
        std::array<T, N> m_sensorBiases;
};

# endif /* _TEMP_FILTER_H_ */