#pragma once

#ifndef _EKF_H_
#define _EKF_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Eigen"
#include "mpu6050.hpp"

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

struct Euler
{
    float roll;
    float pitch;
    float yaw;
};

struct EKFConfig
{
    float procCovariance = 0.001;
    float rAcc = 0.08f;
    float rGyro = 0.0016f;

    float alpha = 0.9;
    float gyroBiasX = 0.023461514634847795f;
    float gyroBiasY = 0.032662799032722976f;
    float gyroBiasZ = 0.025085952195352697f;

// Means
// XDown
//   'gyr_x': 0.02595579941612678,
//   'gyr_y': 0.03492308890195085,
//   'gyr_z': 0.022853506204849463,
//   'acc_x': -0.9450034090308521,
//   'acc_y': 0.013869747430548647,
//   'acc_z': 0.23397593437967767

// XUP
//   'gyr_x': 0.023461514634847795,
//   'gyr_y': 0.032662799032722976,
//   'gyr_z': 0.025085952195352697,
//   'acc_x': 1.0438745300177898,
//   'acc_y': 0.011133938508400583,
//   'acc_z': 0.25662393276472034

// Variances
// XUDown
//   'acc_x': 0.009505706663229536,
//   'acc_y': 0.0032798450455657044,
//   'acc_z': 0.005372362435752023
//   'gyr_x': 0.0013098222601957874,
//   'gyr_y': 0.0014154144943249018,
//   'gyr_z': 0.0016240791469711725

// XUp
//    gyr_x': 0.0013179880222443353,
//   'gyr_y': 0.0014056273923198912,
//   'gyr_z': 0.0016237815300458353,
//   'acc_x': 0.003274304716148571,
//   'acc_y': 0.003276572697293971,
//   'acc_z': 0.005424007552667784
};

class EKF{
    public:
        EKF(I2C_HandleTypeDef* mpu, const EKFConfig& config);

        void StepEKFLoop(Quaternion& qOut);

        void QuaternionToEuler(const Quaternion& q, Euler& e);
        void EulerToQuaternion(const Euler& e, Quaternion& q);

    private:

        void h(const Eigen::Vector<float, 4>& qEst, Eigen::Vector<float, 3>& hOut);
        void H(const Eigen::Vector<float, 4>& qEst, Eigen::Matrix<float,3,4>& HOut);

        // First order filter for N channels
        template <uint32_t N>
        class Filter{
            public:
                Filter(){};

                void Initialize(float alpha, std::array<float, N> sensorBiases)
                {
                    m_alpha = alpha;
                    m_prevValues =std::array<float, N>{0.0f};
                    m_sensorBiases = sensorBiases;
                };

                std::array<float, N> Apply(std::array<float, N> values)
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

        // Process noise covariance
        Eigen::Matrix<float, 4, 4> m_Q;

        // Accelerometer and gyroscope measurement noise covariance
        Eigen::Matrix<float,3,3> m_RAcc;
        Eigen::Matrix<float,3,3> m_RGyro;

        // Current estimate for the attitude quaternion in form [w, x, y, z].T
        Eigen::Vector<float, 4> m_q;
        Eigen::Matrix<float, 4, 4> m_P;

        // Gravity. Convention: gravity along -x axis.
        Eigen::Vector<float, 3> m_g;

        I2C_HandleTypeDef* m_mpuHandle;

        // Loop variables
        uint32_t m_prevTick_ms;
        MPU6050_t m_mpu6050Data;
        Eigen::Matrix<float,4,4> m_omegaSkew;
        Eigen::Matrix<float,4,4> m_F;
        Eigen::Matrix<float,4,4> m_I4x4;

        Eigen::Matrix<float,3,4> m_H;

        Eigen::Matrix<float,4,3> m_K;

        Eigen::Matrix<float,3,3> m_S;

        // Predicted Accel measurement
        Eigen::Vector<float,3> m_h;

        // Actual Accel measurement
        Eigen::Vector<float,3> m_z;

        // Innovation (surprise w.r.t estimated measurement)
        Eigen::Vector<float,3> m_v;

        // Filter for gyroscope readings
        Filter<3u> m_filter;

        // Debug vars
        uint32_t ctr = 0;
        Eigen::Vector<float, 4> m_trueq;


        // TODO: Gyro and Accelerometer should be read off-step with one another
        // dt should be calculated based on the difference between when these samples were collected
        // dt is only used for the Prediction step, so it's important that it's accurate w.r.t. the time between gyro read and comparing prediction against accel measurement
        // dt should NOT be the time in between EKF runs!!!
};


#endif /* _EKF_H_ */