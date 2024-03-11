
// #include <stdint.h>
#include "EKF.h"
#include "Logger.h"
#include "mpu6050.hpp"
#include <cmath>


namespace {
    void ConstructSkewMatrix(float omega_x, float omega_y, float omega_z, Eigen::Matrix<float,4,4>& skewOut)
    {
        skewOut << 0, -omega_x, -omega_y, -omega_z,
                omega_x, 0, omega_z, -omega_y,
                omega_y, -omega_z, 0, omega_x,
                omega_z, omega_y, -omega_x, 0;
    }

    // TODO: Modify this matrix in-place to avoid reallocations.
    Eigen::Matrix<float,3,3> QuaternionToRotMatrix(const Eigen::Vector<float, 4>& q)
    {
        Eigen::Matrix<float,3,3> rotMat;

        const float w = q[0];
        const float x = q[1];
        const float y = q[2];
        const float z = q[3];

        rotMat << 1 - 2*(y*y - z*z), 2*(x*y - w*z), 2*(x*z + w*y),
                  2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x,
                  2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y;

        return rotMat;
    }

    void ReadGyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
    {
        MPU6050_Read_Gyro(I2Cx, DataStruct);

        // Remap to X up, Y fwd, Z left
        const auto x = DataStruct->Gx;
        const auto z = DataStruct->Gz;
        DataStruct->Gx = z;
        DataStruct->Gz = -x;
    }

    void ReadAccel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
    {
        MPU6050_Read_Accel(I2Cx, DataStruct);

        // Remap to X up, Y fwd, Z left
        const auto x = DataStruct->Ax;
        const auto z = DataStruct->Az;
        DataStruct->Ax = z;
        DataStruct->Az = -x;
    }
}

EKF::EKF(I2C_HandleTypeDef* mpu, const EKFConfig& config){
    // Process covariance
    m_Q.setIdentity();
    m_Q = m_Q * config.procCovariance;

    // Accelerometer and Gyro noise
    m_RAcc.setIdentity();
    m_RAcc = m_RAcc * config.rAcc;

    m_RGyro.setIdentity();
    m_RGyro = m_RGyro * config.rGyro;

    // Initialize state estimate and covariance
    m_q = Eigen::Vector<float, 4>(1.0f, 0.0f, 0.0f, 0.0f);
    m_P = Eigen::Matrix4f::Identity();

    m_g = Eigen::Vector<float, 3>(0.0f, 0.0f, -1.0f);

    m_mpuHandle = mpu;

    m_I4x4.setIdentity();

    // First order 3-channel (gyro x,y,z) filter 
    const auto biases = std::array<float,3>{config.gyroBiasX, config.gyroBiasY, config.gyroBiasZ};
    m_filter = Filter<3u>();
    m_filter.Initialize(config.alpha, biases);

    // Debug vars
    ctr = 0;
    m_trueq = Eigen::Vector<float, 4>(1.0f, 0.0f, 0.0f, 0.0f);
}

void EKF::QuaternionToEuler(const Quaternion& q, Euler& e)
{
    // Extract quaternion components
    const float qw = q.w;
    const float qx = q.x;
    const float qy = q.y;
    const float qz = q.z;

    // Roll (x-axis rotation)
    const auto sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const auto cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);

    const auto roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const auto sinp = 2.0f * (qw * qy - qz * qx);
    auto pitch = 0.0f;
    if (abs(sinp) >= 1)
        pitch = (M_PI / 2.0) * (sinp > 0.0f ? 1.0f : -1.0f); // Use +-pi/2 if out of range
    else
        pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    const auto siny_cosp = 2.0f * (qw * qz + qx * qy);
    const auto cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    const auto yaw = atan2f(siny_cosp, cosy_cosp);

    // Assign the output variable
    e.roll = roll;
    e.pitch = pitch;
    e.yaw = yaw;
}

void EKF::EulerToQuaternion(const Euler& e, Quaternion& q)
{
    const auto cy = cos(e.yaw * 0.5);
    const auto sy = sin(e.yaw * 0.5);
    const auto cp = cos(e.pitch * 0.5);
    const auto sp = sin(e.pitch * 0.5);
    const auto cr = cos(e.roll * 0.5);
    const auto sr = sin(e.roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

void EKF::h(const Eigen::Vector<float, 4>& qEst, Eigen::Vector<float, 3>& hOut){
    auto rotMat = QuaternionToRotMatrix(qEst);
    rotMat.transposeInPlace();
    hOut = rotMat * m_g;
}

void EKF::H(const Eigen::Vector<float, 4>& qEst, Eigen::Matrix<float,3,4>& HOut)
{
    const auto g_x = m_g[0];
    const auto g_y = m_g[1];
    const auto g_z = m_g[2];

    const auto q_w = qEst[0];
    const auto q_x = qEst[1];
    const auto q_y = qEst[2];
    const auto q_z = qEst[3];

    HOut << g_x*q_w + g_y*q_z - g_z*q_y , g_x*q_x + g_y*q_y + g_z*q_z , -g_x*q_y + g_y*q_x - g_z*q_w , -g_x*q_z + g_y*q_w + g_z*q_x,
            -g_x*q_z + g_y*q_w + g_z*q_x , g_x*q_y - g_y*q_x + g_z*q_w ,  g_x*q_x + g_y*q_y + g_z*q_z , -g_x*q_w - g_y*q_z + g_z*q_y,
            g_x*q_y - g_y*q_x + g_z*q_w , g_x*q_z - g_y*q_w - g_z*q_x ,  g_x*q_w + g_y*q_z - g_z*q_y ,  g_x*q_x + g_y*q_y + g_z*q_z;

    HOut = 2 * HOut;
}

void EKF::StepEKFLoop(Quaternion& qOut){

    const auto tick_ms = HAL_GetTick();
    auto dt_s = (tick_ms - m_prevTick_ms)/1000.0f;

    m_prevTick_ms = tick_ms;

    if (dt_s > 0.1)
    {
        debugLogFmt("WARNING: EKF iteration time too long. dt=%.3fs. Integration not accurate. Using dt=0.01s.");
        dt_s = 0.01;
    }

    // const auto dt_s = 0.01;

    /////// PREDICTION STEP ///////
    // Get angular velocity wx, wy, wz
    ReadGyro(m_mpuHandle, &m_mpu6050Data);

    const auto filteredGyro = m_filter.Apply({
        static_cast<float>(m_mpu6050Data.Gx),
        static_cast<float>(m_mpu6050Data.Gy),
        static_cast<float>(m_mpu6050Data.Gz)
    });

    m_mpu6050Data.Gx = filteredGyro[0];
    m_mpu6050Data.Gy = filteredGyro[1];
    m_mpu6050Data.Gz = filteredGyro[2];

    //////////////////////////////////////
    //////////////////////////////////////
    // Debugging only: Mock out gyro data:
    // ctr++;
    // m_mpu6050Data.Gx = 4*sin(ctr/15.0f);
    // m_mpu6050Data.Gy = 4*cos(ctr/10.0);
    // m_mpu6050Data.Gz = 4*sin(ctr/7.777);

    debugLogFmt("gyr_x:%.16f\n", m_mpu6050Data.Gx);
    debugLogFmt("gyr_y:%.16f\n", m_mpu6050Data.Gy);
    debugLogFmt("gyr_z:%.16f\n", m_mpu6050Data.Gz);

    // ConstructSkewMatrix(m_mpu6050Data.Gx, m_mpu6050Data.Gy, m_mpu6050Data.Gz, m_omegaSkew);

    // m_trueq = m_trueq + 0.5 * dt_s * m_omegaSkew * m_trueq;
    // m_trueq.normalize();
    //////////////////////////////////////
    //////////////////////////////////////

    ConstructSkewMatrix(m_mpu6050Data.Gx, m_mpu6050Data.Gy, m_mpu6050Data.Gz, m_omegaSkew);

    // Integrate velocity to current quaternion estimate
    m_q = m_q + 0.5 * dt_s * m_omegaSkew * m_q;
    m_q.normalize();

    m_F = m_I4x4 + (0.5 * dt_s * m_omegaSkew);
    m_P = m_F * m_P * m_F.transpose() + m_Q; // TODO: Maybe add the covariance depending on w as in https://ahrs.readthedocs.io/en/latest/filters/ekf.html

    ////// MEASUREMENT UPDATE STEP //////
    // Get linear acceleration x, y, z
    ReadAccel(m_mpuHandle, &m_mpu6050Data);

    //////////////////////////////////////
    //////////////////////////////////////
    // const auto rotMat = QuaternionToRotMatrix(m_trueq);
    // const Eigen::Vector3f gravInBodyFrame = rotMat.transpose() * m_g;
    // // Debugging only: Mock out accel data:
    // m_mpu6050Data.Ax = gravInBodyFrame[0];
    // m_mpu6050Data.Ay = gravInBodyFrame[1];
    // m_mpu6050Data.Az = gravInBodyFrame[2];

    // debugLogFmt("acc_x:%.16f\n", m_mpu6050Data.Ax);
    // debugLogFmt("acc_y:%.16f\n", m_mpu6050Data.Ay);
    // debugLogFmt("acc_z:%.16f\n", m_mpu6050Data.Az);
    //////////////////////////////////////
    //////////////////////////////////////

    m_z << m_mpu6050Data.Ax, m_mpu6050Data.Ay, m_mpu6050Data.Az;
    m_z.normalize();

    debugLogFmt("acc_x:%.16f\n", m_z[0]);
    debugLogFmt("acc_y:%.16f\n", m_z[1]);
    debugLogFmt("acc_z:%.16f\n", m_z[2]);

    // Compute estimated accelerometer measurement
    h(m_q, m_h);

    // Calculate Innovation
    m_v = m_z - m_h;

    // Get Jacobian to propagate covariance
    H(m_q, m_H);

    m_S = m_H * m_P * m_H.transpose() + m_RAcc;

    // Kalman gain
    m_K =  m_P * m_H.transpose() * m_S.inverse();

    // Update the state with the latest data
    m_q = m_q + m_K * m_v;
    m_q.normalize();

    // Update covariance matrix
    m_P = (m_I4x4 - (m_K * m_H)) * m_P;


    // Update output variable
    qOut.w = m_q[0];
    qOut.x = m_q[1];
    qOut.y = m_q[2];
    qOut.z = m_q[3];
    
    // Remove Yaw
    {
        auto e = Euler{};
        QuaternionToEuler(qOut, e);
        e.yaw = 0.0f;
        EulerToQuaternion(e, qOut);
    }
}

// void stepEKFLoop()
