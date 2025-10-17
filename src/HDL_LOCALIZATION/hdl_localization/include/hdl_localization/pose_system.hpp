#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y,
 * gyro_bias_z]
 */
class PoseSystem {
 public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;

 public:
  PoseSystem() { dt = 0.01f; }  // 初始化值，实际由PoseEstimator类中传入IMU时间戳

  // system equation (without input)
  VectorXt f(const VectorXt& state) const {
    VectorXt next_state(16);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);

    // position
    next_state.middleRows(0, 3) = pt + vt * dt;  //

    // velocity
    next_state.middleRows(3, 3) = vt;

    // orientation
    Quaterniont qt_ = qt;

    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity

    return next_state;
  }

  // system equation 系统状态方程
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);

    Vector3t pt = state.middleRows(0, 3);                    // 位置
    Vector3t vt = state.middleRows(3, 3);                    // 速度
    Quaterniont qt(state[6], state[7], state[8], state[9]);  // 四元数
    qt.normalize();                                          // 归一化四元数

    Vector3t acc_bias = state.middleRows(10, 3);   // 加速度偏差
    Vector3t gyro_bias = state.middleRows(13, 3);  // 陀螺仪偏差

    Vector3t raw_acc = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);

    // position 更新下一时刻位置状态
    next_state.middleRows(0, 3) = pt + vt * dt;  //

    // velocity 更新速度，是否利用加速度矫正速度，加速度噪声大及静止重力大小无法一致
    Vector3t g(0.0f, 0.0f, 9.81f);
    Vector3t acc_ = raw_acc - acc_bias;
    Vector3t acc = qt * acc_;
    // acceleration didn't contribute to accuracy due to large noise
    // next_state.middleRows(3, 3) = vt;  // + (acc - g) * dt;
    next_state.middleRows(3, 3) = vt + (acc - g) * dt;  // todo test

    // orientation 更新四元数
    Vector3t gyro = raw_gyro - gyro_bias;
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);  // 构造角速度四元数
    dq.normalize();                                                           // 四元数归一化
    Quaterniont qt_ = (qt * dq).normalized();
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();  // 更新四元数状态矩阵

    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity

    return next_state;
  }

  // observation equation 观测方程 将当前状态量作为观测量
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
};

}  // namespace hdl_localization

#endif  // POSE_SYSTEM_HPP
