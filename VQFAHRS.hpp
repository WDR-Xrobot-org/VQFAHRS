#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 基于 VQF 算法的六轴姿态和航向参考系统 (AHRS) 模块 / 6-axis Attitude and Heading Reference System (AHRS) Module based on VQF
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "libxr.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "transform.hpp"

/**
 * @brief 基于 VQF 算法的六轴 IMU 姿态解算模块，发布四元数和欧拉角。
 *        6-axis IMU attitude estimation module based on VQF (Versatile Quaternion-based
 *        Filter); publishes the quaternion and Euler angles.
 *
 *        Reference: D. Laidig and T. Seel, "VQF: Highly Accurate IMU Orientation
 *        Estimation with Bias Estimation and Magnetic Disturbance Rejection",
 *        Information Fusion 2023, 91, 187-204. https://github.com/dlaidig/vqf (MIT License)
 */
class VQFAHRS
{
 public:
  /**
   * @brief 构造参数。
   *        Construction parameters.
   */
  struct Param
  {
    const char* gyro_topic_name;            ///< 陀螺仪 Topic 名称 Gyroscope Topic name
    const char* accl_topic_name;            ///< 加速度计 Topic 名称 Accelerometer Topic name
    const char* quaternion_topic_name;      ///< 四元数 Topic 名称 Quaternion Topic name
    const char* euler_topic_name;           ///< 欧拉角 Topic 名称 Euler angle Topic name
    bool accl_in_g;                         ///< 加速度计单位是否为 g（true: g，false: m/s^2）
    float tau_acc;                          ///< 加速度计低通滤波时间常数 [s]
    uint32_t task_stack_depth;              ///< 线程栈深 Thread stack depth
    const char* command_name = "ahrs";      ///< RamFS 命令名，同一目录下须唯一。
  };

  /**
   * @brief 构造 VQFAHRS，创建发布 Topic，注册 RamFS 命令并创建 `ahrs` 线程。
   *        Construct VQFAHRS, create the published Topics, register the RamFS
   *        command and create the `ahrs` thread.
   *
   *        其余 VQF 算法参数（零偏估计、静止检测等）为模块内部编译期常量，
   *        取值见私有常量 kBias* / kRest*。
   *        Remaining VQF algorithm parameters (bias estimation, rest detection,
   *        etc.) are compile-time constants inside the module; see the private
   *        kBias* / kRest* constants for their values.
   *
   * @param ramfs 接收模块命令的 RamFS。
   *              RamFS that receives the module command.
   * @param param 构造参数。
   *              Construction parameters.
   */
  VQFAHRS(LibXR::RamFS& ramfs,
          const Param& param = {.gyro_topic_name = "imu_gyro",
                                .accl_topic_name = "imu_accl",
                                .quaternion_topic_name = "ahrs_quaternion",
                                .euler_topic_name = "ahrs_euler",
                                .accl_in_g = true,
                                .tau_acc = 3.0f,
                                .task_stack_depth = 2048,
                                .command_name = "ahrs"})
      : param_(param),
        gyro_topic_name_(param.gyro_topic_name),
        accl_topic_name_(param.accl_topic_name),
        quaternion_topic_(LibXR::Topic::CreateTopic<decltype(quaternion_)>(
            param.quaternion_topic_name)),
        euler_topic_(
            LibXR::Topic::CreateTopic<decltype(euler_)>(param.euler_topic_name)),
        cmd_file_(LibXR::RamFS::CreateFile(param.command_name, CommandFunc, this))
  {
    gyr_ts_ = 0.001f;
    acc_ts_ = gyr_ts_;
    gyr_ts_locked_ = false;
    acc_ts_locked_ = false;
    Setup();

    ramfs.Add(cmd_file_);

    thread_.Create(this, ThreadFunc, "ahrs", param.task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  /**
   * @brief 监控回调：四元数含有 NaN 或 Inf 时输出告警。
   *        Monitor callback: log a warning when the quaternion contains NaN or Inf.
   */
  void OnMonitor()
  {
    if (std::isinf(quaternion_.x()) || std::isinf(quaternion_.y()) ||
        std::isinf(quaternion_.z()) || std::isinf(quaternion_.w()) ||
        std::isnan(quaternion_.x()) || std::isnan(quaternion_.y()) ||
        std::isnan(quaternion_.z()) || std::isnan(quaternion_.w()))
    {
      XR_LOG_WARN("AHRS: NaN data detected\r\n");
    }
  }

  /**
   * @brief 线程函数：每收到一条陀螺仪消息更新一次姿态并发布结果，
   *        同时异步接收加速度计消息（原生支持同频与异频采样）。
   *        Thread function: update the attitude on every gyroscope message and publish
   *        the result, while receiving accelerometer messages asynchronously.
   *
   * @param ahrs VQFAHRS 实例。
   *             VQFAHRS instance.
   */
  static void ThreadFunc(VQFAHRS* ahrs)
  {
    LibXR::Topic::SyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        ahrs->gyro_topic_name_, ahrs->gyro_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> accl_suber(
        ahrs->accl_topic_name_);

    accl_suber.StartWaiting();

    while (true)
    {
      gyro_suber.Wait();
      const auto imu_timestamp = gyro_suber.GetTimestamp();

      bool has_new_accl = false;
      LibXR::MicrosecondTimestamp accl_timestamp = imu_timestamp;
      if (accl_suber.Available())
      {
        ahrs->accl_ = accl_suber.GetData();
        accl_timestamp = accl_suber.GetTimestamp();
        accl_suber.StartWaiting();
        has_new_accl = true;
      }

      ahrs->Update(imu_timestamp, has_new_accl, accl_timestamp);

      ahrs->quaternion_topic_.Publish(ahrs->quaternion_, imu_timestamp);
      ahrs->euler_topic_.Publish(ahrs->euler_, imu_timestamp);
    }
  }

  /**
   * @brief 使用最近的陀螺仪与加速度计数据更新四元数和欧拉角（兼容 MadgwickAHRS 接口）。
   *        Update the quaternion and Euler angles from the latest gyroscope and
   *        accelerometer data.
   *
   * @param imu_timestamp 陀螺仪消息时间戳，相邻两次之差为 dt，第一次 dt 为 0。
   * @param has_new_accl 本次是否收到新的加速度计数据（默认 true）。
   * @param accl_timestamp 加速度计消息时间戳（默认与陀螺仪时间戳一致）。
   */
  void Update(LibXR::MicrosecondTimestamp imu_timestamp, bool has_new_accl = true,
              LibXR::MicrosecondTimestamp accl_timestamp = 0)
  {
    if (this->time_initialized_)
    {
      this->dt_ = (imu_timestamp - this->last_time_).ToSecondf();
    }
    else
    {
      this->dt_ = 0.0f;
      this->time_initialized_ = true;
    }
    this->last_time_ = imu_timestamp;

    if (this->dt_ > 0.0f && this->dt_ < 0.5f)
    {
      if (!gyr_ts_locked_)
      {
        gyr_ts_ = this->dt_;
        gyr_ts_locked_ = true;
        if (!acc_ts_locked_)
        {
          acc_ts_ = this->dt_;
        }
        Setup();
      }
      else if (std::fabs(this->dt_ - gyr_ts_) > gyr_ts_ * 0.1f)
      {
        AdaptGyrTs(this->dt_);
      }

      const float gyr[3] = {this->gyro_.x(), this->gyro_.y(), this->gyro_.z()};
      UpdateGyr(gyr, this->dt_);
    }

    if (has_new_accl)
    {
      LibXR::MicrosecondTimestamp eff_acc_ts =
          (accl_timestamp != 0) ? accl_timestamp : imu_timestamp;
      float acc_dt = 0.0f;
      if (this->acc_time_initialized_)
      {
        acc_dt = (eff_acc_ts - this->last_acc_time_).ToSecondf();
      }
      else
      {
        this->acc_time_initialized_ = true;
      }
      this->last_acc_time_ = eff_acc_ts;

      if (acc_dt > 0.0f && acc_dt < 0.5f)
      {
        if (!acc_ts_locked_)
        {
          acc_ts_ = acc_dt;
          acc_ts_locked_ = true;
          Setup();
        }
        else if (std::fabs(acc_dt - acc_ts_) > acc_ts_ * 0.1f)
        {
          AdaptAccTs(acc_dt);
        }
      }

      float scale = param_.accl_in_g ? kGravity : 1.0f;
      const float acc[3] = {this->accl_.x() * scale, this->accl_.y() * scale,
                            this->accl_.z() * scale};
      if (!(acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f))
      {
        UpdateAcc(acc);
      }
    }

    float q6d[4];
    GetQuat6D(q6d);
    this->quaternion_.w() = q6d[0];
    this->quaternion_.x() = q6d[1];
    this->quaternion_.y() = q6d[2];
    this->quaternion_.z() = q6d[3];
    this->euler_ = this->quaternion_.ToEulerAngle();
  }

  /**
   * @brief 重置滤波器内部状态，与重新初始化实例等价。
   *        Reset the internal filter state.
   */
  void ResetState()
  {
    QuatSetToIdentity(gyr_quat_);
    QuatSetToIdentity(acc_quat_);

    last_acc_lp_[0] = 0.0f;
    last_acc_lp_[1] = 0.0f;
    last_acc_lp_[2] = 0.0f;
    acc_lp_state_.fill(kNaN);
    last_acc_corr_angular_rate_ = 0.0f;

    rest_detected_ = false;
    rest_t_ = 0.0f;
    rest_last_gyr_lp_[0] = 0.0f;
    rest_last_gyr_lp_[1] = 0.0f;
    rest_last_gyr_lp_[2] = 0.0f;
    rest_gyr_lp_state_.fill(kNaN);
    rest_last_acc_lp_[0] = 0.0f;
    rest_last_acc_lp_[1] = 0.0f;
    rest_last_acc_lp_[2] = 0.0f;
    rest_acc_lp_state_.fill(kNaN);
    rest_last_squared_dev_[0] = 0.0f;
    rest_last_squared_dev_[1] = 0.0f;

    bias_[0] = 0.0f;
    bias_[1] = 0.0f;
    bias_[2] = 0.0f;
    Matrix3SetToScaledIdentity(bias_p0_, bias_p_);
    motion_bias_est_r_lp_state_.fill(kNaN);
    motion_bias_est_bias_lp_state_.fill(kNaN);
  }

  /**
   * @brief 获取当前 6D 姿态四元数 [w, x, y, z]。
   */
  void GetQuat6D(float out[4]) const { QuatMultiply(acc_quat_, gyr_quat_, out); }

  /**
   * @brief 获取当前陀螺仪零偏估计值（rad/s）及其不确定度标准差（rad/s）。
   */
  float GetBiasEstimate(float out[3]) const
  {
    if (out != nullptr)
    {
      out[0] = bias_[0];
      out[1] = bias_[1];
      out[2] = bias_[2];
    }
    // Upper bound on largest eigenvalue of covariance matrix (trace of P)
    // Scaled back from 0.01 deg/s units to rad/s
    float p_trace = bias_p_[0] + bias_p_[4] + bias_p_[8];
    float sigma_deg_s = std::sqrt(std::max(p_trace, 0.0f)) * 0.01f;
    return sigma_deg_s * (kPi / 180.0f);
  }

  /**
   * @brief 获取当前是否处于静止状态。
   */
  bool GetRestDetected() const { return rest_detected_; }

  /**
   * @brief 获取静止检测相对偏差（均小于 1 时计入静止时间）。
   */
  void GetRelativeRestDeviations(float out[2]) const
  {
    float gyr_th = kRestThGyr * (kPi / 180.0f);
    out[0] = std::sqrt(rest_last_squared_dev_[0]) / std::max(gyr_th, kEps);
    out[1] = std::sqrt(rest_last_squared_dev_[1]) / std::max(kRestThAcc, kEps);
  }

 private:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kSqrt2 = 1.41421356237309504880f;
  static constexpr float kGravity = 9.80665f;
  static constexpr float kEps = 1.1920929e-7f;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // VQF 算法参数（编译期常量，取值为原 Param 默认值，不对外配置）。
  // VQF algorithm parameters as compile-time constants (former Param defaults).
  static constexpr bool kMotionBiasEstEnabled = true;   ///< 运动阶段零偏估计
  static constexpr bool kRestBiasEstEnabled = true;     ///< 静止检测与静止零偏估计
  static constexpr float kBiasSigmaInit = 0.5f;         ///< 初始零偏不确定度 [deg/s]
  static constexpr float kBiasForgettingTime = 100.0f;  ///< 零偏遗忘时间 [s]
  static constexpr float kBiasClip = 2.0f;              ///< 最大零偏 [deg/s]
  static constexpr float kBiasSigmaMotion = 0.1f;       ///< 运动阶段收敛零偏不确定度 [deg/s]
  static constexpr float kBiasVerticalForgettingFactor = 0.0001f;  ///< 垂直不可观遗忘因子
  static constexpr float kBiasSigmaRest = 0.03f;        ///< 静止阶段收敛零偏不确定度 [deg/s]
  static constexpr float kRestMinT = 1.5f;              ///< 判定静止的最短时间 [s]
  static constexpr float kRestFilterTau = 0.5f;         ///< 静止检测低通时间常数 [s]
  static constexpr float kRestThGyr = 2.0f;             ///< 静止检测陀螺仪阈值 [deg/s]
  static constexpr float kRestThAcc = 0.5f;             ///< 静止检测加速度计阈值 [m/s^2]

  static inline float Square(float x) { return x * x; }

  void Setup()
  {
    FilterCoeffs(param_.tau_acc, acc_ts_, acc_lp_b_, acc_lp_a_);
    FilterCoeffs(kRestFilterTau, gyr_ts_, rest_gyr_lp_b_, rest_gyr_lp_a_);
    FilterCoeffs(kRestFilterTau, acc_ts_, rest_acc_lp_b_, rest_acc_lp_a_);

    SetupBiasCoeffs();
    ResetState();
  }

  void SetupBiasCoeffs()
  {
    // Scaled by 100^2 (0.01 deg/s units) for numerical stability in single-precision float
    bias_p0_ = Square(kBiasSigmaInit * 100.0f);
    bias_v_ = Square(0.1f * 100.0f) * acc_ts_ / std::max(kBiasForgettingTime, 1e-6f);

    float p_motion = Square(kBiasSigmaMotion * 100.0f);
    bias_motion_w_ = Square(p_motion) / bias_v_ + p_motion;
    bias_vertical_w_ =
        bias_motion_w_ / std::max(kBiasVerticalForgettingFactor, 1e-10f);

    float p_rest = Square(kBiasSigmaRest * 100.0f);
    bias_rest_w_ = Square(p_rest) / bias_v_ + p_rest;
  }

  void AdaptGyrTs(float new_ts)
  {
    double old_b[3], old_a[2];
    std::copy(rest_gyr_lp_b_, rest_gyr_lp_b_ + 3, old_b);
    std::copy(rest_gyr_lp_a_, rest_gyr_lp_a_ + 2, old_a);
    gyr_ts_ = new_ts;
    FilterCoeffs(kRestFilterTau, gyr_ts_, rest_gyr_lp_b_, rest_gyr_lp_a_);
    FilterAdaptStateForCoeffChange(rest_last_gyr_lp_, 3, old_b, old_a, rest_gyr_lp_b_,
                                   rest_gyr_lp_a_, rest_gyr_lp_state_.data());
  }

  void AdaptAccTs(float new_ts)
  {
    double old_acc_b[3], old_acc_a[2];
    std::copy(acc_lp_b_, acc_lp_b_ + 3, old_acc_b);
    std::copy(acc_lp_a_, acc_lp_a_ + 2, old_acc_a);

    double old_rest_b[3], old_rest_a[2];
    std::copy(rest_acc_lp_b_, rest_acc_lp_b_ + 3, old_rest_b);
    std::copy(rest_acc_lp_a_, rest_acc_lp_a_ + 2, old_rest_a);

    acc_ts_ = new_ts;
    FilterCoeffs(param_.tau_acc, acc_ts_, acc_lp_b_, acc_lp_a_);
    FilterCoeffs(kRestFilterTau, acc_ts_, rest_acc_lp_b_, rest_acc_lp_a_);
    SetupBiasCoeffs();

    FilterAdaptStateForCoeffChange(last_acc_lp_, 3, old_acc_b, old_acc_a, acc_lp_b_,
                                   acc_lp_a_, acc_lp_state_.data());
    FilterAdaptStateForCoeffChange(rest_last_acc_lp_, 3, old_rest_b, old_rest_a,
                                   rest_acc_lp_b_, rest_acc_lp_a_,
                                   rest_acc_lp_state_.data());
  }

  void UpdateGyr(const float gyr[3], float dt)
  {
    // Rest detection (gyroscope part)
    if (kRestBiasEstEnabled)
    {
      FilterVec(gyr, 3, kRestFilterTau, gyr_ts_, rest_gyr_lp_b_, rest_gyr_lp_a_,
                rest_gyr_lp_state_.data(), rest_last_gyr_lp_);

      rest_last_squared_dev_[0] = Square(gyr[0] - rest_last_gyr_lp_[0]) +
                                  Square(gyr[1] - rest_last_gyr_lp_[1]) +
                                  Square(gyr[2] - rest_last_gyr_lp_[2]);

      float bias_clip = kBiasClip * (kPi / 180.0f);
      if (rest_last_squared_dev_[0] >= Square(kRestThGyr * (kPi / 180.0f)) ||
          std::fabs(rest_last_gyr_lp_[0]) > bias_clip ||
          std::fabs(rest_last_gyr_lp_[1]) > bias_clip ||
          std::fabs(rest_last_gyr_lp_[2]) > bias_clip)
      {
        rest_t_ = 0.0f;
        rest_detected_ = false;
      }
    }

    // Remove estimated gyro bias
    float gyr_no_bias[3] = {gyr[0] - bias_[0], gyr[1] - bias_[1], gyr[2] - bias_[2]};

    // Gyroscope strapdown prediction step
    float gyr_norm = Norm(gyr_no_bias, 3);
    float angle = gyr_norm * dt;
    if (gyr_norm > kEps)
    {
      float c = std::cos(angle * 0.5f);
      float s = std::sin(angle * 0.5f) / gyr_norm;
      float gyr_step_quat[4] = {c, s * gyr_no_bias[0], s * gyr_no_bias[1],
                                s * gyr_no_bias[2]};
      QuatMultiply(gyr_quat_, gyr_step_quat, gyr_quat_);
      Normalize(gyr_quat_, 4);
    }
  }

  void UpdateAcc(const float acc[3])
  {
    // Ignore [0, 0, 0] samples
    if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f)
    {
      return;
    }

    // Rest detection (accelerometer part)
    if (kRestBiasEstEnabled)
    {
      FilterVec(acc, 3, kRestFilterTau, acc_ts_, rest_acc_lp_b_, rest_acc_lp_a_,
                rest_acc_lp_state_.data(), rest_last_acc_lp_);

      rest_last_squared_dev_[1] = Square(acc[0] - rest_last_acc_lp_[0]) +
                                  Square(acc[1] - rest_last_acc_lp_[1]) +
                                  Square(acc[2] - rest_last_acc_lp_[2]);

      if (rest_last_squared_dev_[1] >= Square(kRestThAcc))
      {
        rest_t_ = 0.0f;
        rest_detected_ = false;
      }
      else
      {
        rest_t_ += acc_ts_;
        if (rest_t_ >= kRestMinT)
        {
          rest_detected_ = true;
        }
      }
    }

    float acc_earth[3];

    // Filter acceleration in inertial strapdown frame I_i
    QuatRotate(gyr_quat_, acc, acc_earth);
    FilterVec(acc_earth, 3, param_.tau_acc, acc_ts_, acc_lp_b_, acc_lp_a_,
              acc_lp_state_.data(), last_acc_lp_);

    // Transform to 6D earth frame E_i and normalize
    QuatRotate(acc_quat_, last_acc_lp_, acc_earth);
    Normalize(acc_earth, 3);

    // Inclination correction
    float acc_corr_quat[4];
    float q_w = std::sqrt(std::max((acc_earth[2] + 1.0f) * 0.5f, 0.0f));
    if (q_w > 1e-6f)
    {
      acc_corr_quat[0] = q_w;
      acc_corr_quat[1] = 0.5f * acc_earth[1] / q_w;
      acc_corr_quat[2] = -0.5f * acc_earth[0] / q_w;
      acc_corr_quat[3] = 0.0f;
    }
    else
    {
      // Avoid numeric singularity when acc is close to [0, 0, -1] (180 deg correction)
      acc_corr_quat[0] = 0.0f;
      acc_corr_quat[1] = 1.0f;
      acc_corr_quat[2] = 0.0f;
      acc_corr_quat[3] = 0.0f;
    }
    QuatMultiply(acc_corr_quat, acc_quat_, acc_quat_);
    Normalize(acc_quat_, 4);

    // Correction angular rate for diagnostics
    last_acc_corr_angular_rate_ =
        std::acos(std::clamp(acc_earth[2], -1.0f, 1.0f)) / acc_ts_;

    // Full VQF Kalman-filter gyroscope bias estimation (motion + rest)
    if (kMotionBiasEstEnabled || kRestBiasEstEnabled)
    {
      float bias_clip = kBiasClip * (kPi / 180.0f);

      float acc_gyr_quat[4];
      float R[9];
      float bias_lp[2];

      // Rotation matrix corresponding to 6D orientation quaternion
      GetQuat6D(acc_gyr_quat);
      R[0] = 1.0f - 2.0f * Square(acc_gyr_quat[2]) - 2.0f * Square(acc_gyr_quat[3]);
      R[1] = 2.0f * (acc_gyr_quat[2] * acc_gyr_quat[1] - acc_gyr_quat[0] * acc_gyr_quat[3]);
      R[2] = 2.0f * (acc_gyr_quat[0] * acc_gyr_quat[2] + acc_gyr_quat[3] * acc_gyr_quat[1]);
      R[3] = 2.0f * (acc_gyr_quat[0] * acc_gyr_quat[3] + acc_gyr_quat[2] * acc_gyr_quat[1]);
      R[4] = 1.0f - 2.0f * Square(acc_gyr_quat[1]) - 2.0f * Square(acc_gyr_quat[3]);
      R[5] = 2.0f * (acc_gyr_quat[2] * acc_gyr_quat[3] - acc_gyr_quat[1] * acc_gyr_quat[0]);
      R[6] = 2.0f * (acc_gyr_quat[3] * acc_gyr_quat[1] - acc_gyr_quat[0] * acc_gyr_quat[2]);
      R[7] = 2.0f * (acc_gyr_quat[0] * acc_gyr_quat[1] + acc_gyr_quat[3] * acc_gyr_quat[2]);
      R[8] = 1.0f - 2.0f * Square(acc_gyr_quat[1]) - 2.0f * Square(acc_gyr_quat[2]);

      // Calculate R * b_hat (only x and y components are needed)
      bias_lp[0] = R[0] * bias_[0] + R[1] * bias_[1] + R[2] * bias_[2];
      bias_lp[1] = R[3] * bias_[0] + R[4] * bias_[1] + R[5] * bias_[2];

      // Low-pass filter R and R * b_hat with tau_acc
      FilterVec(R, 9, param_.tau_acc, acc_ts_, acc_lp_b_, acc_lp_a_,
                motion_bias_est_r_lp_state_.data(), R);
      FilterVec(bias_lp, 2, param_.tau_acc, acc_ts_, acc_lp_b_, acc_lp_a_,
                motion_bias_est_bias_lp_state_.data(), bias_lp);

      // Measurement error and noise covariance for Kalman update
      float w[3];
      float e[3];
      if (rest_detected_ && kRestBiasEstEnabled)
      {
        e[0] = rest_last_gyr_lp_[0] - bias_[0];
        e[1] = rest_last_gyr_lp_[1] - bias_[1];
        e[2] = rest_last_gyr_lp_[2] - bias_[2];
        Matrix3SetToScaledIdentity(1.0f, R);
        w[0] = bias_rest_w_;
        w[1] = bias_rest_w_;
        w[2] = bias_rest_w_;
      }
      else if (kMotionBiasEstEnabled)
      {
        e[0] = -acc_earth[1] / acc_ts_ + bias_lp[0] - R[0] * bias_[0] -
               R[1] * bias_[1] - R[2] * bias_[2];
        e[1] = acc_earth[0] / acc_ts_ + bias_lp[1] - R[3] * bias_[0] -
               R[4] * bias_[1] - R[5] * bias_[2];
        e[2] = -R[6] * bias_[0] - R[7] * bias_[1] - R[8] * bias_[2];
        w[0] = bias_motion_w_;
        w[1] = bias_motion_w_;
        w[2] = bias_vertical_w_;
      }
      else
      {
        w[0] = -1.0f;
        w[1] = -1.0f;
        w[2] = -1.0f;
      }

      // Kalman step 1: P = P + V (bounded by bias_p0_)
      if (bias_p_[0] < bias_p0_)
      {
        bias_p_[0] += bias_v_;
      }
      if (bias_p_[4] < bias_p0_)
      {
        bias_p_[4] += bias_v_;
      }
      if (bias_p_[8] < bias_p0_)
      {
        bias_p_[8] += bias_v_;
      }

      if (w[0] >= 0.0f)
      {
        // Clip innovation to [-bias_clip, bias_clip]
        Clip(e, 3, -bias_clip, bias_clip);

        // Kalman step 2: K = P R^T inv(W + R P R^T)
        float K[9];
        Matrix3MultiplyTpsSecond(bias_p_, R, K);  // K = P R^T
        Matrix3Multiply(R, K, K);                 // K = R P R^T
        K[0] += w[0];
        K[4] += w[1];
        K[8] += w[2];                             // K = W + R P R^T
        Matrix3Inv(K, K);                         // K = inv(W + R P R^T)
        Matrix3MultiplyTpsFirst(R, K, K);         // K = R^T inv(W + R P R^T)
        Matrix3Multiply(bias_p_, K, K);           // K = P R^T inv(W + R P R^T)

        // Kalman step 3: bias = bias + K e
        bias_[0] += K[0] * e[0] + K[1] * e[1] + K[2] * e[2];
        bias_[1] += K[3] * e[0] + K[4] * e[1] + K[5] * e[2];
        bias_[2] += K[6] * e[0] + K[7] * e[1] + K[8] * e[2];

        // Kalman step 4: P = P - K R P
        Matrix3Multiply(K, R, K);       // K = K R
        Matrix3Multiply(K, bias_p_, K); // K = K R P
        for (size_t i = 0; i < 9; ++i)
        {
          bias_p_[i] -= K[i];
        }

        // Clip bias estimate to [-bias_clip, bias_clip]
        Clip(bias_, 3, -bias_clip, bias_clip);
      }
    }
  }

  static void FilterCoeffs(float tau, float ts, double out_b[3], double out_a[2])
  {
    if (tau < ts * 0.5f || tau <= 0.0f || ts <= 0.0f)
    {
      out_b[0] = 1.0;
      out_b[1] = 0.0;
      out_b[2] = 0.0;
      out_a[0] = 0.0;
      out_a[1] = 0.0;
      return;
    }

    double fc = (static_cast<double>(kSqrt2) / (2.0 * static_cast<double>(kPi))) /
                static_cast<double>(tau);
    double C = std::tan(static_cast<double>(kPi) * fc * static_cast<double>(ts));
    double D = C * C + static_cast<double>(kSqrt2) * C + 1.0;
    double b0 = C * C / D;
    out_b[0] = b0;
    out_b[1] = 2.0 * b0;
    out_b[2] = b0;
    out_a[0] = 2.0 * (C * C - 1.0) / D;
    out_a[1] = (1.0 - static_cast<double>(kSqrt2) * C + C * C) / D;
  }

  static void FilterInitialState(float x0, const double b[3], const double a[2],
                                 double out[2])
  {
    out[0] = static_cast<double>(x0) * (1.0 - b[0]);
    out[1] = static_cast<double>(x0) * (b[2] - a[1]);
  }

  static void FilterAdaptStateForCoeffChange(const float last_y[], size_t n,
                                             const double b_old[3], const double a_old[2],
                                             const double b_new[3], const double a_new[2],
                                             double state[])
  {
    if (std::isnan(state[0]))
    {
      return;
    }
    for (size_t i = 0; i < n; ++i)
    {
      state[0 + 2 * i] += (b_old[0] - b_new[0]) * static_cast<double>(last_y[i]);
      state[1 + 2 * i] +=
          (b_old[1] - b_new[1] - a_old[0] + a_new[0]) * static_cast<double>(last_y[i]);
    }
  }

  static float FilterStep(float x, const double b[3], const double a[2], double state[2])
  {
    double y = b[0] * static_cast<double>(x) + state[0];
    state[0] = b[1] * static_cast<double>(x) - a[0] * y + state[1];
    state[1] = b[2] * static_cast<double>(x) - a[1] * y;
    return static_cast<float>(y);
  }

  static void FilterVec(const float x[], size_t n, float tau, float ts, const double b[3],
                        const double a[2], double state[], float out[])
  {
    // Averaging-based initialization for the first tau seconds (requires n >= 2)
    if (std::isnan(state[0]))
    {
      if (std::isnan(state[1]))
      {
        state[1] = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
          state[2 + i] = 0.0;
        }
      }
      state[1] += 1.0;
      for (size_t i = 0; i < n; ++i)
      {
        state[2 + i] += static_cast<double>(x[i]);
        out[i] = static_cast<float>(state[2 + i] / state[1]);
      }
      if (static_cast<float>(state[1]) * ts >= tau)
      {
        for (size_t i = 0; i < n; ++i)
        {
          FilterInitialState(out[i], b, a, state + 2 * i);
        }
      }
      return;
    }

    for (size_t i = 0; i < n; ++i)
    {
      out[i] = FilterStep(x[i], b, a, state + 2 * i);
    }
  }

  static void QuatMultiply(const float q1[4], const float q2[4], float out[4])
  {
    float w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    float x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    float y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    float z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
  }

  static void QuatSetToIdentity(float out[4])
  {
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
  }

  static void QuatRotate(const float q[4], const float v[3], float out[3])
  {
    float x = (1.0f - 2.0f * q[2] * q[2] - 2.0f * q[3] * q[3]) * v[0] +
              2.0f * v[1] * (q[2] * q[1] - q[0] * q[3]) +
              2.0f * v[2] * (q[0] * q[2] + q[3] * q[1]);
    float y = 2.0f * v[0] * (q[0] * q[3] + q[2] * q[1]) +
              v[1] * (1.0f - 2.0f * q[1] * q[1] - 2.0f * q[3] * q[3]) +
              2.0f * v[2] * (q[2] * q[3] - q[1] * q[0]);
    float z = 2.0f * v[0] * (q[3] * q[1] - q[0] * q[2]) +
              2.0f * v[1] * (q[0] * q[1] + q[3] * q[2]) +
              v[2] * (1.0f - 2.0f * q[1] * q[1] - 2.0f * q[2] * q[2]);
    out[0] = x;
    out[1] = y;
    out[2] = z;
  }

  static float Norm(const float vec[], size_t n)
  {
    float s = 0.0f;
    for (size_t i = 0; i < n; ++i)
    {
      s += vec[i] * vec[i];
    }
    return std::sqrt(s);
  }

  static void Normalize(float vec[], size_t n)
  {
    float len = Norm(vec, n);
    if (len < kEps)
    {
      return;
    }
    for (size_t i = 0; i < n; ++i)
    {
      vec[i] /= len;
    }
  }

  static void Clip(float vec[], size_t n, float min_val, float max_val)
  {
    for (size_t i = 0; i < n; ++i)
    {
      if (vec[i] < min_val)
      {
        vec[i] = min_val;
      }
      else if (vec[i] > max_val)
      {
        vec[i] = max_val;
      }
    }
  }

  static void Matrix3SetToScaledIdentity(float scale, float out[9])
  {
    out[0] = scale;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
    out[4] = scale;
    out[5] = 0.0f;
    out[6] = 0.0f;
    out[7] = 0.0f;
    out[8] = scale;
  }

  static void Matrix3Multiply(const float in1[9], const float in2[9], float out[9])
  {
    float tmp[9];
    tmp[0] = in1[0] * in2[0] + in1[1] * in2[3] + in1[2] * in2[6];
    tmp[1] = in1[0] * in2[1] + in1[1] * in2[4] + in1[2] * in2[7];
    tmp[2] = in1[0] * in2[2] + in1[1] * in2[5] + in1[2] * in2[8];
    tmp[3] = in1[3] * in2[0] + in1[4] * in2[3] + in1[5] * in2[6];
    tmp[4] = in1[3] * in2[1] + in1[4] * in2[4] + in1[5] * in2[7];
    tmp[5] = in1[3] * in2[2] + in1[4] * in2[5] + in1[5] * in2[8];
    tmp[6] = in1[6] * in2[0] + in1[7] * in2[3] + in1[8] * in2[6];
    tmp[7] = in1[6] * in2[1] + in1[7] * in2[4] + in1[8] * in2[7];
    tmp[8] = in1[6] * in2[2] + in1[7] * in2[5] + in1[8] * in2[8];
    std::copy(tmp, tmp + 9, out);
  }

  static void Matrix3MultiplyTpsFirst(const float in1[9], const float in2[9],
                                      float out[9])
  {
    float tmp[9];
    tmp[0] = in1[0] * in2[0] + in1[3] * in2[3] + in1[6] * in2[6];
    tmp[1] = in1[0] * in2[1] + in1[3] * in2[4] + in1[6] * in2[7];
    tmp[2] = in1[0] * in2[2] + in1[3] * in2[5] + in1[6] * in2[8];
    tmp[3] = in1[1] * in2[0] + in1[4] * in2[3] + in1[7] * in2[6];
    tmp[4] = in1[1] * in2[1] + in1[4] * in2[4] + in1[7] * in2[7];
    tmp[5] = in1[1] * in2[2] + in1[4] * in2[5] + in1[7] * in2[8];
    tmp[6] = in1[2] * in2[0] + in1[5] * in2[3] + in1[8] * in2[6];
    tmp[7] = in1[2] * in2[1] + in1[5] * in2[4] + in1[8] * in2[7];
    tmp[8] = in1[2] * in2[2] + in1[5] * in2[5] + in1[8] * in2[8];
    std::copy(tmp, tmp + 9, out);
  }

  static void Matrix3MultiplyTpsSecond(const float in1[9], const float in2[9],
                                       float out[9])
  {
    float tmp[9];
    tmp[0] = in1[0] * in2[0] + in1[1] * in2[1] + in1[2] * in2[2];
    tmp[1] = in1[0] * in2[3] + in1[1] * in2[4] + in1[2] * in2[5];
    tmp[2] = in1[0] * in2[6] + in1[1] * in2[7] + in1[2] * in2[8];
    tmp[3] = in1[3] * in2[0] + in1[4] * in2[1] + in1[5] * in2[2];
    tmp[4] = in1[3] * in2[3] + in1[4] * in2[4] + in1[5] * in2[5];
    tmp[5] = in1[3] * in2[6] + in1[4] * in2[7] + in1[5] * in2[8];
    tmp[6] = in1[6] * in2[0] + in1[7] * in2[1] + in1[8] * in2[2];
    tmp[7] = in1[6] * in2[3] + in1[7] * in2[4] + in1[8] * in2[5];
    tmp[8] = in1[6] * in2[6] + in1[7] * in2[7] + in1[8] * in2[8];
    std::copy(tmp, tmp + 9, out);
  }

  static bool Matrix3Inv(const float in[9], float out[9])
  {
    double A = static_cast<double>(in[4] * in[8] - in[5] * in[7]);
    double D = static_cast<double>(in[2] * in[7] - in[1] * in[8]);
    double G = static_cast<double>(in[1] * in[5] - in[2] * in[4]);
    double B = static_cast<double>(in[5] * in[6] - in[3] * in[8]);
    double E = static_cast<double>(in[0] * in[8] - in[2] * in[6]);
    double H = static_cast<double>(in[2] * in[3] - in[0] * in[5]);
    double C = static_cast<double>(in[3] * in[7] - in[4] * in[6]);
    double F = static_cast<double>(in[1] * in[6] - in[0] * in[7]);
    double I = static_cast<double>(in[0] * in[4] - in[1] * in[3]);

    double det = static_cast<double>(in[0]) * A + static_cast<double>(in[1]) * B +
                 static_cast<double>(in[2]) * C;

    if (det >= -static_cast<double>(kEps) && det <= static_cast<double>(kEps))
    {
      std::fill(out, out + 9, 0.0f);
      return false;
    }

    out[0] = static_cast<float>(A / det);
    out[1] = static_cast<float>(D / det);
    out[2] = static_cast<float>(G / det);
    out[3] = static_cast<float>(B / det);
    out[4] = static_cast<float>(E / det);
    out[5] = static_cast<float>(H / det);
    out[6] = static_cast<float>(C / det);
    out[7] = static_cast<float>(F / det);
    out[8] = static_cast<float>(I / det);
    return true;
  }

  static int CommandFunc(VQFAHRS* ahrs, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms]        - Show Euler angles and "
          "quaternion periodically.\r\n">();
      LibXR::STDIO::Printf<
          "  print_quat  [time_ms] [interval_ms]   - Show quaternion in "
          "VOFA+ format periodically.\r\n">();
      LibXR::STDIO::Printf<
          "  test                                  - Test gyroscope "
          "calibration.\r\n">();
      LibXR::STDIO::Printf<
          "  bias                                  - Show estimated gyro bias, "
          "sigma and rest status.\r\n">();
    }
    else if (argc == 2)
    {
      if (strcmp(argv[1], "test") == 0)
      {
        LibXR::STDIO::Printf<"Please keep the device steady, start measurement\r\n">();
        LibXR::Thread::Sleep(3000);
        LibXR::STDIO::Printf<"Please wait\r\n">();
        float start_yaw = ahrs->euler_.Yaw();
        LibXR::Thread::Sleep(10000);
        float yaw = ahrs->euler_.Yaw();

        LibXR::STDIO::Printf<"Zero offset:%f°/min\r\n">((yaw - start_yaw) / M_PI *
                                                        180.0f * 6.0f);
      }
      else if (strcmp(argv[1], "bias") == 0)
      {
        float b[3];
        float sigma = ahrs->GetBiasEstimate(b);
        float dev[2];
        ahrs->GetRelativeRestDeviations(dev);
        LibXR::STDIO::Printf<
            "Bias(rad/s): x=%+7.5f, y=%+7.5f, z=%+7.5f, sigma=%6.4f, rest=%d "
            "(gyr_dev=%.2f, acc_dev=%.2f)\r\n">(
            b[0], b[1], b[2], sigma, ahrs->rest_detected_ ? 1 : 0, dev[0], dev[1]);
      }
    }
    else if (argc == 4)
    {
      std::string cmd(argv[1]);
      int time = std::stoi(argv[2]);
      int interval = std::stoi(argv[3]);

      interval = std::clamp(interval, 2, 1000);

      if (cmd == "show")
      {
        while (time > 0)
        {
          LibXR::STDIO::Printf<
              "Euler: pitch=%+7.5f, roll=%+7.5f, yaw=%+7.5f Quat: w="
              "%+6.4f, x=%+6.4f, y=%+6.4f, z=%+6.4f, dt=%+7.5f\r\n">(
              ahrs->euler_.Pitch(), ahrs->euler_.Roll(), ahrs->euler_.Yaw(),
              ahrs->quaternion_.w(), ahrs->quaternion_.x(), ahrs->quaternion_.y(),
              ahrs->quaternion_.z(), ahrs->dt_);
          LibXR::Thread::Sleep(interval);
          time -= interval;
        }
      }
      else if (cmd == "print_quat")
      {
        while (time > 0)
        {
          LibXR::STDIO::Printf<"%f,%f,%f,%f\n">(ahrs->quaternion_.w(),
                                                ahrs->quaternion_.x(),
                                                ahrs->quaternion_.y(),
                                                ahrs->quaternion_.z());
          LibXR::Thread::Sleep(interval);
          time -= interval;
        }
      }
      else
      {
        LibXR::STDIO::Printf<"Error: Unknown command: %s\r\n">(argv[1]);
      }
    }
    else
    {
      LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    }

    return 0;
  }

  Param param_;
  const char* gyro_topic_name_;
  const char* accl_topic_name_;

  LibXR::Topic quaternion_topic_;
  LibXR::Topic euler_topic_;

  LibXR::Quaternion<float> quaternion_;
  LibXR::EulerAngle<float> euler_;
  Eigen::Matrix<float, 3, 1> accl_, gyro_;

  LibXR::MicrosecondTimestamp last_time_ = 0;
  bool time_initialized_ = false;
  LibXR::MicrosecondTimestamp last_acc_time_ = 0;
  bool acc_time_initialized_ = false;
  float dt_ = 0.0f;

  LibXR::Thread thread_;
  LibXR::RamFS::File cmd_file_;

  // VQF coefficients
  float gyr_ts_ = 0.001f;
  float acc_ts_ = 0.001f;
  bool gyr_ts_locked_ = false;
  bool acc_ts_locked_ = false;
  double acc_lp_b_[3]{};
  double acc_lp_a_[2]{};
  double rest_gyr_lp_b_[3]{};
  double rest_gyr_lp_a_[2]{};
  double rest_acc_lp_b_[3]{};
  double rest_acc_lp_a_[2]{};
  float bias_p0_ = 0.0f;
  float bias_v_ = 0.0f;
  float bias_motion_w_ = 0.0f;
  float bias_vertical_w_ = 0.0f;
  float bias_rest_w_ = 0.0f;

  // VQF state
  float gyr_quat_[4]{1.0f, 0.0f, 0.0f, 0.0f};
  float acc_quat_[4]{1.0f, 0.0f, 0.0f, 0.0f};
  float last_acc_lp_[3]{};
  std::array<double, 6> acc_lp_state_{};
  float last_acc_corr_angular_rate_ = 0.0f;

  bool rest_detected_ = false;
  float rest_t_ = 0.0f;
  float rest_last_gyr_lp_[3]{};
  std::array<double, 6> rest_gyr_lp_state_{};
  float rest_last_acc_lp_[3]{};
  std::array<double, 6> rest_acc_lp_state_{};
  float rest_last_squared_dev_[2]{};

  float bias_[3]{};
  float bias_p_[9]{};
  std::array<double, 18> motion_bias_est_r_lp_state_{};
  std::array<double, 4> motion_bias_est_bias_lp_state_{};
};
