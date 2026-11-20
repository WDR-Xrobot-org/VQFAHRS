# VQFAHRS

## 1. 模块作用 / Purpose

基于 [VQF (Versatile Quaternion-based Filter)](https://github.com/dlaidig/vqf) 开源算法的六轴姿态和航向参考系统 (AHRS) 模块。构造时创建四元数和欧拉角发布 Topic，在 `RamFS` 中注册 `ahrs` 调试命令，并启动 `ahrs` 线程订阅陀螺仪与加速度计 Topic 进行完整 6D 姿态解算（含均值初始化二阶巴特沃斯低通倾角校正、静止检测以及基于 3×3 卡尔曼滤波的静止/运动阶段陀螺仪零偏在线估计）。

6-axis Attitude and Heading Reference System (AHRS) module based on the open-source [VQF](https://github.com/dlaidig/vqf) algorithm. On construction, it creates quaternion and Euler angle Topics, registers the `ahrs` command in `RamFS`, and starts the `ahrs` thread to subscribe to gyroscope and accelerometer Topics for full 6D orientation estimation with averaging-initialized Butterworth filtering, rest detection, and Kalman-filter online gyroscope bias estimation.

## 2. 构造接口 / Constructor

```
VQFAHRS(LibXR::RamFS& ramfs, const Param& param = {...});

```

* **依赖参数 / Dependencies**：

  * `ramfs`：接收 `ahrs` 命令的 `LibXR::RamFS` 对象（由 BSP 通过 `XR_REGISTER` 注册）。

* **配置参数 / Configuration (`Param`)**：

  * `gyro_topic_name`（默认 `"imu_gyro"`）：订阅的陀螺仪 Topic 名称。

  * `accl_topic_name`（默认 `"imu_accl"`）：订阅的加速度计 Topic 名称。

  * `quaternion_topic_name`（默认 `"ahrs_quaternion"`）：发布的四元数 Topic 名称。

  * `euler_topic_name`（默认 `"ahrs_euler"`）：发布的欧拉角 Topic 名称。

  * `accl_in_g`（默认 `true`）：输入加速度计单位是否为 `g`（`false` 表示 `m/s²`）。

  * `tau_acc`（默认 `3.0f`）：加速度计低通滤波时间常数（秒），主要可调的算法参数。

  * `task_stack_depth`（默认 `2048`）：`ahrs` 线程栈深（字节）。
  * `command_name`（默认 `"ahrs"`）：RamFS 调试命令名。同目录下各模块的命令名须唯一。

其余 VQF 算法参数（陀螺仪/加速度计采样周期自动锁存、运动与静止阶段零偏估计、静止检测阈值等）为模块内部编译期常量（取值：零偏估计与静止检测均启用，`bias_sigma_init=0.5 deg/s`、`bias_forgetting_time=100 s`、`bias_clip=2.0 deg/s`、`bias_sigma_motion=0.1 deg/s`、`bias_vertical_forgetting_factor=1e-4`、`bias_sigma_rest=0.03 deg/s`、`rest_min_t=1.5 s`、`rest_filter_tau=0.5 s`、`rest_th_gyr=2.0 deg/s`、`rest_th_acc=0.5 m/s²`），不对外配置。

## 3. Topic

| Topic | 方向 / Direction | 类型 / Type | 说明 / Description | 
| ----- | ----- | ----- | ----- | 
| `gyro_topic_name`（默认 `imu_gyro`） | 订阅 / Subscribe | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 `rad/s` / Angular velocity in rad/s | 
| `accl_topic_name`（默认 `imu_accl`） | 订阅 / Subscribe | `Eigen::Matrix<float, 3, 1>` | 加速度（默认 `g`，可配置 `m/s²`）/ Acceleration (default `g`) | 
| `quaternion_topic_name`（默认 `ahrs_quaternion`） | 发布 / Publish | `LibXR::Quaternion<float>` | 6D 姿态四元数 / 6D orientation quaternion | 
| `euler_topic_name`（默认 `ahrs_euler`） | 发布 / Publish | `LibXR::EulerAngle<float>` | 欧拉角（`rad`）/ Euler angles in rad | 

## 4. 配置示例 / Configuration Example

```
modules:
  - module: WDR-Xrobot-org/VQFAHRS
    id: ahrs
    args:
      - ramfs: ramfs
      - param:
          gyro_topic_name: "imu_gyro"
          accl_topic_name: "imu_accl"
          quaternion_topic_name: "ahrs_quaternion"
          euler_topic_name: "ahrs_euler"
          accl_in_g: true
          tau_acc: 3.0
          task_stack_depth: 2048
          command_name: "ahrs"

```

## 5. 依赖与硬件 / Dependencies and Hardware

* **依赖 / Dependencies**：`LibXR`（无其他 XRobot 模块依赖，`depends: []`）。

* **硬件 / Hardware**：无直接外设依赖，需在配置顺序中位于提供 `imu_gyro` / `imu_accl` Topic 的 IMU 驱动模块之后，并由 BSP 注册 `LibXR::RamFS`。