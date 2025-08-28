# LinearControl

PX4Ctrl 中的 `LinearControl` 类之所以叫作**线性控制器**，我才是因为它假设油门比率（油门通道的占空比）和无人机垂直机体方向上的加速度之间的关系是线性的，即 `computeDesiredCollectiveThrustSignal()` 方法做的事情。它假设油门值和垂直机体方向上的期望加速度满足以下的关系：

`src/realflight_modules/px4ctrl/src/controller.cpp`
```c++
/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;  // 输出油门值 = 期望的垂直加速度 / thr2acc_(比率)

  return throttle_percentage;
}
```

而该比率 `thr2acc_` 是通过一个简单的卡尔曼滤波器进行实时更新估计的：

`src/realflight_modules/px4ctrl/src/controller.cpp`
```c++
bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
    //...
    //* 目测是一个简单的卡尔曼滤波器，用于估计 thr2acc_
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //...
}
```

# PX4CtrlFSM

`PX4CtrlFSM` 是整个控制器最重要的部分，而方法 `void PX4CtrlFSM::process()` 是维护状态机并计算控制量输出的主干，它在每个控制循环中被调用。按顺序经过五个步骤：

1. STEP1: state machine runs

    主要负责 FSM 状态的检测和切换

2. STEP2: estimate thrust model

    估计重力模型，其实就是调用线性控制器中的 `estimateThrustModel()` 更新 `thr2acc_`

3. STEP3: solve and update new control commands

    求解输出给飞控的控制量

4. STEP4: publish control commands to mavros

    通过 mavros 发布控制指令

5. STEP5: Detect if the drone has landed

    降落检查

## FSM

状态机的切换流程如下图所示

```c++
/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL
*/
```

需要注意的是：

* `AUTO_TAKEOFF` `AUTO_HOVER` `CMD_CTRL` `AUTO_LAND` 都是 PX4Ctrl 控制的，飞控在这四个状态下都应该被切换为 `offboard` 模式，即由机载计算机控制。

* 而 `MANUAL_CTRL` 是手动模式，飞控在该状态下应该被切换为 `stablized` 模式，即由飞手控制

* `AUTO_TAKEOFF` 在自动起飞开始时的很短时间内，为了避免突然给电机太大的油门值导致无人机震动，同时给操作人员以反应时间，`get_rotor_speed_up_des()` 通过缓慢地增加无人机的期望加速度，使电机转速逐渐提高：

`src/realflight_modules/px4ctrl/src/PX4CtrlFSM.cpp`
```c++
Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
    // ...
    // 期望的加速度，通过一个指数函数进行映射
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
    //...
}
```

默认配置下，这个过程会持续3s:

```c++
	case AUTO_TAKEOFF:
	{
		// 起飞之前，电机先缓慢加速一段时间（MOTORS_SPEEDUP_TIME），
        if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
		{
			des = get_rotor_speed_up_des(now_time);
		}
        //...
```


# 使用方法总结

## 通道映射

PX4Ctrl 的摇杆(ch1~ch4)逻辑为美国手

ch5(aux_1) 用于在紧急情况下强制切换为 `MANUAL_CTRL` 模式。高位时，飞控退出 offboard 模式返回 stablized 模式（PX4Ctrl 处于 `MANUAL_CTRL` 模式），由飞手和飞控控制无人机。切换为低位时,由机载计算机 PX4Ctrl 控制无人机，飞控处于 offboard 模式。

ch6(aux_2) 代码中叫做离合器 (gear)，非常形象的命名。用于紧急情况下将 PX4Ctrl 强制切换为 `AUTO_HOVER` 模式。高位时断开"离合器"，进入 `AUTO_HOVER` 模式，低值为闭合离合器，此时机载计算机可以通过mavlink 控制无人机，即 `CMD_CTRL` 模式。所以在 `ego-planner` 轨迹规划失败，但是定位没飘的时候，可以将 ch6(aux_2) 打到高位，切换到悬停模式

ch7(aux_3) 用于重启飞控，打到高位则向飞控发送重启指令

## 自动起飞/降落

使用脚本 `shfiles/takeoff.sh` 进行自动起飞，其实该脚本内只有一句指令，即向 `quadrotor_msgs/TakeoffLand` 话题发布起飞的消息：

```sh
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```
自动降落同理

