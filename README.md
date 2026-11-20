# shao_new：哨兵四舵轮底盘迁移

本工程将旧哨兵底盘控制迁移到 PnX V2 架构，目标板为 STM32H723（`h723_mc02`），使用 ThreadX。当前范围是 DR16 遥控下的前后左右平移、手动旋转，以及定速、阶跃变速和多正弦变速小陀螺。

当前已完成应用代码接入，曾完成 H723 Debug 编译链接和烧录校验；这些结果不等于运动功能验收通过。前后左右运动和三种小陀螺的实机验收结果尚未在本仓库记录。

## 迁移了什么

迁移参考是旧工程 `demo/control/chassis_controller.hpp`、`demo/control/chassis_controller.cpp` 及其电机配置。保留四舵轮运动学、舵向最短路径优化、驱动速度单环和舵向位置/速度串级控制；将入口、电机管理、配置和遥控输入接入当前框架。

| 内容 | 旧代码 | 当前实现与变化 |
| --- | --- | --- |
| 应用位置 | `demo/control/chassis_controller.*` | `app/chassis.hpp`、`app/chassis.cpp`，从 `app_start()` 调用 `chassis::run()` |
| 控制组织 | `controller` 类及单例 | `chassis` 命名空间，内部函数与状态由一个底盘线程组织，对外只提供 `run()` |
| 电机注册与发送 | `djimotorhandler::instance()` 注册电机 | 应用持有 `motors::motor_service`，依次注册、`start()`，每轮调用 `send_control()` |
| 电机配置 | 旧工程配置和电机初始化 | `robot.json` 描述型号、总线、ID；使用生成的 `robot::motors::*` 构造电机，并用 `static_assert` 检查型号 |
| PID 调用 | `pid_parameters`、`pid_loop`、串级封装 | 直接使用框架 `control::pid`，填写 `ref`、`fdb`，调用 `update()` 后读取 `result`；没有重写 PID 算法 |
| 舵向零点 | 初始化写入电机 `offset`，解算时读取 | 在 `steer_offsets` 数组中保存，直接传入 `optimize_target()` |
| 运动模式 | 含云台指令与 `following` 联动 | 当前只实现 `relax`、`manual`、`patrol`，由本板 DR16 直接选择 |
| 遥控数据 | 旧工程遥控接入及模式判断 | 初始化框架遥控服务，通过 `msg::subscribe()`、`msg::read()` 获取 `remoter::state` |
| 消息有效期 | 旧控制链路的判断 | 底盘额外检查遥控消息是否持续更新，约 200 ms 无更新时清空状态，并走松弛模式 |
| 时间换算 | 旧波形函数 | 使用 `tick / TX_TIMER_TICKS_PER_SECOND` 转为秒，供阶跃和多正弦波形计算 |

以上是本次应用迁移的实现选择，不表示框架要求所有应用都采用同样的类或函数组织方式。

## 文件与配置位置

| 文件或目录 | 作用 |
| --- | --- |
| [app/app.cpp](app/app.cpp) | 固件应用入口；`auto_run_on_boot` 为真时启动诊断，否则启动底盘 |
| [app/chassis.hpp](app/chassis.hpp) | `velocity_command`、模式枚举和底盘启动接口 |
| [app/chassis.cpp](app/chassis.cpp) | 电机对象、PID、运动学、遥控映射、小陀螺波形和线程循环 |
| [robot.json](configs/boards/h723_mc02/robot.json) | 八个底盘电机的型号、CAN 总线、反馈 ID，以及 IMU 等设备选择 |
| [params.json](configs/boards/h723_mc02/params.json) | DR16 输入源、UART 绑定、诊断启动等参数 |
| [boards/h723_mc02](boards/h723_mc02) | CubeMX IOC、生成的板级代码、板级绑定和链接配置 |
| `pnx_bsp/`、`pnx_devices/`、`pnx_modules/`、`pnx_libs/` | BSP、设备、服务和通用库；底盘复用这些已有能力 |

外设引脚、时钟、DMA、NVIC 和 CAN 模式由 IOC 及生成板级代码管理；`robot.json` 不替代 CubeMX 外设配置。`build/` 内的生成头文件不应手工修改。

## 电机接线配置

RF / LF / LR / RR 分别表示右前、左前、左后、右后。以下为当前配置中的反馈 ID，实际接线及电调 ID 仍需与实物核对。

| 轮组 | 舵向 GM6020 | 驱动 M3508 | 总线 |
| --- | --- | --- | --- |
| RF | `RFsteerMotor`，`0x205` | `RFdriveMotor`，`0x202` | FDCAN3 |
| LF | `LFsteerMotor`，`0x206` | `LFdriveMotor`，`0x207` | FDCAN2 |
| LR | `LRsteerMotor`，`0x205` | `LRdriveMotor`，`0x208` | FDCAN2 |
| RR | `RRsteerMotor`，`0x206` | `RRdriveMotor`，`0x201` | FDCAN3 |

当前板级 FDCAN2、FDCAN3 为经典 CAN、标称 1 Mbit/s。注册和启动通过框架电机服务完成，底盘应用不重复手写 CAN 初始化和收发回调。

## 一轮底盘控制做什么

1. 读取最新遥控状态，检查在线状态、输入源和消息超时。
2. 选择松弛、手动或小陀螺模式；模式变化时清除 PID 历史并执行松弛状态设置，随后在同一轮执行新模式，不保证额外发送一帧松弛命令。
3. 将输入转换为底盘速度 `vx`、`vy`、`vw`。坐标约定：前为 X 正方向、左为 Y 正方向、俯视逆时针为角速度正方向。
4. 通过刚体运动学，将平移与旋转叠加，求出四个轮组的速度矢量。
5. 求轮组方向和轮速；若舵向误差超过 90°，翻转驱动方向，减少舵向转动距离。
6. 驱动速度 PID 输出驱动电流命令；舵向位置 PID 输出目标角速度，再由舵向速度 PID 输出舵向电流命令。
7. 设置八个电机命令，调用 `service.send_control()`，然后休眠 1 tick。

当前 H723 时基为 1000 tick/s，循环每轮工作后休眠 1 tick；这不是严格的绝对时刻 1 kHz 调度，实际周期还包含计算和调度开销。PID 参数不能脱离实际周期判断。

## 遥控操作

仅 DR16 输入被当前底盘模式判断接受。检测到遥控离线或输入源不是 DR16 时，请求松弛模式。

| 左拨杆 | 模式 | 行为 |
| --- | --- | --- |
| 下档 | `relax` | 八个电机松弛 |
| 中档 | `manual` | 左摇杆控制平移，右摇杆 X 控制旋转 |
| 上档 | `patrol` | 原地小陀螺，右拨杆选择转速波形；此模式不叠加平移 |

手动映射为 `vx = left_y * 2`、`vy = -left_x * 2`、`vw = right_x * 5`，输入先做有限数检查、限幅和 0.05 死区处理。线速度单位 m/s，角速度单位 rad/s。

| 小陀螺时的右拨杆 | 名称 | 目标角速度 |
| --- | --- | --- |
| 下档 | 定速小陀螺 | 5 rad/s |
| 中档 | 阶跃变速小陀螺 | 当前单个周期参数为 1 s，在 4 与 9 rad/s 之间阶跃切换 |
| 上档 | 多正弦变速小陀螺 | 多个正弦项加权叠加并映射到 4～9 rad/s 范围 |

以上描述的是软件目标，不是实测转速；两种变速模式当前均不切换旋转正负方向。

## 参数与标定

几何参数、舵向零点、安装方向及 PID 目前集中在 `app/chassis.cpp`，不是全部放进 JSON。

- 轮半径：0.066 m；轮组到中心距离：0.280014 m，当前解算按对称四舵轮布局计算。
- 舵向零点按 RF / LF / LR / RR 排列：`2.868, 1.287, 0.761, -2.888` rad。
- 驱动安装方向系数：`+1, -1, -1, +1`。
- PID 参数顺序：`kp, ki, kd, max_out, max_iout`。

| 控制环 | 当前参数 |
| --- | --- |
| 驱动速度环 | `{500, 0, 0, 10000, 0}` |
| 舵向位置环 | `{40, 0, 0, 10000, 0}` |
| 舵向速度环 | `{1000, 5, 200, 10000, 3000}` |

这些值沿用旧代码作为迁移起点，不代表已经针对当前硬件完成调参。电流输出为电机协议命令值，不是安培；改变零点、安装方向或电机型号后应重新确认对应参数。

## 构建

本目录当前包含底层库源码，按 H723 配置构建，不需要先执行切换子模块分支的命令。

```powershell
cmake --preset h723-debug
cmake --build --preset h723-debug --parallel 1
```

输出固件为 `build/h723-debug/pnx_embedded.elf`。移动工程目录后，应清理旧 CMake 缓存并重新配置，避免缓存引用搬动前的路径。编译通过表示接口和链接成立；烧录 `Verified OK` 表示写入校验通过，两者均不替代实机运动验收。

## 当前边界与待验证内容

- 当前 `robot.json` 关闭 BMI088、DMIMU 和 LED 消费者；底盘应用没有启动 IMU 服务，没有用 IMU 做航向闭环或云台坐标变换。本次没有将旧 IMU 控制链路接入底盘。
- 暂未接入云台控制消息、`following`、导航、决策、功率管理或超级电容联动。
- 保留模板 `diagnose/`，但底盘初始化失败目前直接退出线程，没有接入底盘专用诊断上报。
- 控制计算仍直接读取电机 `fdb`，没有加入一轮统一反馈快照；应用控制循环尚未调用电机反馈存活检查来联锁停机。
- 电机反馈 ID、物理轮组顺序、驱动方向、舵向零点和 DR16 实机接收需逐项核对。
- 运动验收项：前进、后退、左移、右移、手动旋转，以及三种小陀螺；同时记录遥控失联和模式切换行为。
- 本次 README 更新仅核对源码与配置，没有重新编译、连接硬件或运行运动测试。

原模板架构与 API 文档继续保留在 [docs/index.md](docs/index.md) 和 [H7/F4 统一架构](docs/concepts/h7-f4-unification.md)。
