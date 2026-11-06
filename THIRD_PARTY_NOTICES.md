# 来源与许可

- 导航、状态适配、轨迹执行和飞控接入整理自 SCUT UAV 导航工程及 2026-11-06 实机源码快照，保留 GPL-3.0 许可与文件内原作者声明。新增工程适配不改变上游算法归属。
- EGO-Planner-v2：ZJU-FAST-Lab/EGO-Planner-v2，来源提交见 `dependencies/lock.json`；ROS 2 参考实现来自 limshoonkit/ego-planner-v2-ros2。保留数值头文件的版权及来源说明。
- PX4 实机消息为 BSD-3-Clause 快照，原许可证与每个消息的来源哈希在 `src/px4_msgs/`。仿真消息由所选 PX4 仿真源码生成，保留同样的来源检查。
- `simulation/odin_simulation_gz` 是 Odin 理想传感器的 Gazebo 插件，不是真实设备驱动。原创部分保持 Apache-2.0，源自本项目 Odin 仿真工程；RGL 场景管理代码来自 RobotecAI/RGLGazeboPlugin，保留该子目录的许可证及修改说明。
- RobotecGPULidar 是独立的仿真运行依赖，按仿真清单获取并校验。其二进制缓存不是本项目原创源码，不随源码包分发。
- Odin 仿真网格源于项目用户提供的 CAD 及其网格衍生文件；它是外观近似，不是厂家标定。本工程不新增宣称 CAD 所有权，模型公开使用遵循项目合作方对该资产的授权范围。
- 实机照片由项目用户提供；轨迹图来自项目留存的飞行分析。照片与实验图不作为独立定位真值或设备规格依据。
- 真实设备驱动、厂商 SDK 和固件均不包含在本仓库中，也不受根目录 GPL 重新许可。实验设备版本仅记录在技术报告中。
- DDS Agent 独立获取，保留其上游许可证；安装后的系统服务与二进制不纳入源码。
