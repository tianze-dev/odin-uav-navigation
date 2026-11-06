# RGL Gazebo 来源与本地改动

来源：RobotecAI/RGLGazeboPlugin，提交 `d4bf3cf36fe4a363a56df1bec2ce3809720db563`。
本 Odin 工程引入范围：Scene.cc、Mesh.cc、Utils.cc、RGLServerPluginManager.cc，及 2 个头文件和上游 LICENSE。
保留原始格式与版权；本项目使用自己的 ament 构建入口，不引入上游 GUI 插件。

RGL Core 使用 v0.21.0。运行库、API 头通过公开来源下载并按
`dependencies/lock.json` 校验；不从任何其他工作空间提取运行依赖。
Core 与本插件该版本的 LICENSE 文本相同，随安装保留本目录 LICENSE。

本地补丁记录：

- 场景管理器沿用来源工程的 reset/rebuild/resource cleanup 修正。
- namespace 改为 odin_sim，CustomSensor 检测名改为 odin_sim::DepthSensor，避免插件类名混淆。
- 本项目自行实现 src/DepthSensor.cc，使用针孔排列射线和未压缩的有序点结果；不包含其他传感器扫描预设。
- 实体删除时只在并行 PostUpdate 标记待清理，在串行 PreUpdate 释放深度图，避免多个传感器并发调用 RGL 清理接口。

这些是基础健壮性修复，不引入逐点时间、历史位姿或运动畸变。
