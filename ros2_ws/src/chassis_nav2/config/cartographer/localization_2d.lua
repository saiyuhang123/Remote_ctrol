-- JD03 巡检车 2D 纯定位配置：加载建图保存的 .pbstream，不扩建地图。
-- 启动时车须停在建图起点附近（地图原点即建图起点），否则纯定位初始位姿对不上。

include "mapping_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20
-- 全局定位门槛放宽回 cartographer 默认 0.6（kybot 即用默认值，自动收敛明显更容易）；
-- 建图保持 0.7 防错回环。环境对称时错误匹配有拉歪定位的风险，但远好于"永远不收敛"
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
