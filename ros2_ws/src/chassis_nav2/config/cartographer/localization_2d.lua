-- JD03 巡检车 2D 纯定位配置：加载建图保存的 .pbstream，不扩建地图。
-- 启动时车须停在建图起点附近（地图原点即建图起点），否则纯定位初始位姿对不上。

include "mapping_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

return options
