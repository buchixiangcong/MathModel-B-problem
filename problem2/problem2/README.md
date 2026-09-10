# B 题问题二程序说明

本目录实现第一次示向观测后的第二检测点稳健选择。

## 文件

- `problem2_strategy.py`：首次可行域、安全接收候选域和极小极大搜索。
- `test_problem2_strategy.py`：几何、安全性和策略测试。
- `example_input.json`：第一次在原点测得 `30°` 的示例。
- `example_output.json`：完整数值结果。
- `example_result.png`：首次可行域、候选域及推荐点示意图。
- `问题二数学推导.md`：可用于论文正文的推导与算法说明。

程序复用 `../../problem1/problem1/problem1_geometry.py` 中的扇形、凸包、直径和最小覆盖圆模块。

## 方法概览

1. 用第一次 `±1°` 示向扇形、目标圆域和最大接收距离 `1500 m` 构造首次可行域。
2. 将圆用外切正多边形逼近，使首次多边形包含真实可行域，保留稳健性。
3. 构造安全核心：候选点到首次可行域中任一点的距离均不超过 `1000 m`。
4. 在安全核心内生成候选第二检测点。
5. 枚举可能真实位置和第二次测量误差，计算两次扇形交的最坏直径。
6. 选择最坏直径最小的候选点，并输出不超过最优值 `5%` 的近优候选区域。

## 运行

```bash
cd problem2
python3 problem2_strategy.py example_input.json \
  --output example_output.json \
  --plot example_result.png
```

运行测试：

```bash
python3 -m unittest -v test_problem2_strategy.py
```

## 输入参数

- `first_observation`：第一次测点坐标与示向度。
- `angle_error_deg`：角度误差上界，题目取 `1.0`。
- `target_radius`：目标区域半径，题目取 `1800 m`。
- `minimum_reception_radius`、`maximum_reception_radius`：题目取 `1000 m`、`1500 m`。
- `circle_side_count`：圆的外切多边形边数；越大，首次可行域越接近真实圆弧区域。
- `candidate_angle_count`、`candidate_radial_levels`：第二测点候选网格分辨率。
- `source_edge_subdivisions`、`source_interior_levels`：首次可行域中的真实位置采样分辨率。
- `error_sample_count`：第二次观测误差在 `[-1°,1°]` 内的采样数。
- `near_optimal_tolerance`：近优区域阈值，默认 `5%`。

## 输出解释

- `first_region`：包含真实首次可行域的保守多边形。
- `circle_outer_approximation_excess_m`：外切多边形相对圆边界的最大径向超出量。
- `guaranteed_core_boundary`：安全接收候选域的内接边界。
- `optimum`：离散极小极大搜索得到的推荐点和最坏定位指标。
- `near_optimal_candidates`：最坏直径不超过最优值 `5%` 的候选点。
- `baselines`：可行域最小覆盖圆圆心和简单正交法的对照结果。

## 保证与限制

安全核心具有连续几何保证：其中任一点到首次保守可行域中任一点均不超过 `1000 m`，所以不论真实有效接收半径在 `[1000,1500] m` 中取何值，第二次均能收到全向源信号。

最坏定位直径通过确定性网格近似计算。它不是未经证明的连续全局最优；应通过加密候选点、真实位置及误差网格做收敛分析。程序所有分辨率都在输入文件中显式给出，便于复现。
