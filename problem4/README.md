# 问题四运行说明

## 文件

- `problem4_strategy.py`：认证发现网格、正测站凸包定位、距离证书环和 19 m 清除证书。
- `问题四数学推导.md`：模型、发现证明、定向定位和清除证明。
- 根目录 `local_simulator_problem4.py`：全向与 180 度定向混合源离线模拟器。
- 根目录 `run_problem4.py`：单案例离线或官方演练入口。
- 根目录 `benchmark_problem4.py`：批量离线测试入口。
- 根目录 `test_problem4.py`：几何证书、方向规则、随机案例和边界对抗测试。

## 离线检查

```bash
python -m unittest -v test_problem4
python run_problem4.py --local --fast --seed 20260911
python benchmark_problem4.py --cases 20 --fast --output problem4/results/offline_20.json
```

离线模拟器只用于回归，不代替官方问题四演练。默认配置使用外扩后的 31 个三角晶格站点；`--fast` 使用“中心 + 8 点内环 + 16 点外环”的 25 点认证网格，外环内切半径约 1804.6 m，仍覆盖整个目标圆和任意 180° 定向源。快速模式还减少定位候选点、误差采样，并在发现阶段达到 2 个正向观测后停止重复扫描。

## 官方演练

先在模拟器中开启“问题 4 演练测试”，等接口就绪，再运行：

```bash
python run_problem4.py --official --confirm-enter REHEARSAL \
  --robot-id 你们的参赛队号 --fast
```

程序当前故意拒绝 `FORMAL`，避免在充分演练前消耗三次正式机会。

## 当前离线基线

本轮优化后的快速配置在随机种子 0-19 的 20 个混合案例中清除全部源（20/20），平均虚拟总时间约 7557 s，最坏约 8015 s，平均测量约 310 次；按全部源加权约 590 s/源。严格配置同样保持方向无关发现保证。官方演练仍不可替代离线回归，正式测试前请继续使用 `REHEARSAL`。
