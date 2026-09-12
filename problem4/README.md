# 问题四运行说明

## 文件

- `problem4_strategy.py`：严格三角晶格发现、正测站凸包定位、距离证书环和 19 m 清除证书。
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

离线模拟器只用于回归，不代替官方问题四演练。默认配置使用 31 个三角晶格站点，具有方向无关的解析发现证明；`--fast` 使用 13 个区域内站点，主要用于探索时间下界，不提供同等级的最坏情况发现保证。

## 官方演练

先在模拟器中开启“问题 4 演练测试”，等接口就绪，再运行：

```bash
python run_problem4.py --official --confirm-enter REHEARSAL \
  --robot-id 你们的参赛队号 --fast
```

程序当前故意拒绝 `FORMAL`，避免在充分演练前消耗三次正式机会。

## 当前离线基线

严格配置在随机种子 0-19 的 20 个混合案例中清除 255/255 个源，平均虚拟总时间约 13947.45 s，最坏约 15020.41 s。快速配置在前 5 个随机案例中清除 65/65 个源，平均约 532 s/源，但不应据此推断最坏情况保证；必须先用演练案例验证漏检风险。严格对抗测试覆盖 16 个全部定向、位于目标圆边界且有效接收半径均为 1000 m 的情形。详细严格基线记录见 `problem4/results/offline_20.json`。
