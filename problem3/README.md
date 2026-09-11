# 问题三运行说明

## 文件说明

- `problem3_strategy.py`：七点覆盖、频道状态、主动定位和清除状态机。
- 主动定位采用路线感知短名单：每一步比较最近的 3 个候选频道，兼顾到达距离与定位不确定性，减少跨区域往返。
- 到达主动定位点后复用同一测站，对满足“保证可接收且交会角充分”的其他频道顺带测量，让一次移动服务多个干扰源。
- 中心点扫描后，在 12 条等长六边形覆盖路径中动态选择后续定位巡回估计最短的一条；覆盖保证和覆盖耗时不变，但结束位置更有利。
- `local_simulator.py`：离线全向源规则模拟器。
- `test_problem3.py`：覆盖、边界源和随机案例测试。
- `问题三数学推导.md`：论文方法、证明和验证方案。
- 根目录 `run_problem3.py`：单案例运行入口。
- 根目录 `benchmark_problem3.py`：离线批量测试入口。
- 根目录 `robot_client.py`：官方 HTTP 接口、幂等重试与原始日志。

## 离线检查

在仓库根目录执行：

```bash
python -m unittest -v problem3.test_problem3
```

运行一个随机案例：

```bash
python run_problem3.py --local --fast --seed 20260910
```

批量运行 20 个随机案例：

```bash
python benchmark_problem3.py --cases 20 --fast \
  --output problem3/results/offline_20_cases.json
```

离线模拟器用于回归与压力测试，不代替官方演练结果。

## 官方演练

1. 打开模拟器并登录。
2. 启动“问题 3 演练测试”。
3. 等界面明确显示接口已就绪。
4. 在仓库根目录执行：

```bash
python run_problem3.py --official --confirm-enter REHEARSAL --robot-id 你们的参赛队号 --fast
```

若模拟器修改了端口：

```bash
python run_problem3.py --official --confirm-enter REHEARSAL \
  --robot-id 你们的参赛队号 \
  --base-url http://127.0.0.1:端口 \
  --fast
```

`--confirm-enter` 是防误触保护。仅当你们已经在模拟器中确认开启正式测试时，才允许将其写成 `FORMAL`。当前阶段禁止使用 `FORMAL`。

## 运行输出

每次官方运行自动生成：

```text
logs/run_时间/run_log.jsonl
logs/run_时间/summary.csv
logs/run_时间/decision_log.jsonl
logs/run_时间/summary.json
```

- `run_log.jsonl`：完整请求体、响应体和错误。
- `summary.csv`：逐动作表格。
- `decision_log.jsonl`：候选点、定位区域和清除证书。
- `summary.json`：发现频道、清除频道、虚拟总时间和平均时间。

官方模拟器自己的加密日志仍需要从模拟器界面导出，不能用上述本地日志替代。

## 演练后必须记录

- 测试案例编码。
- 模拟器公布的真实干扰源总数。
- 程序发现频道数和清除频道数。
- 清除比例。
- 虚拟总时间。
- 平均定位清除时间。
- 程序现实运行时间。
- 是否出现异常、重试或失败清除。
- 本地日志目录和官方日志文件名。

只有连续演练无漏检、无失败清除，且日志与界面结果一致后，才能讨论正式测试。
