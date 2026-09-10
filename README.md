# B题机器狗接口与日志模块

这个目录里已经放好一个最小可用版本：

- `robot_client.py`：给算法调用的接口层。
- `smoke_test.py`：最小联调脚本。
- `logs/`：运行后自动生成，不需要手动创建。

## 运行前

先打开 B 题模拟器，登录，启动演练测试，等界面显示机器狗接口已经就绪。

## 最小联调

把下面的 `<参赛队号>` 换成模拟器当前登录的参赛队号：

```powershell
python smoke_test.py --robot-id <参赛队号> --channel 1
```

如果模拟器端口不是 `2026`：

```powershell
python smoke_test.py --robot-id <参赛队号> --base-url http://127.0.0.1:你的端口 --channel 1
```

如果想顺便试一次清除请求：

```powershell
python smoke_test.py --robot-id <参赛队号> --channel 1 --try-clear
```

## 给算法同学调用

```python
from robot_client import RobotClient, config_from_env

with RobotClient(config_from_env(robot_id="<参赛队号>")) as client:
    client.enter()
    client.measure(0, 0, 1)
    client.measure(600, 0, 1)
    client.clear(600, 0, 1)
    client.exit()
```

每次运行会生成：

- `logs/run_时间/run_log.jsonl`
- `logs/run_时间/summary.csv`

`jsonl` 方便完整复盘，`csv` 方便统计和放进论文。

## 问题3第一版自动搜索

模拟器进入“问题3演练测试”并显示接口就绪后运行：

```powershell
python run_problem3.py --robot-id <参赛队号>
```

如果要先小范围测试某几个频道：

```powershell
python run_problem3.py --robot-id <参赛队号> --channels 1,2,3
```

运行结束后查看统计：

```powershell
python analyze_logs.py
```

这版策略是保守基线：固定扫描中心点、内圈和外圈探测点，记录各频道示向度，用交会定位估计位置，然后在估计点附近尝试清除。先用它跑出演练结果，再根据日志优化移动路径和检测点数量。

## 问题3优化版

优化版访问相同的 17 个覆盖点，但以内外圈交替顺序缩短移动路径；每个频道收集到 4 条有效方向后停止继续测量，最后仍集中规划清除顺序：

```powershell
python run_problem3_fast.py --robot-id <参赛队号> --channels 1-20
```

四轮基线日志的离线回放中，优化版仍定位全部 54 个信号，预计平均虚拟时间约为 5600。正式使用前应至少运行三轮演练，与基线比较清除成功率和最终虚拟时间。
