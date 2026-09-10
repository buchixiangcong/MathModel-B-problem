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
