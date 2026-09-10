# B 题问题一程序说明

本目录包含 2026 年全国大学生数学建模竞赛 B 题问题一的独立实现。

## 文件

- `problem1_geometry.py`：定位区域、直径、最小覆盖圆和作图程序。
- `test_problem1_geometry.py`：单元测试与随机交叉验证。
- `example_input.json`：两测点交会示例。
- `问题一数学推导.md`：可用于论文正文的完整推导。

## 输入格式

角度从 `x` 轴正向开始逆时针计算，单位为度，程序自动处理跨越 `0°/360°` 的情况。

```json
{
  "angle_error_deg": 1.0,
  "observations": [
    {"x": -1000.0, "y": 0.0, "bearing_deg": 45.0},
    {"x": 1000.0, "y": 0.0, "bearing_deg": 135.0}
  ]
}
```

## 运行

只计算并在终端输出 JSON：

```bash
cd problem1
python3 problem1_geometry.py example_input.json
```

保存结果并生成论文插图：

```bash
python3 problem1_geometry.py example_input.json \
  --output example_output.json \
  --plot example_result.png
```

作图时优先使用 `matplotlib`；若环境中没有，则自动改用 Pillow。其余计算只使用 Python 标准库。

运行测试：

```bash
python3 -m unittest -v test_problem1_geometry.py
```

## 输出状态

- `bounded`：得到正常的有界多边形，可以计算有限直径。
- `degenerate`：交集退化为线段或点，仍可计算直径。
- `unbounded`：观测交会条件不足，区域直径为无穷大。
- `empty`：各观测角扇形没有公共点，应检查观测归属或数值输入。

## 结果解释

- `diameter.length` 是定位多边形内任意两点的最大距离。
- `minimum_enclosing_circle` 是真正覆盖整个定位区域的最小圆。
- `diameter_circle` 以一对最远顶点为直径端点。
- `diameter_circle_covers` 直接回答题目的第二问。
- `coverage_margin >= 0` 表示直径圆覆盖全部顶点；负值表示存在顶点落在圆外。

定位区域一般不保证能被直径为其直径的圆覆盖。等边三角形的最小覆盖圆半径为 `D/sqrt(3)`，严格大于 `D/2`，因此是直接反例。
