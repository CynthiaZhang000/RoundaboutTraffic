# 环岛交通流行为分析系统 (Roundabout Traffic Analysis System)

这是一个基于 Python 和 Pygame 开发的环岛交通仿真系统。该系统旨在研究**激进型 (Aggressive)** 与 **保守型 (Conservative)** 驾驶行为对环岛通行效率及安全性的影响。

---

## 🌟 核心功能

* **多智能体仿真**：实时模拟不同驾驶权重的车辆在环岛中的决策与交互。
* **实时数据采集**：自动记录车辆的通行时间 (Travel Time)、排队等待时间 (Wait Time) 及安全冲突次数 (Conflicts)。
* **一键数据导出**：按数字键 `8` 自动将所有统计数据以时间戳命名并同步存入 `report/` 文件夹。
* **深度行为分析**：内置自动化分析脚本，一键生成包含通行时间频率分布、冲突对比及效率区间的专业研究报告。

---

## 📂 项目结构

```text
.
├── simulation.py           # 仿真主程序（包含车辆逻辑与 GUI 界面）
├── analysis_report.py      # 数据分析脚本（从 report 文件夹读取数据生成 2x2 综合报告）
├── setup.py                # 环境安装与项目配置脚本
├── requirements.txt        # Python 依赖包列表
└── report/                 # 数据仓库（自动生成，存放所有 CSV 原始数据与分析图表）
    ├── traffic_analysis_*.csv    # 详细车辆档案（包含每一辆车的类型、时间、冲突数）
    ├── flow_data_*.csv          # 宏观流量效率数据
    ├── travel_time_*.csv        # 通行时间原始记录
    └── report_*.png             # 综合分析可视化报告图