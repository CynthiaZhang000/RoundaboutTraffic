import numpy as np


class Vehicle:
    def __init__(self, id, behavior_type="conservative"):
        self.id = id
        self.type = behavior_type

        # 基于驾驶行为的参数设置
        if behavior_type == "aggressive":  # 激进型
            self.v0 = 40.0  # 期望速度 (m/s)
            self.T = 0.8  # 期望车头时距 (s) - 跟车更近
            self.a = 2.5  # 最大加速度 (m/s^2)
            self.b = 3.0  # 舒适减速度 (m/s^2)
            self.s0 = 15.0  # 最小安全间距 (m)
        else:  # conservative (保守型)
            self.v0 = 30.0
            self.T = 1.8
            self.a = 1.2
            self.b = 1.5
            self.s0 = 30.0

        self.v = self.v0 * 0.5  # 当前速度
        self.pos = 0.0  # 沿路径的累计位置
        self.lane = 0  # 车道索引

        self.wait_time = 0  # 统计在入口等待的总时长
        self.enter_time = 0  # 进入环岛的时间
        self.finish_time = 0  # 离开环岛的时间
        self.conflict_count = 0  # 记录该车参与的冲突次数

        self.visual_x = None
        self.visual_y = None

    def update_acceleration(self, lead_vehicle, R):
        # --- [ 第一步：计算间距 s ] ---
        if lead_vehicle:
            if self.state == "CIRCULATING" and lead_vehicle.state == "CIRCULATING":
                # 顺时针间距：(我的角度 - 前车角度) % 2π
                angle_diff = (self.current_angle - lead_vehicle.current_angle) % (2 * np.pi)
                s = (angle_diff * R) - 30
            else:
                s = abs(self.dist_to_center - lead_vehicle.dist_to_center) - 30
        else:
            s = 1000

        # 保护：防止间距过小导致计算爆炸
        s = max(s, 2.0)

        # --- [ 第二步：核心 IDM 公式 (这是你最熟悉的，没变过) ] ---
        # 1. 自由流加速度
        accel_free = self.a * (1 - (self.v / self.v0) ** 4)

        # 2. 交互加速度 (跟车逻辑)
        delta_v = self.v - (lead_vehicle.v if lead_vehicle else 0)
        s_star = self.s0 + max(0, self.v * self.T + (self.v * delta_v) / (2 * np.sqrt(self.a * self.b)))

        accel_int = -self.a * (s_star / s) ** 2

        # 3. 结果合并
        total_accel = accel_free + accel_int

        # 限制范围，不让车倒退，也不让它加速过猛
        return max(-self.b * 2, min(self.a, total_accel))

    def get_bezier_path(start_pos, control_pos, end_pos, steps=15):
        """生成贝塞尔曲线上的点列表"""
        path = []
        for t in np.linspace(0, 1, steps):
            # 二次贝塞尔曲线公式: (1-t)^2*P0 + 2t(1-t)*P1 + t^2*P2
            x = (1 - t) ** 2 * start_pos[0] + 2 * (1 - t) * t * control_pos[0] + t ** 2 * end_pos[0]
            y = (1 - t) ** 2 * start_pos[1] + 2 * (1 - t) * t * control_pos[1] + t ** 2 * end_pos[1]
            path.append((x, y))
        return path