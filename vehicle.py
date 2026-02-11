import numpy as np
import math


class Vehicle:
    def __init__(self, id, behavior_type="conservative"):
        self.id = id
        self.type = behavior_type

        # 基于驾驶行为的参数设置
        if behavior_type == "aggressive":  # 激进型
            self.v0 = 40.0  # 期望速度 (m/s)
            self.T = 0.8  # 期望车头时距 (s) - 跟车更近
            self.a_max = 2.5  # 最大加速度 (m/s^2)
            self.b = 3.0  # 舒适减速度 (m/s^2)
            self.s0 = 15.0  # 最小安全间距 (m)
        else:  # conservative (保守型)
            self.v0 = 30.0
            self.T = 1.8
            self.a_max = 1.2
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

    def update_acceleration(self, lead_vehicle):
        # --- 1. 基础参数获取 ---
        # 自动适配 a 或 a_max，b 为减速度，v0 为期望速度
        max_accel = getattr(self, 'a_max', getattr(self, 'a', 1.5))
        s0 = getattr(self, 's0', 10)  # 静止安全距离
        T = getattr(self, 'T', 1.5)  # 时间安全距离
        b = getattr(self, 'b', 1.5)  # 舒适减速度

        s = 1000.0  # 默认前方无车，间距无穷大
        delta_v = 0.0

        # --- 2. 距离计算逻辑 ---
        if lead_vehicle:
            if self.state in ["ENTERING", "CIRCULATING", "EXITING"]:
                # 环岛内：通过弧度差计算距离 (半径 120)
                # 使用 % (2*pi) 确保永远是正向间距
                angle_diff = (self.current_angle - lead_vehicle.current_angle) % (2 * math.pi)
                s = angle_diff * 120.0
            elif self.state == "APPROACHING":
                # 进场直线：车头对车尾
                s = self.dist_to_center - lead_vehicle.dist_to_center
            elif self.state == "STRAIGHT_OUT":
                # 出场直线：车尾对车头
                s = lead_vehicle.dist_to_center - self.dist_to_center

            # 速度差（我比前车快多少）
            delta_v = self.v - lead_vehicle.v

            # 【关键修正】车辆长度补偿：调小一点（30-35）能让车流更紧凑，不易锁死
            s -= 30.0

            # --- 3. IDM 核心公式计算 ---
        # 确保 s 永远不为 0 或负数，防止除法报错
        s = max(s, 1.0)

        # 自由加速项
        accel_free = max_accel * (1 - (max(0, self.v) / self.v0) ** 4)

        # 交互项（跟车项）
        s_star = s0 + max(0, self.v * T + (self.v * delta_v) / (2 * math.sqrt(max_accel * b)))
        accel_int = -max_accel * (s_star / s) ** 2

        total_accel = accel_free + accel_int

        # --- 4. 强制防御补丁（解决“走一走就不动”的问题） ---

        # A. 彻底死锁保护：如果前方其实有空间（s > 15），但 IDM 依然算不出正数
        if self.v < 0.2 and s > 15.0:
            total_accel = max(total_accel, 0.8)  # 强行给一个起步推力

        # B. 倒车保护：如果已经停了，就不再允许负加速度
        if self.v <= 0 and total_accel < 0:
            total_accel = 0

        # C. 极端情况：如果距离太近（s < 2），强制刹车
        if s < 2.0:
            total_accel = -b * 2.0
        if self.v < 0.1 and s > 10:  # 只要前面有 10 像素空隙
            return 0.5  # 强行给个起步速度
        # 最终限幅
        return max(-b * 2.5, min(max_accel, total_accel))
