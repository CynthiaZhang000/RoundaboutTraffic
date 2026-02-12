import numpy as np
import math
import pygame


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

        self.spawn_time = pygame.time.get_ticks()

    def update_acceleration(self, lead_vehicle):
        max_accel = getattr(self, 'a_max', 1.5)
        s0 = getattr(self, 's0', 10)
        T = getattr(self, 'T', 1.5)
        b = getattr(self, 'b', 1.5)

        s = 1000.0
        delta_v = 0.0

        if lead_vehicle:
            if self.state in ["ENTERING", "CIRCULATING", "EXITING"]:
                # 环岛内距离计算：弧长
                angle_diff = (self.current_angle - lead_vehicle.current_angle) % (2 * math.pi)
                # 如果算出来 angle_diff 太接近 2pi，说明前车就在屁股后面，间距应该是极小的正数
                if angle_diff > math.pi: angle_diff = 0.1
                s = angle_diff * 145.0  # 使用外圈半径计算更准确
            elif self.state == "APPROACHING":
                s = self.dist_to_center - lead_vehicle.dist_to_center

            delta_v = self.v - lead_vehicle.v
            # 补偿车身长度：40 像素是安全值
            s -= 45.0

        s = max(s, 2.0)  # 严禁距离变成 0

        if s < 10.0:  # 极其危险距离
            if delta_v > 0:  # 且我比前车快
                return -self.b * 4.0  # 强制紧急刹车
            else:
                return 0  # 已经慢下来了，保持静止

        # IDM 核心公式
        accel_free = max_accel * (1 - (max(0, self.v) / self.v0) ** 4)
        s_star = s0 + max(0, self.v * T + (self.v * delta_v) / (2 * math.sqrt(max_accel * b)))
        accel_int = -max_accel * (s_star / s) ** 2

        total_accel = accel_free + accel_int

        # --- 【关键疏通逻辑】 ---
        # 如果车辆在环岛系统内（非直线排队态），且速度极低
        if self.state in ["ENTERING", "CIRCULATING", "EXITING"]:
            if self.v < 1.0:
                # 只要前面有 15 像素（约半个车身）的空隙，强制给加速度起步
                if s > 15.0:
                    total_accel = 1.0
                else:
                    total_accel = 0  # 实在贴太近了才停

        # 严禁倒车
        if self.v <= 0 and total_accel < 0:
            total_accel = 0

        return max(-b * 3, min(max_accel, total_accel))
