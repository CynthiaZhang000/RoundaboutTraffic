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

    # def update_acceleration(self, lead_vehicle, R):
    #     """
    #     基于智能驾驶模型 (IDM) 计算车辆的加速度。
    #     lead_vehicle: 前方车辆对象 (Vehicle)，如果没有前车则为 None。
    #     """
    #     # --- 0. 环境适配：根据状态动态调整期望速度 ---
    #     # 环岛内弯道较急，即使是激进型也应该降速，这里设为期望速度的 70%
    #     current_v0 = self.v0
    #     if hasattr(self, 'state') and self.state == "CIRCULATING":
    #         current_v0 = self.v0 * 0.7
    #
    #         # 1. 自由流加速度 (使用动态调整后的 v0)
    #     accel_free = self.a * (1 - (self.v / current_v0) ** 4)
    #
    #     if lead_vehicle is None:
    #         # 同样对自由流加速度做限制，防止切换状态时 v0 突变导致的弹射
    #         return max(-self.b * 2, min(self.a, accel_free))
    #
    #     # 2. 计算间距 s
    #     if hasattr(self, 'state') and self.state == "CIRCULATING":
    #         # 环岛内：半径 R 建议动态获取，这里暂用你代码里的 120
    #         circle_length = 2 * np.pi * 120
    #         # # 净间距 = 圆周距离 - 车辆占位长度 (25)
    #         # s = (lead_vehicle.pos - self.pos) % circle_length - 25
    #         if lead_vehicle and lead_vehicle.state == "CIRCULATING":
    #             # 计算两车之间的角度差 (处理跨越 0 度的情况)
    #             # 顺时针流向：(self - lead)；逆时针流向：(lead - self)
    #             angle_diff = (self.current_angle - lead_vehicle.current_angle) % (2 * np.pi)
    #             # 间距 s = 弧长 - 车身长度
    #             s = (angle_diff * R) - 25
    #         else:
    #             # 前方没车或前车已出环岛
    #             s = 1000
    #     else:
    #         # 引道中：净间距 = 距离差 - 车辆占位长度 (25)
    #         s = self.dist_to_center - lead_vehicle.dist_to_center - 25
    #
    #     # --- 鲁棒性保护：s0 是静止间距，s 是实时间距 ---
    #     # 如果 s 变成负数（重叠了），给一个极小的正值防止公式崩溃，并触发强制刹车
    #     s = max(s, 0.5)
    #
    #     # 3. 计算 IDM 的交互项
    #     delta_v = self.v - lead_vehicle.v
    #
    #     # 期望间距 s_star 公式
    #     term1 = self.v * self.T
    #     term2 = (self.v * delta_v) / (2 * np.sqrt(self.a * self.b))
    #     s_star = self.s0 + max(0, term1 + term2)
    #
    #     # 交互加速度
    #     accel_int = -self.a * (s_star / s) ** 2
    #
    #     # 4. 总加速度
    #     total_accel = accel_free + accel_int
    #
    #     # --- 5. 核心：最终限制 (防止瞬间高速/撞停的关键) ---
    #     # 哪怕公式算出 100，这里也只会输出 self.a (比如 2.5)
    #     # 哪怕公式算出 -500，这里也只会输出 -self.b * 2 (比如 -6.0)
    #     accel = max(-self.b * 2, min(self.a, total_accel))
    #
    #     return accel
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