import csv
import math
import pygame
import numpy as np
import random
from vehicle import Vehicle

# --- 增强版常量 ---
SCREEN_SIZE = 800
WIDTH = 800  # 这里的数值应与你 pygame.display.set_mode 里的宽度一致
HEIGHT = 800 # 这里的数值应与高度一致
CENTER = WIDTH // 2
R = 120  # 环岛半径
ROAD_LEN = 250  # 引道长度
FPS = 60
DT = 1 / FPS

# 颜色
WHITE = (255, 255, 255)
ROAD_COLOR = (50, 50, 50)
RED = (200, 50, 50)  # 激进
BLUE = (50, 50, 200)  # 保守


class AdvancedSim:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
        self.clock = pygame.time.Clock()
        self.vehicles = []
        self.spawn_timer = 0
        # 入口/出口角度定义: 0:右, pi/2:下, pi:左, 3pi/2:上
        self.directions = [0, np.pi / 2, np.pi, 3 * np.pi / 2]
        # self.directions = [np.pi / 2, 3 * np.pi / 2]

        self.vehicles = []
        self.data_logs = []  # 新增：用于存放已离开车辆的数据
        self.spawn_timer = 0

        self.total_conflicts = 0  # 全局冲突计数器
        self.safety_threshold = 2.0  # 定义危险距离（米）：小于2米视为冲突

        self.config = {
            "car_max_v": 25.0,  # 红车最大速度
            "truck_max_v": 15.0,  # 蓝车最大速度
            "safe_gap": 40.0,  # 基础安全跟车距离 (像素)
            "yield_angle": 1,  # 入场礼让判定弧度 (越大越保守)
            "spawn_rate": 0.5  # 车辆生成频率
        }

    def spawn_vehicle(self):
        # 1. 随机选择一个入口角度 (0, pi/2, pi, 3pi/2)
        selected_angle = random.choice(self.directions)
        if len(self.vehicles) >= 15:
            return
        # 2. 安全检查：防止在同一个入口瞬间生成两辆重合的车
        # 检查在该入口引道末端（ROAD_LEN + R 附近）是否已经有车
        too_close = any(
            v.dist_to_center > (ROAD_LEN + R - 40)
            for v in self.vehicles
            if v.state == "APPROACHING" and v.start_angle == selected_angle
        )

        if not too_close:
            # 3. 确定驾驶员类型（你可以根据研究需要调整这个比例）
            v_type = random.choices(["aggressive", "conservative"], weights=[0.3, 0.7])[0]

            # 4. 实例化车辆
            v = Vehicle(random.randint(1000, 9999), v_type)

            # 5. 初始化路径与状态属性
            v.state = "APPROACHING"
            v.start_angle = selected_angle
            # 随机选择一个不等于入口的出口角度
            v.end_angle = random.choice([a for a in self.directions if a != selected_angle])

            # 初始距离：引道尽头
            v.dist_to_center = ROAD_LEN + R

            # 初始化位置变量（防止 draw 或 update 报错）
            v.pos = 0  # 环岛内线性位置
            v.current_angle = v.start_angle  # 环岛内实时角度

            # 6. 初始化统计变量（用于 CSV 导出）
            v.wait_time = 0  # 排队时长
            v.conflict_count = 0  # 安全冲突次数

            # 将新车加入列表
            self.vehicles.append(v)

    def get_coords(self, v):
        CENTER = 400
        R = 120
        LANE_OFFSET = 25

        # --- 1. 弧线进入状态：直接返回 update 算好的位置 ---
        if v.state == "ENTERING":
            return v.visual_x, v.visual_y

        # --- 2. 环岛状态：没有任何 base + offset，直接用圆周方程 ---
        if v.state == "CIRCULATING":
            v.visual_x = CENTER + (R + 25) * np.cos(v.current_angle)
            v.visual_y = CENTER + (R + 25) * np.sin(v.current_angle)
            v.angle_to_draw = v.current_angle + np.pi / 2 + np.pi
            return v.visual_x, v.visual_y

        # --- 3. 引道状态：手动处理偏移，确保不产生跳变 ---
        if v.state == "APPROACHING":
            # 基础直线位置
            base_x = CENTER + v.dist_to_center * np.cos(v.start_angle)
            base_y = CENTER + v.dist_to_center * np.sin(v.start_angle)

            # 偏移向量（向右 25 像素）
            target_ox = LANE_OFFSET * np.cos(v.start_angle - np.pi / 2)
            target_oy = LANE_OFFSET * np.sin(v.start_angle - np.pi / 2)

            # 这里的关键：当距离很近时（< R+100），我们降低平滑权重，让它死死咬住 target
            v.visual_x = base_x + target_ox
            v.visual_y = base_y + target_oy
            v.angle_to_draw = v.start_angle + np.pi
            return v.visual_x, v.visual_y

        return v.visual_x, v.visual_y


    def get_avg_speed(self):
        """计算当前场上所有车辆的平均速度"""
        if not self.vehicles:
            return 0.0
        total_speed = sum(v.v for v in self.vehicles)
        return total_speed / len(self.vehicles)

    # def update(self):
    #     R = 120
    #     DT = 0.02
    #     CENTER_X, CENTER_Y = 400, 400
    #     LANE_OFFSET = 25
    #
    #     # 预筛选环岛内车辆，用于礼让
    #     circulating_v = [v for v in self.vehicles if v.state in ["CIRCULATING", "ENTERING"]]
    #
    #     for v in self.vehicles[:]:
    #         # 获取当前前车
    #         lead_v = self.get_lead_vehicle(v)
    #         # 调用你的 IDM 物理公式计算本帧加速度
    #         v.a = v.update_acceleration(lead_v)
    #         # --- 强行解冻逻辑 ---
    #         if v.v < 0.1:  # 如果车已经停了
    #             # 如果前面没有 lead_v，或者 lead_v 离得很远（比如 > 50 像素）
    #             if not lead_v or (v.dist_to_center - lead_v.dist_to_center > 50):
    #                 # 且不是因为礼让环岛而停下
    #                 if v.state != "APPROACHING" or not self.check_ring_conflict(v):
    #                     v.a = max(v.a, 0.5)  # 强行给一个起步加速度
    #         # 应用加速度更新速度
    #         v.v += v.a * DT
    #         # 严禁倒车：如果 v 变成负数，强制归零
    #         if v.v < 0.05:
    #             v.v = 0
    #             v.a = max(0, v.a)  # 既然停了，就不再允许继续施加负加速度
    #         # 状态机处理位移
    #         if v.state == "APPROACHING":
    #             # 环岛礼让逻辑
    #             is_blocked = False
    #             if v.dist_to_center < R + 80:
    #                 for other in circulating_v:
    #                     d_angle = (v.start_angle - other.current_angle) % (2 * np.pi)
    #                     if 0 < d_angle < 0.7:
    #                         is_blocked = True
    #                         break
    #
    #             if is_blocked and v.dist_to_center < R + 45:
    #                 v.v = max(0, v.v - 4.0 * DT * 60)  # 强制刹车
    #                 if v.v < 0.1: v.v = 0
    #             else:
    #                 v.v = max(0, v.v + v.a * DT)
    #
    #             v.dist_to_center -= v.v * DT
    #             if v.dist_to_center <= R + 70 and not is_blocked:
    #                 # 记录切换前的最后一刻坐标
    #                 last_x, last_y = v.visual_x, v.visual_y
    #                 v.state = "ENTERING"
    #                 v.entry_path = self.generate_entry_path(v, R)
    #                 # 【衔接补丁】：强制让路径的起点等于车辆当前坐标
    #                 # 这样即便 generate_entry_path 有微小偏差，第一帧也不会跳变
    #                 v.entry_path[0] = (last_x, last_y)
    #                 v.path_index = 0
    #                 continue
    #
    #
    #         elif v.state == "ENTERING":
    #             v.v = min(v.v, 6.0)
    #             v.v = max(v.v + v.a * DT, 1.5)  # 保底给 1.0 速度，防止在弧线上冻死
    #
    #             # 关键：根据物理速度计算索引增加多少，而不是 +1
    #
    #             # 假设你的 path 点间距大约是 2 像素，如果点很密就除以更大的数
    #             v.path_index += 1
    #
    #             idx = int(v.path_index)
    #
    #             if idx < len(v.entry_path) - 1:
    #
    #                 v.visual_x, v.visual_y = v.entry_path[idx]
    #
    #                 # 同步坐标给 IDM 判定
    #
    #                 dx, dy = v.visual_x - 400, v.visual_y - 400
    #
    #                 v.current_angle = math.atan2(dy, dx)
    #
    #                 # 角度平滑
    #
    #                 p_next = v.entry_path[min(idx + 1, len(v.entry_path) - 1)]
    #
    #                 v.angle_to_draw = math.atan2(p_next[1] - v.visual_y, p_next[0] - v.visual_x)
    #
    #             else:
    #
    #                 # 弧线走完，强行同步角度进入环岛
    #
    #                 # dx, dy = v.visual_x - 400, v.visual_y - 400
    #                 #
    #                 # v.current_angle = math.atan2(dy, dx)
    #
    #                 v.state = "CIRCULATING"
    #
    #                 continue
    #
    #         elif v.state == "CIRCULATING":
    #             v.v = max(0, v.v + v.a * DT)
    #             v.v = min(v.v, 15.0)
    #             v.current_angle -= (v.v / (R + LANE_OFFSET)) * DT
    #             v.current_angle %= (2 * np.pi)
    #
    #             angle_to_exit = (v.current_angle - v.end_angle) % (2 * np.pi)
    #             if angle_to_exit < 0.2:
    #                 v.state = "EXITING"
    #                 v.exit_path = self.generate_exit_path(v, R)
    #                 v.path_index = 0
    #                 continue
    #
    #
    #         elif v.state == "EXITING":
    #
    #             # 强制压制转弯速度，不许超过 5
    #
    #             v.v = min(v.v + v.a * DT, 5.0)
    #
    #             v.v = max(v.v, 1.0)  # 保底速度
    #
    #             # 降低 path_index 的增加速度。如果 0.5 还是快，就改成 0.3
    #
    #             v.path_index += 0.5
    #
    #             idx = int(v.path_index)
    #
    #             if idx < len(v.exit_path):
    #
    #                 v.visual_x, v.visual_y = v.exit_path[idx]
    #
    #                 # 更新 current_angle 和 dist_to_center 供 IDM 使用
    #
    #                 dx, dy = v.visual_x - 400, v.visual_y - 400
    #
    #                 v.dist_to_center = math.sqrt(dx ** 2 + dy ** 2)
    #
    #                 v.current_angle = math.atan2(dy, dx)
    #
    #             else:
    #
    #                 v.state = "STRAIGHT_OUT"
    #
    #                 continue
    #
    #         elif v.state == "STRAIGHT_OUT":
    #             v.v = max(0, v.v + v.a * DT)
    #             v.dist_to_center += v.v * DT
    #
    #             exit_angle = v.end_angle
    #             v.visual_x = CENTER_X + v.dist_to_center * math.cos(exit_angle) - LANE_OFFSET * math.sin(exit_angle)
    #             v.visual_y = CENTER_Y + v.dist_to_center * math.sin(exit_angle) + LANE_OFFSET * math.cos(exit_angle)
    #             v.angle_to_draw = exit_angle
    #
    #             if v.dist_to_center > 1000:
    #                 if v in self.vehicles: self.vehicles.remove(v)
    #
    #         # 同步逻辑坐标
    #         if v.state in ["APPROACHING", "CIRCULATING"]:
    #             self.get_coords(v)

    def update(self):
        R = 120
        DT = 0.02
        CENTER_X, CENTER_Y = 400, 400
        LANE_OFFSET = 25
        STOP_LINE_DISTANCE = 175  # 停止线位置

        # --- 补回 15 辆车限制 ---
        # 如果当前车数超过 15 辆，就暂时停止生成新车（这通常在 spawn 函数里处理）
        # 这里我们也做个安全检查，确保不会因为车太多导致系统崩溃
        if len(self.vehicles) > 15:
            # 如果你发现依然停死，可以尝试在这里减慢全局车速或增加礼让间距
            pass

        for v in self.vehicles[:]:
            # 1. 基础物理更新
            lead_v = self.get_lead_vehicle(v)
            v.a = v.update_acceleration(lead_v)

            # 强行解冻：只要前面没车或很远，且不用礼让，就给个力
            if v.v < 0.1:
                # 检查距离 (s) 是否足够
                dist_to_lead = 1000
                if lead_v:
                    if v.state == "APPROACHING":
                        dist_to_lead = v.dist_to_center - lead_v.dist_to_center
                    elif v.state == "CIRCULATING":
                        dist_to_lead = (v.current_angle - lead_v.current_angle) % (2 * math.pi) * R

                if dist_to_lead > 45:  # 如果前方有超过一个车身的空隙
                    if v.state != "APPROACHING" or not self.check_ring_conflict(v):
                        v.a = max(v.a, 0.6)

            v.v += v.a * DT
            v.v = max(0, v.v)

            # 2. 状态机逻辑
            if v.state == "APPROACHING":
                # 核心修正：等候时不但看环岛，也要看前车
                # 如果前车还没走，或者环岛有车，就停下
                needs_to_stop = self.check_ring_conflict(v)

                # 如果前面已经有车在排队（lead_v 离中心更近）
                if lead_v and (v.dist_to_center - lead_v.dist_to_center < 50):
                    needs_to_stop = True

                count_limit_ok = len(self.vehicles) < 15
                no_conflict = not self.check_ring_conflict(v)

                # 重新计算场内实时车数（排除已经在 STRAIGHT_OUT 准备消失的车）
                active_vehicles = [veh for veh in self.vehicles if veh.state != "STRAIGHT_OUT"]
                count_limit_ok = len(active_vehicles) < 15
                no_conflict = not self.check_ring_conflict(v)

                if v.dist_to_center <= STOP_LINE_DISTANCE:
                    if count_limit_ok and no_conflict:
                        # --- 唤醒补丁 ---
                        v.state = "ENTERING"
                        v.entry_path = self.generate_entry_path(v, R)
                        v.path_index = 0
                        v.v = max(v.v, 2.0)  # 进场瞬间给个初速度，防止死在起步线上
                        continue
                    else:
                        v.v = 0
                        v.a = 0
                        v.dist_to_center = STOP_LINE_DISTANCE
                else:
                    # 还没到停止线，正常行驶
                    v.dist_to_center -= max(v.v, 0.5) * DT  # 给个极小的保底位移，防止在直线段冻死

                self.get_coords(v)
                v.angle_to_draw = v.start_angle

            elif v.state == "ENTERING":
                v.v = min(max(v.v, 1.8), 6.0)  # 略微提高保底速度到 1.8，防止在路口犹豫
                v.path_index += 0.8
                idx = int(v.path_index)
                if idx < len(v.entry_path) - 1:
                    v.visual_x, v.visual_y = v.entry_path[idx]
                    p_next = v.entry_path[idx + 1]
                    v.angle_to_draw = math.atan2(p_next[1] - v.visual_y, p_next[0] - v.visual_x)
                    dx, dy = v.visual_x - CENTER_X, v.visual_y - CENTER_Y
                    v.current_angle = math.atan2(dy, dx)
                else:
                    v.state = "CIRCULATING"

            elif v.state == "CIRCULATING":
                v.v = min(v.v, 10.0)
                v.current_angle -= (v.v / (R + LANE_OFFSET)) * DT
                v.current_angle %= (2 * np.pi)
                self.get_coords(v)
                v.angle_to_draw = v.current_angle - math.pi / 2

                angle_to_exit = (v.current_angle - v.end_angle) % (2 * np.pi)
                if angle_to_exit < 0.3:
                    v.state = "EXITING"
                    v.exit_path = self.generate_exit_path(v, R)
                    v.path_index = 0
                    continue


            elif v.state == "EXITING":

                # --- 强行清场补丁 ---

                # 在离场弧线上，我们不再完全依赖 v.a (IDM)

                # 只要进入了退出程序，就给它一个恒定的退出速度，不许停！

                v.v = max(v.v, 2.0)  # 强制最小速度 2.0，哪怕前面有车也慢慢蹭出去

                v.v = min(v.v, 5.0)  # 同时限速，防止飞出去

                v.path_index += 0.8

                idx = int(v.path_index)

                if idx < len(v.exit_path) - 1:

                    v.visual_x, v.visual_y = v.exit_path[idx]

                    p_next = v.exit_path[idx + 1]

                    v.angle_to_draw = math.atan2(p_next[1] - v.visual_y, p_next[0] - v.visual_x)

                    dx, dy = v.visual_x - 400, v.visual_y - 400

                    v.dist_to_center = math.sqrt(dx ** 2 + dy ** 2)

                else:

                    v.state = "STRAIGHT_OUT"

            elif v.state == "STRAIGHT_OUT":
                v.dist_to_center += v.v * DT
                exit_angle = v.end_angle
                v.visual_x = CENTER_X + v.dist_to_center * math.cos(exit_angle) - LANE_OFFSET * math.sin(exit_angle)
                v.visual_y = CENTER_Y + v.dist_to_center * math.sin(exit_angle) + LANE_OFFSET * math.cos(exit_angle)
                v.angle_to_draw = exit_angle

                if v.dist_to_center > 1000:
                    if v in self.vehicles: self.vehicles.remove(v)

    def generate_entry_path(self, v, R):
        CENTER = 400
        LANE_OFFSET = 25
        TARGET_R = R + LANE_OFFSET  # 目标永远是外圈 (145)

        # 1. 起点 P0: 严格锁定在当前车道的右侧边缘
        # 不要信任当前的 visual_x，手动根据 start_angle 计算它应该在的位置
        p0 = np.array([
            CENTER + (R + 40) * np.cos(v.start_angle) + LANE_OFFSET * np.cos(v.start_angle - np.pi / 2),
            CENTER + (R + 40) * np.sin(v.start_angle) + LANE_OFFSET * np.sin(v.start_angle - np.pi / 2)
        ])

        # 2. 终点 P2: 关键点！我们要把进入点往“下游”挪动
        # 减去约 20 度 (0.35 弧度)，让车子斜着切入，而不是对着圆心冲
        entry_angle = v.start_angle - 0.35
        p2 = np.array([
            CENTER + TARGET_R * np.cos(entry_angle),
            CENTER + TARGET_R * np.sin(entry_angle)
        ])

        # 3. 控制点 P1: 位于 P0 的正前方，确保车子先直行一小段
        fwd_direction = np.array([-np.cos(v.start_angle), -np.sin(v.start_angle)])
        p1 = p0 + fwd_direction * 25

        # 生成路径
        path = []
        for t in np.linspace(0, 1, 100):  # 增加点数让曲线更平滑
            pt = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2
            path.append(pt)

        # 强制同步：把车子的物理角度也对准 entry_angle
        # 这样当它结束弧线进入环岛时，位置和角度是完美的
        v.current_angle = entry_angle
        v.visual_x, v.visual_y = p0[0], p0[1]

        return path

    def generate_exit_path(self, v, R):
        LANE_OFFSET = 25
        # P0: 车辆当前在圆周上的视觉位置
        p0 = np.array([v.visual_x, v.visual_y])

        # 这里的 v.current_angle 是车辆在圆周上的角度位置
        # 在逆时针环岛中，切线方向是 v.current_angle - pi/2
        tangent_angle = v.current_angle - np.pi / 2

        # P1: 控制点。我们让它沿着切线方向延伸一段距离
        # 距离（50-70）决定了转向的平缓程度，越大越平缓
        p1_dist = 20
        p1 = p0 + np.array([p1_dist * math.cos(tangent_angle),
                            p1_dist * math.sin(tangent_angle)])

        # P2: 最终驶入直线车道的靠右点 (R + 100 处)
        exit_angle = v.end_angle
        target_r = R + 100
        p2 = np.array([
            400 + target_r * math.cos(exit_angle) - LANE_OFFSET * math.sin(exit_angle),
            400 + target_r * math.sin(exit_angle) + LANE_OFFSET * math.cos(exit_angle)
        ])

        # 生成贝塞尔曲线点
        path = []
        for t in np.linspace(0, 1, 100):
            pt = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2
            path.append(pt)
        return path

    def check_ring_conflict(self, v):
        for other in self.vehicles:
            if other.state in ["CIRCULATING", "ENTERING"]:
                # 计算环岛车相对于我入场点的角度差
                d_angle = (v.start_angle - other.current_angle) % (2 * math.pi)
                # 0.4 弧度大约是 20 度，只要它不在我正前方这 20 度内，我就敢进！
                if 0 < d_angle < 0.4:
                    return True
        return False

    def get_lead_vehicle(self, v):
        lead_v = None
        min_dist = float('inf')

        for other in self.vehicles:
            if v == other: continue

            # --- 核心逻辑：空间隔离 ---
            if v.state in ["ENTERING", "CIRCULATING", "EXITING"]:
                # 环岛内的车，只看同样在环岛系统里的车
                if other.state not in ["ENTERING", "CIRCULATING", "EXITING"]:
                    continue

                # 这里的 R 对应你的环岛半径 120
                d_angle = (v.current_angle - other.current_angle) % (2 * math.pi)
                if 0 < d_angle < 0.6:  # 只看前方 35 度
                    dist = d_angle * 120
                    if dist < min_dist:
                        min_dist, lead_v = dist, other

            elif v.state == "APPROACHING":
                # 引道上的车，只看同路口且同样在引道上的前车
                if other.state == "APPROACHING" and v.start_angle == other.start_angle:
                    dist = v.dist_to_center - other.dist_to_center
                    if 0 < dist < min_dist:
                        min_dist, lead_v = dist, other

        return lead_v

    def _draw_roundabout(self):
        # --- 1. 基础参数定义 ---
        ROAD_GRAY = (50, 50, 50)
        GRASS_GREEN = (10, 80, 10)
        LINE_WHITE = (255, 255, 255)
        CENTER_YELLOW = (200, 200, 0)

        total_road_width = 100
        outer_radius = R + (total_road_width // 2)  # 环岛路面外边缘 (R+50)
        inner_radius = R - (total_road_width // 8)  # 绿化带边缘 (R-50)

        # --- 2. 绘制引道路面 (最底层) ---
        road_half_w = 40
        # 横向引道
        pygame.draw.rect(self.screen, ROAD_GRAY, (0, CENTER - road_half_w, WIDTH, road_half_w * 2))
        # 纵向引道
        pygame.draw.rect(self.screen, ROAD_GRAY, (CENTER - road_half_w, 0, road_half_w * 2, HEIGHT))

        # --- 3. 绘制环岛主体 (覆盖十字路口中心) ---
        # 环岛灰色路面
        pygame.draw.circle(self.screen, ROAD_GRAY, (CENTER, CENTER), outer_radius)
        # 中心岛绿化带
        pygame.draw.circle(self.screen, GRASS_GREEN, (CENTER, CENTER), inner_radius)
        # 环岛中间白色分界线 (R)
        pygame.draw.circle(self.screen, LINE_WHITE, (CENTER, CENTER), R, 2)

        # --- 4. 绘制引道黄线 (截断逻辑) ---
        # 黄线只画到环岛外边缘 (outer_radius) 为止，防止贯穿环岛

        # 水平左侧黄线
        pygame.draw.line(self.screen, CENTER_YELLOW, (0, CENTER), (CENTER - outer_radius, CENTER), 2)
        # 水平右侧黄线
        pygame.draw.line(self.screen, CENTER_YELLOW, (CENTER + outer_radius, CENTER), (WIDTH, CENTER), 2)
        # 垂直上方黄线
        pygame.draw.line(self.screen, CENTER_YELLOW, (CENTER, 0), (CENTER, CENTER - outer_radius), 2)
        # 垂直下方黄线
        pygame.draw.line(self.screen, CENTER_YELLOW, (CENTER, CENTER + outer_radius), (CENTER, HEIGHT), 2)

    def draw(self):
        self.screen.fill((220, 220, 220))  # 浅背景色
        self._draw_roundabout()  # 绘制路面

        for v in self.vehicles:
            x, y = self.get_coords(v)

            # --- 核心：计算车辆朝向 (Heading) ---
            # 进场和出场朝向是固定的角度，环岛内朝向是切线方向
            if v.state == "APPROACHING":
                angle = v.start_angle + np.pi  # 朝向中心
            elif v.state == "CIRCULATING":
                # 切线方向 = 当前角度 + 90度 (逆时针)
                angle = getattr(v, 'current_angle', v.start_angle) + np.pi / 2
            else:  # EXITING
                angle = v.end_angle  # 背离中心

            # --- 绘制长方形车身 ---
            car_length = 24
            car_width = 14

            # 创建一个可以旋转的 Surface
            car_surface = pygame.Surface((car_length, car_width), pygame.SRCALPHA)
            color = (200, 30, 30) if v.type == "aggressive" else (30, 80, 200)  # 深红/深蓝

            # 画车身矩形
            pygame.draw.rect(car_surface, color, (0, 0, car_length, car_width), border_radius=3)
            # 画一个深色矩形代表前挡风玻璃（区分车头车尾）
            pygame.draw.rect(car_surface, (50, 50, 50), (car_length - 8, 2, 6, car_width - 4))

            # 旋转 Surface (pygame 逆时针旋转角度为度数)
            angle_deg = np.degrees(-v.angle_to_draw)
            rotated_car = pygame.transform.rotate(car_surface, angle_deg)
            rect = rotated_car.get_rect(center=(int(x), int(y)))

            self.screen.blit(rotated_car, rect)

    def draw_dashboard(self, screen):
        # 1. 绘制背景板
        overlay = pygame.Surface((300, 220))
        overlay.set_alpha(180)
        overlay.fill((40, 40, 40))
        screen.blit(overlay, (10, 10))

        # 2. 显示实时指标
        font = pygame.font.SysFont("Arial", 18)
        stats = [
            f"Total Conflicts: {self.total_conflicts}",
            f"Active Vehicles: {len(self.vehicles)}",
            f"Avg Speed: {self.get_avg_speed():.1f}",
            "-----------------------",
            f"[1/2] Car Max V: {self.config['car_max_v']}",
            f"[3/4] Safe Gap: {self.config['safe_gap']}",
            f"[5/6] Yield Angle: {self.config['yield_angle']:.1f}",
            "Press keys to adjust values"
        ]

        for i, text in enumerate(stats):
            color = (255, 255, 255) if i < 4 else (0, 255, 127)
            txt_surface = font.render(text, True, color)
            screen.blit(txt_surface, (20, 20 + i * 25))

    def export_data(self):
        """将所有统计数据导出为 CSV"""
        # 合并数据：已离开的车辆 + 当前还在场上的车辆
        final_list = list(self.data_logs)  # 复制一份档案库

        for v in self.vehicles:
            final_list.append({
                'id': v.id,
                'type': v.type,
                'wait_time': round(v.wait_time, 2),
                'conflicts': v.conflict_count,  # <-- 这里也要加，否则报错
                'status': 'in_sim'
            })

        if not final_list:
            print("没有记录到任何车辆数据！")
            return

        filename = "traffic_research_results.csv"
        keys = final_list[0].keys()

        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                dict_writer = csv.DictWriter(f, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(final_list)
            print(f"数据导出成功！文件名: {filename}")
        except PermissionError:
            print("错误：无法保存文件。请检查 CSV 文件是否被 Excel 打开了？")

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("正在自动保存数据...")
                self.export_data()
                self.running = False  # 确保类里有 self.running 属性

            if event.type == pygame.KEYDOWN:
                # S 键保存数据
                if event.key == pygame.K_s:
                    self.export_data()

                # --- 实验参数实时调整 ---
                # 1 & 2: 调整最高速
                if event.key == pygame.K_1:
                    self.config['car_max_v'] += 2
                elif event.key == pygame.K_2:
                    self.config['car_max_v'] = max(10, self.config['car_max_v'] - 2)

                # 3 & 4: 调整安全跟车距离
                elif event.key == pygame.K_3:
                    self.config['safe_gap'] += 5
                elif event.key == pygame.K_4:
                    self.key_3_4_logic = True  # 标记位（可选）
                    self.config['safe_gap'] = max(20, self.config['safe_gap'] - 5)

                # 5 & 6: 调整礼让角度 (敏感度)
                elif event.key == pygame.K_5:
                    self.config['yield_angle'] += 0.1
                elif event.key == pygame.K_6:
                    self.config['yield_angle'] = max(0.5, self.config['yield_angle'] - 0.1)

    def run(self):
        # 移除多余的 clock = pygame.time.Clock()，使用 self.clock
        running = True
        while running:
            # 1. 专门处理所有输入事件
            self.handle_events()

            # 2. 车辆生成逻辑
            self.spawn_timer += 1
            if self.spawn_timer > 15:
                self.spawn_vehicle()
                self.spawn_timer = 0

            # 3. 物理逻辑更新
            self.update()

            # 4. 绘图渲染
            self.draw()  # 画背景、道路和车辆
            self.draw_dashboard(self.screen)  # 在最上层画仪表盘
            # --- 调试绘图开始 ---
            # 必须遍历 self.vehicles 才能拿到每一辆车 v
            # for v in self.vehicles:
            #     # 只为处于 ENTERING 状态的车画出红点路径
            #     if v.state == "ENTERING" and hasattr(v, 'entry_path'):
            #         for p in v.entry_path:
            #             # 使用 self.screen，并将坐标转为整数
            #             pygame.draw.circle(self.screen, (255, 0, 0), (int(p[0]), int(p[1])), 2)
            #
            #     # 【额外建议】给每辆车画一个小绿点代表其 visual_x/y 的实时位置
            #     # 如果绿点偏离了红点路径，说明 get_coords 正在干扰路径
            #     pygame.draw.circle(self.screen, (0, 255, 0), (int(v.visual_x), int(v.visual_y)), 3)

            # 画出环岛的理想轨道（绿色大圆圈），确认 lane_offset=25 是否对齐
            # pygame.draw.circle(self.screen, (0, 255, 0), (400, 400), 120 + 25, 1)
            # --- 调试绘图结束 ---
            # 5. 刷新屏幕
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

if __name__ == "__main__":
    sim = AdvancedSim()
    sim.run()