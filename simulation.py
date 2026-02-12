import os
import sys
import csv
from datetime import datetime
import math
import pygame
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib
import seaborn as sns  # 如果没有 sns 就用 plt.hist
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
        if not os.path.exists('report'):
            os.makedirs('report')
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
        self.clock = pygame.time.Clock()
        self.vehicles = []
        self.spawn_timer = 0
        # 入口/出口角度定义: 0:右, pi/2:下, pi:左, 3pi/2:上
        self.directions = [0, np.pi / 2, np.pi, 3 * np.pi / 2]

        self.vehicles = []
        self.data_logs = []  # 新增：用于存放已离开车辆的数据
        self.spawn_timer = 0

        self.total_conflicts = 0  # 全局冲突计数器
        self.safety_threshold = 2.0  # 定义危险距离（米）：小于2米视为冲突

        self.weight_aggressive = 5  # 激进车初始权重
        self.weight_conservative = 5  # 保守车初始权重

        self.config = {
            "car_max_v": 25.0,  # 红车最大速度
            "truck_max_v": 15.0,  # 蓝车最大速度
            "safe_gap": 40.0,  # 基础安全跟车距离 (像素)
            "yield_angle": 1,  # 入场礼让判定弧度 (越大越保守)
            "spawn_rate": 0.5  # 车辆生成频率
        }

        # 数据统计容器
        self.data_logs = []
        self.stats_travel_times = []  # 存储每张车完成通行的总时间
        self.stats_conflicts = []  # 存储所有冲突点的坐标 (x, y)
        self.stats_flow_data = []  # 存储 (当前车数, 瞬时效率/吞吐量)
        self.stat_timer = 0

    def spawn_vehicle(self):
        # 1. 总数限制：场上总车数不超 25 (环岛15 + 4路口*2-3辆)
        if len(self.vehicles) >= 25:
            return

        # 计算当前的概率分布
        total_w = self.weight_aggressive + self.weight_conservative
        if total_w == 0: return  # 防止除以0

        prob_agg = self.weight_aggressive / total_w

        # 抽签决定车型
        v_type = "aggressive" if random.random() < prob_agg else "conservative"

        # 2. 统计每个路口排队的人数
        lane_counts = {angle: 0 for angle in self.directions}
        for v in self.vehicles:
            if v.state == "APPROACHING":
                lane_counts[v.start_angle] += 1

        # 3. 找出目前排队最少的路口，且排队不能超过 5 辆
        available_lanes = [ang for ang, count in lane_counts.items() if count < 8]
        if not available_lanes:
            return

        selected_angle = random.choice(available_lanes)

        # 4. 安全间距检查：该路口最后一张车后方是否有位置
        too_close = any(
            v.dist_to_center > (ROAD_LEN + R - 50)
            for v in self.vehicles
            if v.state == "APPROACHING" and v.start_angle == selected_angle
        )

        if not too_close:
            v_type = random.choices(["aggressive", "conservative"], weights=[0.4, 0.6])[0]
            v = Vehicle(random.randint(1000, 9999), v_type)
            v.state = "APPROACHING"
            v.start_angle = selected_angle
            v.end_angle = random.choice([a for a in self.directions if a != selected_angle])
            v.dist_to_center = ROAD_LEN + R
            v.current_angle = v.start_angle
            v.wait_time = 0
            v.conflict_count = 0
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

    def update(self):
        R = 120
        DT = 0.02
        CENTER_X, CENTER_Y = 400, 400
        LANE_OFFSET = 25
        STOP_LINE_DISTANCE = 175

        # 计算环岛内的活跃车辆数
        in_ring_count = len([veh for veh in self.vehicles if veh.state in ["ENTERING", "CIRCULATING", "EXITING"]])

        for v in self.vehicles[:]:
            # 1. 基础物理更新
            lead_v = self.get_lead_vehicle(v)
            v.a = v.update_acceleration(lead_v)

            # 强行起步补丁
            if v.v < 0.2:
                if not lead_v or (v.state == "APPROACHING" and v.dist_to_center - lead_v.dist_to_center > 55):
                    if v.state != "APPROACHING" or not self.check_ring_conflict(v):
                        v.a = max(v.a, 0.8)
                self.wait_time += 0.016

            v.v += v.a * DT
            v.v = max(0, v.v)

            # 2. 状态机
            if v.state == "APPROACHING":
                # 入场门槛：环岛内小于 15 辆 且 门口没车
                can_enter_ring = (in_ring_count < 15) and (not self.check_ring_conflict(v))

                # 判定前车：如果前车在停止线没走，我也不能动
                if lead_v and (v.dist_to_center - lead_v.dist_to_center < 50):
                    can_enter_ring = False

                if v.dist_to_center <= STOP_LINE_DISTANCE:
                    if can_enter_ring:
                        v.state = "ENTERING"
                        v.entry_path = self.generate_entry_path(v, R)
                        v.path_index = 0
                        v.v = max(v.v, 2.5)  # 瞬时速度，防止卡死
                        in_ring_count += 1  # 实时更新计数
                        continue
                    else:
                        v.v = 0
                        v.dist_to_center = STOP_LINE_DISTANCE
                else:
                    v.dist_to_center -= v.v * DT

                # 同步位置
                self.get_coords(v)

            elif v.state == "ENTERING":
                v.v = min(max(v.v, 2.0), 6.0)  # 转弯保底 2.0
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
                v.v = max(v.v, 3.0)
                v.v = min(v.v, 12.0)
                v.current_angle -= (v.v / (R + LANE_OFFSET)) * DT
                v.current_angle %= (2 * np.pi)
                self.get_coords(v)
                v.angle_to_draw = v.current_angle - math.pi / 2

                # 检查是否到出口
                angle_to_exit = (v.current_angle - v.end_angle) % (2 * np.pi)
                if angle_to_exit < 0.25:
                    v.state = "EXITING"
                    v.exit_path = self.generate_exit_path(v, R)
                    v.path_index = 0

            elif v.state == "EXITING":
                v.v = max(v.v, 2.5)  # 强行排空，出口车就是大爷
                v.path_index += 0.8
                idx = int(v.path_index)
                if idx < len(v.exit_path) - 1:
                    v.visual_x, v.visual_y = v.exit_path[idx]
                    p_next = v.exit_path[idx + 1]
                    v.angle_to_draw = math.atan2(p_next[1] - v.visual_y, p_next[0] - v.visual_x)
                    dx, dy = v.visual_x - CENTER_X, v.visual_y - CENTER_Y
                    v.dist_to_center = math.sqrt(dx ** 2 + dy ** 2)
                else:
                    v.state = "STRAIGHT_OUT"

            elif v.state == "STRAIGHT_OUT":
                v.dist_to_center += v.v * DT
                exit_angle = v.end_angle
                # 重新计算坐标防止漂移
                v.visual_x = CENTER_X + v.dist_to_center * math.cos(exit_angle) - LANE_OFFSET * math.sin(exit_angle)
                v.visual_y = CENTER_Y + v.dist_to_center * math.sin(exit_angle) + LANE_OFFSET * math.cos(exit_angle)
                v.angle_to_draw = exit_angle

                if v.dist_to_center > 1000:
                    self.stats_travel_times.append(v.wait_time + v.enter_time)  # 或者是你定义的统计字段
                    if v in self.vehicles: self.vehicles.remove(v)

                duration = (pygame.time.get_ticks() - v.spawn_time) / 1000.0  # 转换为秒
                self.stats_travel_times.append(duration)

                # --- 2. 写入档案库 (关键：修复数据为空的问题) ---
                self.data_logs.append({
                    'id': v.id,
                    'type': v.type,
                    'travel_time': round(duration, 2),
                    'wait_time': round(v.wait_time, 2),
                    'conflicts': v.conflict_count,
                    'status': 'completed'
                })

        # 统计效率
        active_ring_vehicles = [v for v in self.vehicles if v.state == "CIRCULATING"]
        flow_count = len(active_ring_vehicles)
        if flow_count > 0:
            avg_speed = sum([v.v for v in active_ring_vehicles]) / flow_count
            self.stats_flow_data.append((flow_count, avg_speed))


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
            if other.state in ["CIRCULATING", "ENTERING", "EXITING"]:
                # 计算环岛车相对于我入场点的角度差
                d_angle = (v.start_angle - other.current_angle) % (2 * math.pi)
                # 0.3 弧度大约是 20 度，只要它不在我正前方这 20 度内，我就敢进！
                if d_angle < 0.5 or d_angle > (2 * math.pi - 0.2):
                    self.stats_conflicts.append((v.visual_x, v.visual_y))
                    return True
        return False

    def get_lead_vehicle(self, v):
        lead_v = None
        min_dist = float('inf')

        for other in self.vehicles:
            if v == other: continue

            if v.state in ["ENTERING", "CIRCULATING", "EXITING"]:
                if other.state not in ["ENTERING", "CIRCULATING", "EXITING"]:
                    continue

                # 逆时针环岛：前车的角度比我小
                # 计算 (我的角度 - 他的角度) % 2pi 得到的是我到他的顺时针距离
                # 在逆时针坐标系下，这正是我们要的前向弧长角度
                d_angle = (v.current_angle - other.current_angle) % (2 * math.pi)

                # 只关注前方 0.8 弧度（约 45 度）内的车，防止误判身后的车
                if 0 < d_angle < 0.8:
                    dist = d_angle * 145.0  # 使用车道半径
                    if dist < min_dist:
                        min_dist, lead_v = dist, other

            elif v.state == "APPROACHING":
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
            # 显示车辆速度
            # font = pygame.font.SysFont("Arial", 12)
            # txt = font.render(f"{v.state} v:{v.v:.1f}", True, (255, 255, 255))
            # self.screen.blit(txt, (v.visual_x, v.visual_y - 20))
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

    def draw_controls(self):
        # 1. 设置面板的基础坐标和大小
        panel_width = 320
        panel_height = 165
        start_x = SCREEN_SIZE - panel_width - 10
        start_y = 10

        # 2. 获取数据（确保 self.weight_aggressive 这些变量已经存在）
        w_agg = self.weight_aggressive
        w_con = self.weight_conservative
        total_w = w_agg + w_con
        ratio_percent = (w_agg / total_w * 100) if total_w > 0 else 0

        # 3. 准备字体 (在这里统一定义，解决“不认识”的问题)
        try:
            # 优先使用中文字体，没有就用系统默认
            main_font = pygame.font.SysFont("SimHei", 20)
            hint_font = pygame.font.SysFont("SimHei", 16)  # 这就是你要的 small font
        except:
            main_font = pygame.font.SysFont("arial", 18)
            hint_font = pygame.font.SysFont("arial", 14)

        # 4. 绘制半透明背景面板
        overlay = pygame.Surface((panel_width, panel_height))
        overlay.set_alpha(180)
        overlay.fill((0, 0, 0))
        self.screen.blit(overlay, (start_x, start_y))

        # 5. 画边框
        pygame.draw.rect(self.screen, (200, 200, 200), (start_x, start_y, panel_width, panel_height), 2)

        # 6. 渲染并绘制文字
        # 激进权重
        txt_agg = main_font.render(f"激进权重 (UP+/DOWN-): {w_agg}", True, (255, 50, 50))
        self.screen.blit(txt_agg, (start_x + 20, start_y + 15))

        # 保守权重
        txt_con = main_font.render(f"保守权重 (RIGHT+/LEFT-): {w_con}", True, (80, 80, 255))
        self.screen.blit(txt_con, (start_x + 20, start_y + 45))

        # 比例显示
        txt_ratio = main_font.render(f"生成倾向: {ratio_percent:.1f}% 激进", True, (255, 255, 255))
        self.screen.blit(txt_ratio, (start_x + 20, start_y + 75))

        # 底部提示文字 (使用刚才定义的 hint_font)
        # txt_hint = hint_font.render("按 [空格键] 生成统计分析图表", True, (200, 200, 200))
        # self.screen.blit(txt_hint, (start_x + 20, start_y + 135))
        self.screen.blit(hint_font.render("按 [数字 8] 汇统计图，导出 CSV 数据", True, (0, 255, 255)),
                         (start_x + 20, start_y + 135))

        # 7. 绘制彩色进度条
        bar_x, bar_y = start_x + 20, start_y + 110
        bar_w, bar_h = panel_width - 40, 15

        if total_w > 0:
            red_w = (w_agg / total_w) * bar_w
            # 底色蓝色
            pygame.draw.rect(self.screen, (50, 50, 200), (bar_x, bar_y, bar_w, bar_h))
            # 覆盖红色
            pygame.draw.rect(self.screen, (200, 50, 50), (bar_x, bar_y, red_w, bar_h))
            # 白色细边框
            pygame.draw.rect(self.screen, (255, 255, 255), (bar_x, bar_y, bar_w, bar_h), 1)

    def plot_results(self):
        matplotlib.use('TkAgg')

        if not self.stats_travel_times:
            print("警告：目前还没有车辆完成通行，没有数据可画！")
            return
        fig, axes = plt.subplots(1, 3, figsize=(18, 5))

        # 1. 通行时间分布图 (Histogram)
        axes[0].hist(self.stats_travel_times, bins=20, color='skyblue', edgecolor='black')
        axes[0].set_title("Travel Time Distribution")
        axes[0].set_xlabel("Time (s)")
        axes[0].set_ylabel("Vehicle Count")

        # 2. 冲突热力图 (Heatmap/Scatter)
        if self.stats_conflicts:
            cx, cy = zip(*self.stats_conflicts)
            hb = axes[1].hexbin(cx, cy, gridsize=30, cmap='YlOrRd', extent=[0, 800, 0, 800])
            axes[1].set_title("Conflict Heatmap (Real Position)")
            axes[1].invert_yaxis()  # 匹配 Pygame 的 Y 轴向下增长
            # 画一个环岛轮廓方便对比
            circle = plt.Circle((400, 400), 120, color='blue', fill=False, linestyle='--')
            axes[1].add_artist(circle)

        # 3. 流量-效率关系图 (Scatter/Line)
        if self.stats_flow_data:
            flows, effs = zip(*self.stats_flow_data)
            axes[2].scatter(flows, effs, alpha=0.5)
            axes[2].set_title("Flow vs Efficiency")
            axes[2].set_xlabel("Number of Vehicles")
            axes[2].set_ylabel("Avg Speed (Efficiency)")

        plt.tight_layout()
        plt.show()

        output_path = os.path.join('report', 'simulation_plots.png')
        plt.savefig(output_path, dpi=300)
        # plt.close()  # 记得关闭，防止内存占用
        print(f"✅ 统计图表已保存至: {output_path}")

    def export_data(self):
        """将所有统计数据导出为 CSV"""
        # 1. 整合已离开和还在场上的车辆
        final_list = list(self.data_logs)

        for v in self.vehicles:
            final_list.append({
                'id': v.id,
                'type': v.type,
                'travel_time': round((pygame.time.get_ticks() - v.spawn_time) / 1000.0, 2),
                'wait_time': round(v.wait_time, 2),
                'conflicts': v.conflict_count,
                'status': 'still_in_simulation'
            })

        if not final_list:
            print("没有记录到任何车辆数据，无法导出！")
            return

        filename = os.path.join('report', f"traffic_analysis_{pygame.time.get_ticks()}.csv")

        # 2. 这里的 keys 必须手动指定或确保 final_list 肯定有数据
        keys = ['id', 'type', 'travel_time', 'wait_time', 'conflicts', 'status']

        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                dict_writer = csv.DictWriter(f, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(final_list)
            print(f"数据已完整导出！共 {len(final_list)} 条记录。文件名: {filename}")
        except PermissionError:
            print("错误：文件被占用，请先关闭正在查看该 CSV 的 Excel 窗口！")

    def save_to_csv(self):

        # 创建一个带时间戳的文件名，防止覆盖之前的实验
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # 1. 保存流量-效率数据
        flow_filename = os.path.join('report', f'flow_data_{timestamp}.csv')
        with open(flow_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Number_of_Vehicles', 'Avg_Speed_Efficiency'])
            writer.writerows(self.stats_flow_data)

        # 2. 保存通行时间数据
        travel_filename = os.path.join('report', f'travel_time_{timestamp}.csv')
        with open(travel_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Travel_Time_Seconds'])
            for t in self.stats_travel_times:
                writer.writerow([t])

        print(f"数据已导出到 CSV 文件！时间戳: {timestamp}")

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("正在自动保存数据...")
                self.export_data()
                pygame.quit()  # 卸载 pygame 模块
                sys.exit()  # 强制终止 Python 进程，防止窗口卡死
                self.running = False  # 确保类里有 self.running 属性

            if event.type == pygame.KEYDOWN:
                print(f"Key pressed: {event.key}")
                # S 键保存数据
                if event.key == pygame.K_8:
                    print(">>> 正在导出全套研究数据至 report 文件夹...")
                    self.export_data()  # 存详细档案
                    self.save_to_csv()  # 存绘图原始数据
                    self.plot_results()  # 存仿真图表
                # 按 Q/A 调激进车比例
                if event.key == pygame.K_UP: self.weight_aggressive += 1
                if event.key == pygame.K_DOWN: self.weight_aggressive = max(0, self.weight_aggressive - 1)
                # 按 W/S 调保守车比例
                if event.key == pygame.K_RIGHT: self.weight_conservative += 1
                if event.key == pygame.K_LEFT: self.weight_conservative = max(0, self.weight_conservative - 1)

                # if event.key == pygame.K_SPACE:  # 按 P 键弹出图表
                #     print("--- 触发数据绘图 ---")
                #     self.plot_results()
                #     self.save_to_csv()
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
        try:
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
                self.draw_controls()
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
        except:
            pass
        finally:
            pygame.quit()

if __name__ == "__main__":
    sim = AdvancedSim()
    sim.run()