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

    # def get_coords(self, v):
    #     lane_offset = 25  # 靠右偏移
    #
    #     # --- 1. 计算基础中心点 (Base) ---
    #     if v.state == "APPROACHING":
    #         base_x = CENTER + v.dist_to_center * np.cos(v.start_angle)
    #         base_y = CENTER + v.dist_to_center * np.sin(v.start_angle)
    #         # 引道偏移：垂直于道路
    #         target_ox = lane_offset * np.cos(v.start_angle - np.pi / 2)
    #         target_oy = lane_offset * np.sin(v.start_angle - np.pi / 2)
    #         # 目标角度：直行
    #         target_angle = v.start_angle + np.pi
    #
    #         if v.direction == 'N':
    #             target_ox, target_oy = 400, 0
    #         elif v.direction == 'S':
    #             target_ox, target_oy = 400, 800
    #         elif v.direction == 'E':
    #             target_ox, target_oy = 800, 400
    #         elif v.direction == 'W':
    #             target_ox, target_oy = 0, 400
    #
    #     elif v.state == "CIRCULATING":
    #         target_ox, target_oy = 400, 400
    #         # 1. 基础圆周位置
    #         base_x = CENTER + v.dist_to_center * np.cos(v.current_angle)
    #         base_y = CENTER + v.dist_to_center * np.sin(v.current_angle)
    #
    #         # 2. 逆时针车头方向（拧正后的方向）
    #         target_angle = v.current_angle + np.pi / 2 + np.pi
    #
    #         # 3. 平滑入弯补偿：
    #         # 如果 v.dist_to_center 依然大于 R，说明还在入弯道上
    #         # 我们让 lane_offset 动态调整，防止车瞬间“弹”到侧边
    #         transition_factor = 1.0
    #         if v.dist_to_center > R:
    #             # 距离 R 越近，factor 越接近 1
    #             transition_factor = np.clip((R + 40 - v.dist_to_center) / 40, 0, 1)
    #
    #         current_lane_offset = lane_offset * transition_factor
    #
    #         target_ox = current_lane_offset * np.cos(v.current_angle)
    #         target_oy = current_lane_offset * np.sin(v.current_angle)
    #
    #     elif v.state == "EXITING":
    #         base_x = CENTER + v.dist_to_center * np.cos(v.end_angle)
    #         base_y = CENTER + v.dist_to_center * np.sin(v.end_angle)
    #         target_ox = lane_offset * np.cos(v.end_angle + np.pi / 2)
    #         target_oy = lane_offset * np.sin(v.end_angle + np.pi / 2)
    #         target_angle = v.end_angle
    #     elif v.state == "ENTERING":
    #         target_ox, target_oy = 400, 400
    #     # --- 2. 核心补丁：偏移向量锁死 (解决侧跳) ---
    #     if not hasattr(v, 'smooth_ox') or v.smooth_ox is None:
    #         v.smooth_ox, v.smooth_oy = target_ox, target_oy
    #
    #     # 强制大幅平滑偏移向量。0.05 的系数会让并线动作非常缓慢自然
    #     v.smooth_ox += (target_ox - v.smooth_ox) * 0.05
    #     v.smooth_oy += (target_oy - v.smooth_oy) * 0.05
    #
    #     # --- 3. 视觉位置全局平滑 ---
    #     raw_x = base_x + v.smooth_ox
    #     raw_y = base_y + v.smooth_oy
    #
    #     if not hasattr(v, 'visual_x') or v.visual_x is None:
    #         v.visual_x, v.visual_y = raw_x, raw_y
    #
    #     # 这里就是最后的防线：即便 raw_x 突然跳了 20 像素往黄线靠
    #     # visual_x 也只会以 0.1 的速度慢慢滑过去，视觉上就是“缓慢转向”
    #     v.visual_x += (raw_x - v.visual_x) * 0.1
    #     v.visual_y += (raw_y - v.visual_y) * 0.1
    #
    #     # 角度平滑
    #     if not hasattr(v, 'visual_angle'): v.visual_angle = target_angle
    #     adiff = (target_angle - v.visual_angle + np.pi) % (2 * np.pi) - np.pi
    #     v.visual_angle += adiff * 0.05
    #     v.angle_to_draw = v.visual_angle
    #
    #     return v.visual_x, v.visual_y

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

        # 1. 统计环岛内当前的车辆总数
        circulating_count = len([v for v in self.vehicles if v.state == "CIRCULATING"])

        for v in self.vehicles[:]:
            # --- 获取参数 ---
            max_speed = self.config["car_max_v"] if v.type == 'car' else self.config["truck_max_v"]
            accel_power = 1.2 if v.type == 'car' else 0.6
            max_ring_v = 15.0

            # --- 2. 状态：APPROACHING (引道) ---
            if v.state == "APPROACHING":
                is_blocked_by_ring = False
                # 只有靠近路口才检查礼让
                if v.dist_to_center < R + 100:
                    if circulating_count >= 15:
                        is_blocked_by_ring = True
                    else:
                        for other in self.vehicles:
                            if other.state == "CIRCULATING":
                                d_angle = (v.start_angle - other.current_angle) % (2 * np.pi)
                                if 0 < d_angle < 0.8:  # 礼让角度区间
                                    is_blocked_by_ring = True
                                    break

                # 前车距离判定
                lead_v = self.get_lead_vehicle(v)
                dist_to_lead = (v.dist_to_center - lead_v.dist_to_center) if (
                        lead_v and lead_v.state == "APPROACHING") else 500

                # 停止判定
                stop_line = R + 45
                if v.dist_to_center <= stop_line and is_blocked_by_ring:
                    v.v = 0
                    v.dist_to_center = stop_line
                elif dist_to_lead < self.config["safe_gap"]:
                    v.v = 0
                else:
                    # 正常加速
                    v.v = min(v.v + accel_power, max_speed)
                    if v.dist_to_center < R + 60: v.v = min(v.v, 8.0)

                # 更新位移（注意：这里只写一次减法）
                v.dist_to_center -= v.v * DT

                # 状态切换：确保切换点灵敏
                if v.dist_to_center <= R + 42 and not is_blocked_by_ring:
                    v.state = "ENTERING"
                    v.entry_path = self.generate_entry_path(v, R)
                    v.path_index = 0
                    if hasattr(v, 'smooth_ox'): delattr(v, 'smooth_ox')
                    if hasattr(v, 'smooth_oy'): delattr(v, 'smooth_oy')
                    continue

            # --- 3. 状态：ENTERING (弧线过渡) ---
            elif v.state == "ENTERING":
                if v.path_index < len(v.entry_path) - 1:
                    v.path_index += 1
                    idx = int(v.path_index)
                    p_current = np.array(v.entry_path[idx])
                    v.visual_x, v.visual_y = p_current[0], p_current[1]
                    p_next = np.array(v.entry_path[min(idx + 1, len(v.entry_path) - 1)])
                    v.angle_to_draw = math.atan2(p_next[1] - p_current[1], p_next[0] - p_current[0]) + np.pi
                else:
                    v.state = "CIRCULATING"
                    dx, dy = v.visual_x - CENTER_X, v.visual_y - CENTER_Y
                    v.current_angle = math.atan2(dy, dx)
                    continue

            # --- 4. 状态：CIRCULATING (环岛内) ---
            elif v.state == "CIRCULATING":
                v.dist_to_center = math.sqrt(R ** 2 + LANE_OFFSET ** 2)

                # 出口判定：一旦到了出口，立刻切换，优先级最高
                angle_to_exit = (v.current_angle - v.end_angle) % (2 * np.pi)
                if angle_to_exit < 0.25:  # 适当扩大判定区
                    v.state = "EXITING"
                    # 核心修复：退出瞬间同步 dist_to_center，确保 get_coords 不会把车拉回圆心
                    v.exit_path = self.generate_exit_path(v, R)  # 生成出口弧线
                    v.path_index = 0
                    continue

                # 环岛内跟车
                ring_lead = None
                min_ring_dist = 1000
                for other in self.vehicles:
                    if other == v or other.state not in ["CIRCULATING", "EXITING"]: continue
                    d_angle = (v.current_angle - other.current_angle) % (2 * np.pi)
                    if 0 < d_angle < 0.8:
                        dist = d_angle * (R + LANE_OFFSET)
                        if dist < min_ring_dist:
                            min_ring_dist, ring_lead = dist, other

                if ring_lead:
                    target_v = ring_lead.v * (min_ring_dist / 50) if min_ring_dist < 50 else max_ring_v
                    v.v = v.v * 0.5 + target_v * 0.5
                    if min_ring_dist < 25: v.v = 0
                else:
                    v.v = min(v.v + 1.0, max_ring_v)

                v.current_angle -= (v.v / (R + LANE_OFFSET)) * DT
                v.current_angle %= (2 * np.pi)

            # --- 5. 状态：EXITING (执行退出弧线) ---
            elif v.state == "EXITING":
                if v.path_index < len(v.exit_path) - 1:
                    v.v = min(v.v + 0.5, 12.0) # 这里的 12.0 是弧线上的限速，比直道低
                    v.path_index += 1
                    idx = int(v.path_index)
                    v.visual_x, v.visual_y = v.exit_path[idx]

                    # 更新朝向
                    p_next = v.exit_path[min(idx + 1, len(v.exit_path) - 1)]
                    v.angle_to_draw = math.atan2(p_next[1] - v.visual_y, p_next[0] - v.visual_x) + np.pi
                else:
                    # --- 【核心修复点】：无缝衔接 ---
                    v.state = "STRAIGHT_OUT"

                    # 不再手动设定 R + 81，而是根据当前弧线终点到中心的距离，
                    # 反推 dist_to_center，实现物理上的完全重合。
                    dx = v.visual_x - CENTER_X
                    dy = v.visual_y - CENTER_Y
                    # 注意：这里要投影到出口射线上，最简单的方法是直接算欧式距离
                    v.dist_to_center = math.sqrt(dx ** 2 + dy ** 2)
                    continue

            # --- 6. 状态：STRAIGHT_OUT (直线冲出) ---
            elif v.state == "STRAIGHT_OUT":
                v.v = min(v.v + 1.5, 25.0)
                v.dist_to_center += v.v * DT

                # 这里的计算必须和 generate_exit_path 的 P2 算法完全一致
                exit_angle = v.end_angle
                v.visual_x = 400 + v.dist_to_center * math.cos(exit_angle) - 25 * math.sin(exit_angle)
                v.visual_y = 400 + v.dist_to_center * math.sin(exit_angle) + 25 * math.cos(exit_angle)

                v.angle_to_draw = exit_angle

                if v.dist_to_center > 1000:
                    if v in self.vehicles: self.vehicles.remove(v)

            # --- 6. 统一坐标更新 ---
            if v.state != "EXITING":
                self.get_coords(v)

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

    def get_lead_vehicle(self, v):
        lead = None
        min_dist = 1000
        R = 120  # 请确保半径一致

        for other in self.vehicles:
            if v.id == other.id: continue

            # --- 场景 1: 引道跟车 (修正点：使用 round 解决浮点数 1.5700001 的误差) ---
            if v.state == "APPROACHING" and other.state == "APPROACHING":
                if round(v.start_angle, 2) == round(other.start_angle, 2):
                    if v.dist_to_center > other.dist_to_center:
                        d = v.dist_to_center - other.dist_to_center
                        if d < min_dist:
                            min_dist = d
                            lead = other

            # --- 场景 2: 引道让环岛 (修正点：逆时针取模判定) ---
            elif v.state == "APPROACHING" and other.state == "CIRCULATING":
                # 计算环岛车距离路口的弧度差
                d_angle = (v.start_angle - other.current_angle) % (2 * np.pi)
                # 只有在 0.1 到 1.2 弧度（路口左侧一小段）才算障碍
                if 0.1 < d_angle < 1.2:
                    d = (v.dist_to_center - R) + (d_angle * R)
                    if d < min_dist:
                        min_dist = d
                        lead = other
        return lead

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
            for v in self.vehicles:
                # 只为处于 ENTERING 状态的车画出红点路径
                if v.state == "ENTERING" and hasattr(v, 'entry_path'):
                    for p in v.entry_path:
                        # 使用 self.screen，并将坐标转为整数
                        pygame.draw.circle(self.screen, (255, 0, 0), (int(p[0]), int(p[1])), 2)

                # 【额外建议】给每辆车画一个小绿点代表其 visual_x/y 的实时位置
                # 如果绿点偏离了红点路径，说明 get_coords 正在干扰路径
                pygame.draw.circle(self.screen, (0, 255, 0), (int(v.visual_x), int(v.visual_y)), 3)

            # 画出环岛的理想轨道（绿色大圆圈），确认 lane_offset=25 是否对齐
            pygame.draw.circle(self.screen, (0, 255, 0), (400, 400), 120 + 25, 1)
            # --- 调试绘图结束 ---
            # 5. 刷新屏幕
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

if __name__ == "__main__":
    sim = AdvancedSim()
    sim.run()