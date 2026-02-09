import csv
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

    def get_coords(self, v):
        lane_offset = 25  # 靠右偏移

        # --- 1. 计算基础中心点 (Base) ---
        if v.state == "APPROACHING":
            base_x = CENTER + v.dist_to_center * np.cos(v.start_angle)
            base_y = CENTER + v.dist_to_center * np.sin(v.start_angle)
            # 引道偏移：垂直于道路
            target_ox = lane_offset * np.cos(v.start_angle - np.pi / 2)
            target_oy = lane_offset * np.sin(v.start_angle - np.pi / 2)
            # 目标角度：直行
            target_angle = v.start_angle + np.pi

        elif v.state == "CIRCULATING":
            # 1. 基础圆周位置
            base_x = CENTER + v.dist_to_center * np.cos(v.current_angle)
            base_y = CENTER + v.dist_to_center * np.sin(v.current_angle)

            # 2. 逆时针车头方向（拧正后的方向）
            target_angle = v.current_angle + np.pi / 2 + np.pi

            # 3. 平滑入弯补偿：
            # 如果 v.dist_to_center 依然大于 R，说明还在入弯道上
            # 我们让 lane_offset 动态调整，防止车瞬间“弹”到侧边
            transition_factor = 1.0
            if v.dist_to_center > R:
                # 距离 R 越近，factor 越接近 1
                transition_factor = np.clip((R + 40 - v.dist_to_center) / 40, 0, 1)

            current_lane_offset = lane_offset * transition_factor

            target_ox = current_lane_offset * np.cos(v.current_angle)
            target_oy = current_lane_offset * np.sin(v.current_angle)

        elif v.state == "EXITING":
            base_x = CENTER + v.dist_to_center * np.cos(v.end_angle)
            base_y = CENTER + v.dist_to_center * np.sin(v.end_angle)
            target_ox = lane_offset * np.cos(v.end_angle + np.pi / 2)
            target_oy = lane_offset * np.sin(v.end_angle + np.pi / 2)
            target_angle = v.end_angle

        # --- 2. 核心补丁：偏移向量锁死 (解决侧跳) ---
        if not hasattr(v, 'smooth_ox') or v.smooth_ox is None:
            v.smooth_ox, v.smooth_oy = target_ox, target_oy

        # 强制大幅平滑偏移向量。0.05 的系数会让并线动作非常缓慢自然
        v.smooth_ox += (target_ox - v.smooth_ox) * 0.05
        v.smooth_oy += (target_oy - v.smooth_oy) * 0.05

        # --- 3. 视觉位置全局平滑 ---
        raw_x = base_x + v.smooth_ox
        raw_y = base_y + v.smooth_oy

        if not hasattr(v, 'visual_x') or v.visual_x is None:
            v.visual_x, v.visual_y = raw_x, raw_y

        # 这里就是最后的防线：即便 raw_x 突然跳了 20 像素往黄线靠
        # visual_x 也只会以 0.1 的速度慢慢滑过去，视觉上就是“缓慢转向”
        v.visual_x += (raw_x - v.visual_x) * 0.1
        v.visual_y += (raw_y - v.visual_y) * 0.1

        # 角度平滑
        if not hasattr(v, 'visual_angle'): v.visual_angle = target_angle
        adiff = (target_angle - v.visual_angle + np.pi) % (2 * np.pi) - np.pi
        v.visual_angle += adiff * 0.05
        v.angle_to_draw = v.visual_angle

        return v.visual_x, v.visual_y

    def update(self):
        R = 120
        DT = 0.02

        for v in self.vehicles[:]:
            # --- 1. 获取前车 ---
            lead_v = self.get_lead_vehicle(v)
            actual_gap = 1000
            # --- 2. 状态：APPROACHING (引道) ---
            if v.state == "APPROACHING":
                is_blocked_by_ring = False
                if v.dist_to_center < R + 70:
                    for other in self.vehicles:
                        if other.state == "CIRCULATING":
                            d_angle = (v.start_angle - other.current_angle) % (2 * np.pi)
                            if 0.05 < d_angle < 0.9 or d_angle > (2 * np.pi - 0.2):
                                is_blocked_by_ring = True
                                is_blocked_by_ring = True
                                break
                if (v.dist_to_center <= R + 45 and is_blocked_by_ring):
                    v.v = 0  # 停下
                else:
                    stop_line = R + 60
                    # gap = (v.dist_to_center - lead_v.dist_to_center) if (lead_v and lead_v.state == "APPROACHING") else 1000

                    if v.v < 1.0: v.wait_time += DT

                    # --- 物理逻辑控制：If (停) Else (冲) ---
                    if (v.dist_to_center <= stop_line and is_blocked_by_ring):
                        v.v = 0
                        v.a = 0
                        if is_blocked_by_ring: v.dist_to_center = stop_line
                    elif lead_v and lead_v.state == "APPROACHING" and (v.dist_to_center - lead_v.dist_to_center) < 40:
                        v.v = 0
                        v.a = 0
                    else:
                        # 重新计算 gap，如果没有前车，我们就按“前方无限远”处理，但速度受 max_speed 封顶
                        actual_gap = (v.dist_to_center - lead_v.dist_to_center) if (
                                    lead_v and lead_v.state == "APPROACHING") else 500
                        # --- 极速增强版参数 ---
                        # 红车最高速提至 25，蓝车提至 15 (显著加快)
                        max_speed = 25 if v.type == 'car' else 15
                        # 起步推力加大，红车 1.2，蓝车 0.6
                        accel_power = 1.2 if v.type == 'car' else 0.6

                        # 强力弹射起步：让车子从静止瞬间获得高初速
                        if v.v < 6.0:
                            v.v += accel_power * 4

                        if actual_gap < 200:  # 扩大感知范围，更早开始动态调速
                            # 让车子在靠近前车时也保持一个较高的动力感
                            target_v = max(0, (actual_gap - 35) / 150 * max_speed)
                            # 提高响应权重 (0.5)，让速度变化更灵敏
                            v.v = v.v * 0.5 + target_v * 0.5
                        else:
                            # 前方大路朝天，快速冲向最高速
                            v.v = min(v.v + accel_power * 2, max_speed)
                        # 增加补丁：如果快到路口且环岛有车，即便还没到停止线，也要强制减速
                        if v.dist_to_center < R + 100 and is_blocked_by_ring:
                            v.v = min(v.v, 5)  # 强行压低入场速度，防止撞击

                        v.dist_to_center -= v.v * DT

                    # 状态切换：确保切换瞬间不漂移
                    if v.dist_to_center <= R + 30:
                        last_x, last_y = self.get_coords(v)
                        v.state = "CIRCULATING"
                        v.current_angle = v.start_angle
                        v.visual_x, v.visual_y = last_x, last_y

            # --- 3. 状态：CIRCULATING (环岛内) ---
            elif v.state == "CIRCULATING":
                # --- [修复 A] 强力向心力：把车“焊”在 R=120 的轨道上 ---
                # 这里的修正必须足够快，才能抵消状态切换时的位移惯性
                # 我们用 0.15 的权重，让它在 3-5 帧内迅速回正
                v.dist_to_center += (R - v.dist_to_center) * 0.01

                # 2. 寻找环岛内的前车 (同在环岛内的车)
                ring_lead = None
                min_ring_dist = 1000
                for other in self.vehicles:
                    if other.id == v.id: continue

                    # 只有对方也在环岛内，或者正在退出时，才真正执行“跟车”
                    if other.state in ["CIRCULATING", "EXITING"]:
                        d_angle = (v.current_angle - other.current_angle) % (2 * np.pi)
                        dist = d_angle * R
                        if dist < min_ring_dist:
                            min_ring_dist = dist
                            ring_lead = other

                    if ring_lead and min_ring_dist < 40:
                        if min_ring_dist < 20:
                            v.v = 0
                        else:
                            v.v = 2.0  # 蠕动，不完全停死
                    else:
                        # 只要前面没车，强制加速，不准在环岛里发呆
                        max_ring_v = 18 if v.type == 'car' else 12
                        v.v = min(v.v + 0.8, max_ring_v)
                    # 情况1：前方也是环岛车
                    if other.state == "CIRCULATING":
                        d_angle = (v.current_angle - other.current_angle) % (2 * np.pi)
                        dist = d_angle * R

                    # 情况2：前方是正在驶出的车 (且它还没走远)
                    # 只有当对方就在你的出口附近时，才需要考虑它
                    elif other.state == "EXITING":
                        # 粗略判断：如果对方的出口角和我的当前角很近
                        d_angle = (v.current_angle - other.end_angle) % (2 * np.pi)
                        dist = d_angle * R
                    else:
                        dist = 1000

                    if dist < min_ring_dist:
                        min_ring_dist = dist
                        ring_lead = other

                # --- 速度控制逻辑 (保持不变) ---
                max_ring_v = 18 if v.type == 'car' else 12
                if ring_lead and min_ring_dist < 40:
                    # if min_ring_dist < 25:  # 离正在出去的车太近了，减速
                    #     v.v = v.v * 0.5
                    #     if min_ring_dist < 30: v.v = 0
                    # else:
                    #     target_ring_v = max(2, (min_ring_dist - 30) / 100 * max_ring_v)
                    #     v.v = v.v * 0.6 + target_ring_v * 0.4
                    if min_ring_dist < 25:  # 只有快贴上了才停
                        v.v = 0
                    else:
                        v.v = 2.0  # 给个保底蠕动速度，防止彻底卡死
                else:
                    v.v = min(v.v + 0.8, max_ring_v)

                # 4. 更新位置
                angular_speed = v.v / v.dist_to_center
                v.current_angle -= angular_speed * DT
                v.current_angle %= (2 * np.pi)

                # 5. 出口判定
                if (v.current_angle - v.end_angle) % (2 * np.pi) < 0.1:
                    last_x, last_y = self.get_coords(v)
                    v.state = "EXITING"
                    v.visual_x, v.visual_y = last_x, last_y

            # --- 4. 其余逻辑 (EXITING 和 冲突检测) ---
            elif v.state == "EXITING":
                # # 寻找前面也是 EXITING 且在同一个方向的车
                exit_lead = None
                for other in self.vehicles:
                    if other.id != v.id and other.state == "EXITING":
                        # 这里的判定比较简单：谁离中心远，谁在前面
                        if v.dist_to_center < other.dist_to_center and abs(v.end_angle - other.end_angle) < 0.1:
                            exit_lead = other
                            break

                # 基础退出速度
                base_exit_v = 20 if v.type == 'car' else 12

                if exit_lead:
                    gap = exit_lead.dist_to_center - v.dist_to_center
                    if gap < 45:
                        v.v = 0  # 别撞上前面的退出车
                    else:
                        v.v = min(base_exit_v, (gap - 35) / 60 * base_exit_v)
                else:
                    v.v = min(v.v + 1.0, base_exit_v)

                v.dist_to_center += v.v * DT
                if v.dist_to_center > 900:
                    self.data_logs.append({
                        'id': v.id, 'type': v.type, 'wait_time': round(v.wait_time, 2),
                        'conflicts': v.conflict_count, 'status': 'exited'
                    })
                    self.vehicles.remove(v)

            # --- 重点：就在这里插入 [暴力解死锁] ---
            # 此时 v 的速度 v.v 已经被上面的逻辑算好了，我们现在检查是否重叠
            for other in self.vehicles:
                if v.id == other.id: continue
                if v.visual_x is None or other.visual_x is None:
                    continue
                # 计算物理距离或角度差
                # 如果两车距离过近（比如小于 15 像素）
                dx = v.visual_x - other.visual_x
                dy = v.visual_y - other.visual_y
                if (dx ** 2 + dy ** 2) < 225:  # 15像素的平方
                    if v.state == "CIRCULATING":
                        # 强制给环岛内的车一个推力，让它离开重叠区
                        v.v = max(v.v, 5.0)
                        # 同时也让它移动，防止位移跳变
                        angular_speed = v.v / v.dist_to_center
                        v.current_angle -= angular_speed * DT

            if v.wait_time > 20 and v.v < 0.1:
                v.v = 2.0
                v.wait_time = 0  # 重置计数，给它推一把
            # 冲突检测
            if lead_v and v.v > 0.5:
                current_gap = actual_gap if v.state == "APPROACHING" else abs(v.current_angle - lead_v.current_angle) * R
                if current_gap < 20:
                    v.conflict_count += 1
                    self.total_conflicts += 1

            if v.v < 0.2:
                v.wait_time += DT
                # 如果等了超过 5 秒还没动
                if v.wait_time > 5.0:
                    # 如果是环岛内的车，强制它走，带起车流
                    if v.state == "CIRCULATING":
                        v.v = 3.0
                        # 如果是直行车，且前面没那么挤，强制它试探性进场
                    elif v.state == "APPROACHING" and (not lead_v or (v.dist_to_center - lead_v.dist_to_center) > 50):
                        v.v = 2.0
            else:
                v.wait_time = 0  # 动起来了就重置


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
        GRASS_GREEN = (144, 238, 144)
        LINE_WHITE = (255, 255, 255)
        CENTER_YELLOW = (200, 200, 0)

        total_road_width = 100
        outer_radius = R + (total_road_width // 2)  # 环岛路面外边缘 (R+50)
        inner_radius = R - (total_road_width // 2)  # 绿化带边缘 (R-50)

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

    def run(self):
        clock = pygame.time.Clock()
        running = True
        while running:
            # 强制每秒只更新 60 次
            clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    # 关键：点击关闭窗口时，先保存数据再退出
                    print("正在自动保存数据...")
                    self.export_data()
                    running = False

                # 手动触发：按下 S 键
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_s:
                        self.export_data()

            # ... 原有的 spawn, update, draw 逻辑 ...
            self.spawn_timer += 1
            if self.spawn_timer > 15:
                self.spawn_vehicle()
                self.spawn_timer = 0

            self.update()
            self.draw()
            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()


if __name__ == "__main__":
    sim = AdvancedSim()
    sim.run()