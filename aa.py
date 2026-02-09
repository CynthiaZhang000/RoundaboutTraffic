def update(self):
    # 获取环岛周长，用于圆周距离计算
    circle_length = 2 * np.pi * R

    for v in self.vehicles[:]:
        # --- 1. 寻找前车 (Lead Vehicle) ---
        lead = None
        # 仅在相同路径状态下寻找前车，防止误判
        same_path_vehicles = [other for other in self.vehicles if other != v and other.state == v.state]

        if v.state == "APPROACHING":
            # 在同一条引道且更接近中心的车
            potential_leads = [other for other in same_path_vehicles
                               if other.start_angle == v.start_angle and other.dist_to_center < v.dist_to_center]
            if potential_leads:
                lead = min(potential_leads, key=lambda x: x.dist_to_center)

        elif v.state == "CIRCULATING":
            # 环岛内寻找圆周前方最接近的车
            if same_path_vehicles:
                lead = min(same_path_vehicles, key=lambda x: (x.pos - v.pos) % circle_length)

        # --- 2. 核心：并入间隙检查 (Gap Acceptance) ---
        # 只有当车辆处于引道末端，准备进入环岛时才执行
        is_blocked_by_ring = False
        if v.state == "APPROACHING" and v.dist_to_center <= R + 15:
            merge_pos = v.start_angle * R
            for other in self.vehicles:
                # 如果环岛内有车，且距离并入点太近
                if other.state == "CIRCULATING":
                    # 计算环岛内车辆距离当前入口的圆周距离
                    dist_in_ring = (other.pos - merge_pos + circle_length / 2) % circle_length - circle_length / 2
                    # 如果距离在 40 像素以内，就必须等待（增加这个距离防止碰撞）
                    if -20 < dist_in_ring < 50:
                        is_blocked_by_ring = True
                        break

        # --- 3. 计算并更新加速度 ---
        if is_blocked_by_ring:
            # 如果被环岛内的流拦截，强制刹车减速
            accel = -v.a if v.v > 0 else 0
            # 这里为了简化，也可以直接用 v.update_acceleration(None) 但设定一个虚拟的前方静止目标
        else:
            accel = v.update_acceleration(lead)

        # 更新速度
        v.v = max(0, v.v + accel * DT)

        # --- 4. 统计等待时间 ---
        # 只要在引道且速度低于 1.5m/s (认为是在排队或等位)，就开始计时
        if v.state == "APPROACHING" and v.v < v.v0 * 0.7:
            v.wait_time += DT

        # 安全冲突检测 (Safety Metric)
        if lead:
            dist = abs(v.dist_to_center - lead.dist_to_center) if v.state != "CIRCULATING" else (
                                                                                                        lead.pos - v.pos) % circle_length
            if dist < 5 and v.v > 0.5:  # 距离小于5且在移动视为冲突
                v.conflict_count += 1
                self.total_conflicts += 1

        # --- 5. 位置移动与状态切换 ---
        if v.state == "APPROACHING":
            # 停止线设在距离中心 R + 50 的位置（即环岛路面的外边缘）
            stop_line_dist = R + 50

            next_dist = v.dist_to_center - v.v * DT

            # 如果前方有车挡住，就停在环岛路口外
            if next_dist <= stop_line_dist and is_blocked_by_ring:
                v.dist_to_center = stop_line_dist
                v.v = 0
            # 只有可以通行时，才继续滑行进入环岛并切换状态
            elif next_dist <= R and not is_blocked_by_ring:
                v.state = "CIRCULATING"
                v.dist_to_center = R
                v.pos = v.start_angle * R
                # v.current_angle = v.start_angle
            else:
                v.dist_to_center = next_dist

        elif v.state == "CIRCULATING":
            v.pos += v.v * DT
            v.current_angle = v.pos / R

            # 检查是否到达出口
            angle_diff = (v.current_angle - v.end_angle) % (2 * np.pi)
            if angle_diff < 0.1 or angle_diff > (2 * np.pi - 0.1):
                v.state = "EXITING"
                v.dist_to_center = R

        elif v.state == "EXITING":
            v.dist_to_center += v.v * DT
            # 移除驶出屏幕的车辆
            if v.dist_to_center > ROAD_LEN + R + 100:
                self.data_logs.append({
                    'id': v.id,
                    'type': v.type,
                    'wait_time': round(v.wait_time, 2),
                    'conflicts': v.conflict_count,
                    'status': 'exited'
                })
                self.vehicles.remove(v)
