#!/usr/bin/env python3
"""
Pacer IMU 可视化 — 3D 四旋翼姿态实时显示

用法:
  python3 imu_viz.py --csv imu_log.csv          # CSV 回放
  python3 imu_viz.py --udp 0.0.0.0:8888          # UDP 实时流
  python3 imu_viz.py --udp 0.0.0.0:8888 --csv imu_log.csv  # 同时录 CSV

UDP 协议 (树莓派发送):
  每 20ms 一帧, JSON 格式:
  {"t":12.345,"ax":0.1,"ay":0.0,"az":9.8,"gx":0.5,"gy":-0.3,"gz":0.1,
   "roll":1.2,"pitch":-0.5,"yaw":180.0,"temp":35.2}

  或二进制 (更高效): 结构体 pack "<f11" (11 个 float32)

依赖:
  pip install matplotlib numpy
"""

import sys
import os
import math
import socket
import struct
import json
import time
import argparse
from collections import deque
from threading import Thread, Event

import numpy as np

# ============================================================
# 姿态滤波器 (Madgwick) — 和 C 代码保持一致
# ============================================================

class Madgwick:
    def __init__(self, beta=0.04):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q

        # rad/s
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        # 归一化加速度
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 0.001:
            return self.get_euler()
        ax /= norm; ay /= norm; az /= norm

        # 重力方向估计
        hx = 2*(q1*q3 - q0*q2)
        hy = 2*(q0*q1 + q2*q3)
        hz = q0*q0 - q1*q1 - q2*q2 + q3*q3

        # 误差
        ex = ay*hz - az*hy
        ey = az*hx - ax*hz
        ez = ax*hy - ay*hx

        e_norm = math.sqrt(ex*ex + ey*ey + ez*ez)
        if e_norm > 0.001:
            ex /= e_norm; ey /= e_norm; ez /= e_norm

        gx += self.beta * ex
        gy += self.beta * ey
        gz += self.beta * ez

        # 四元数积分
        hdt = 0.5 * dt
        q0 += (-q1*gx - q2*gy - q3*gz) * hdt
        q1 += ( q0*gx + q2*gz - q3*gy) * hdt
        q2 += ( q0*gy - q1*gz + q3*gx) * hdt
        q3 += ( q0*gz + q1*gy - q2*gx) * hdt

        self.q = np.array([q0, q1, q2, q3])
        self.q /= np.linalg.norm(self.q)
        return self.get_euler()

    def get_euler(self):
        q0, q1, q2, q3 = self.q
        roll = math.degrees(math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)))
        sinp = 2*(q0*q2 - q3*q1)
        sinp = max(-1, min(1, sinp))
        pitch = math.degrees(math.asin(sinp))
        yaw = math.degrees(math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)))
        return roll, pitch, yaw


# ============================================================
# 数据源
# ============================================================

class IMUSample:
    __slots__ = ['t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                 'mx', 'my', 'mz', 'temp', 'roll', 'pitch', 'yaw']
    def __init__(self):
        self.t = 0.0
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.mx = self.my = self.mz = 0.0
        self.temp = 0.0
        self.roll = self.pitch = self.yaw = 0.0


class CSVSource:
    """CSV 回放数据源"""
    def __init__(self, path, sample_hz=400):
        self.path = path
        self.sample_hz = sample_hz
        self.filter = Madgwick(beta=0.04)
        self._load()

    def _load(self):
        self.data = []
        with open(self.path, 'r') as f:
            f.readline()  # skip header
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 7:
                    continue
                s = IMUSample()
                s.t = float(parts[0]) / 1e6  # us → s
                s.ax = float(parts[1]); s.ay = float(parts[2]); s.az = float(parts[3])
                s.gx = float(parts[4]); s.gy = float(parts[5]); s.gz = float(parts[6])
                if len(parts) >= 11:
                    s.mx = float(parts[7]); s.my = float(parts[8]); s.mz = float(parts[9])
                    s.temp = float(parts[10])
                elif len(parts) >= 8:
                    s.temp = float(parts[7])
                self.data.append(s)

        # 如果 CSV 没有姿态角, 用 Madgwick 计算
        dt = 1.0 / self.sample_hz
        for s in self.data:
            s.roll, s.pitch, s.yaw = self.filter.update(
                s.gx, s.gy, s.gz, s.ax, s.ay, s.az, dt)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, i):
        return self.data[i]


class UDPSource:
    """UDP 实时数据源"""
    def __init__(self, listen_addr, csv_out=None):
        self.listen_addr = listen_addr
        self.csv_out = csv_out
        self.filter = Madgwick(beta=0.04)
        self.samples = deque(maxlen=2000)
        self.stop_flag = Event()
        self._fp = None
        if csv_out:
            self._fp = open(csv_out, 'w')
            self._fp.write("time_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp\n")

        self._thread = Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        host, port = self.listen_addr.rsplit(':', 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, int(port)))
        sock.settimeout(0.5)

        t0 = time.time()
        last_t = t0

        while not self.stop_flag.is_set():
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                continue

            now = time.time()
            dt = now - last_t
            last_t = now

            s = IMUSample()
            s.t = now - t0

            # 尝试 JSON
            try:
                msg = json.loads(data)
                s.ax = msg.get('ax', 0); s.ay = msg.get('ay', 0); s.az = msg.get('az', 9.8)
                s.gx = msg.get('gx', 0); s.gy = msg.get('gy', 0); s.gz = msg.get('gz', 0)
                s.temp = msg.get('temp', 0)

                if 'roll' in msg:
                    s.roll = msg['roll']; s.pitch = msg['pitch']; s.yaw = msg['yaw']
                else:
                    s.roll, s.pitch, s.yaw = self.filter.update(
                        s.gx, s.gy, s.gz, s.ax, s.ay, s.az, dt)

            except (json.JSONDecodeError, UnicodeDecodeError):
                # 尝试二进制: 11 个 float32
                if len(data) == 44:
                    vals = struct.unpack('<11f', data)
                    s.t = vals[0]
                    s.ax, s.ay, s.az = vals[1], vals[2], vals[3]
                    s.gx, s.gy, s.gz = vals[4], vals[5], vals[6]
                    s.mx, s.my, s.mz = vals[7], vals[8], vals[9]
                    s.temp = vals[10]
                    s.roll, s.pitch, s.yaw = self.filter.update(
                        s.gx, s.gy, s.gz, s.ax, s.ay, s.az, dt)
                else:
                    continue

            if self._fp:
                self._fp.write(f"{int(s.t*1e6)},{s.ax:.6f},{s.ay:.6f},{s.az:.6f},"
                               f"{s.gx:.6f},{s.gy:.6f},{s.gz:.6f},"
                               f"{s.mx:.2f},{s.my:.2f},{s.mz:.2f},{s.temp:.2f}\n")
                self._fp.flush()

            self.samples.append(s)

    def latest(self):
        return self.samples[-1] if self.samples else None

    def close(self):
        self.stop_flag.set()
        if self._fp:
            self._fp.close()


# ============================================================
# 3D 四旋翼模型
# ============================================================

def build_quad_model():
    """
    构建四旋翼 3D 模型顶点 (NED → 标准右手系)

    电机布局 ("X" 型):
        前
       FL  FR      FL=前左(CW)   FR=前右(CCW)
        \/
        /\
       RL  RR      RL=后左(CCW)  RR=后右(CW)
        后

    返回各部件的顶点数组
    """
    arm_len = 0.6  # 轴距比例

    # 电机位置 (机体坐标系, X 前 Y 右 Z 下)
    motors = np.array([
        [-arm_len * 0.707, -arm_len * 0.707, 0],  # FL
        [-arm_len * 0.707,  arm_len * 0.707, 0],  # FR
        [ arm_len * 0.707, -arm_len * 0.707, 0],  # RL
        [ arm_len * 0.707,  arm_len * 0.707, 0],  # RR
    ])

    # 机臂 (中心板到各电机)
    arms = []
    for m in motors:
        arms.append(np.array([[0,0,0], m]))

    # 中心板 (方形)
    plate_size = 0.15
    plate = np.array([
        [-plate_size, -plate_size, 0],
        [ plate_size, -plate_size, 0],
        [ plate_size,  plate_size, 0],
        [-plate_size,  plate_size, 0],
        [-plate_size, -plate_size, 0],
    ])

    # 桨叶 (每个电机一对线段, 表示旋转的桨)
    prop_radius = 0.25
    props = []
    for m in motors:
        # 桨叶用圆盘近似 (多个线段)
        n_seg = 8
        circle = []
        for i in range(n_seg + 1):
            ang = 2 * math.pi * i / n_seg
            circle.append(m + np.array([
                prop_radius * math.cos(ang),
                prop_radius * math.sin(ang),
                0.02
            ]))
        props.append(np.array(circle))

    # 方向指示 (机头朝前 = -X 方向)
    nose = np.array([
        [0, 0, 0],
        [-arm_len * 1.2, 0, 0],
    ])

    # 上方箭头 (表示 "上" 方向, 始终指向 -Z 的反方向)
    up_arrow = np.array([
        [0, 0, 0],
        [0, 0, -0.3],
    ])

    return motors, arms, plate, props, nose, up_arrow


def euler_to_rotation(roll, pitch, yaw):
    """
    欧拉角 (度) → 3x3 旋转矩阵

    旋转顺序: Z(yaw) → Y(pitch) → X(roll)
    机体坐标系 → 世界坐标系
    """
    cr = math.cos(math.radians(roll))
    sr = math.sin(math.radians(roll))
    cp = math.cos(math.radians(pitch))
    sp = math.sin(math.radians(pitch))
    cy = math.cos(math.radians(yaw))
    sy = math.sin(math.radians(yaw))

    # ZYX 旋转矩阵
    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr   ],
    ])

    return R


def rotate_points(points, R):
    """应用旋转矩阵到点集"""
    return points @ R.T


# ============================================================
# 可视化主程序
# ============================================================

def run_viz(source, mode='csv', playback_speed=1.0):
    """
    主可视化循环

    source: CSVSource 或 UDPSource
    mode: 'csv' or 'udp'
    """
    import matplotlib
    matplotlib.use('TkAgg')  # 或 Qt5Agg

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

    # 构建模型
    motors, arms, plate, props, nose, up_arrow = build_quad_model()

    # 颜色
    ARM_COLORS = ['#FF4444', '#4444FF', '#44FF44', '#FFFF44']  # FL红 FR蓝 RL绿 RR黄
    PROP_COLORS = ['#FF6666', '#6666FF', '#66FF66', '#FFFF66']
    BG_COLOR = '#1a1a2e'
    GRID_COLOR = '#16213e'
    TEXT_COLOR = '#e0e0e0'

    # 创建图
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(12, 8), facecolor=BG_COLOR)

    # 左边: 3D 可视化
    ax3d = fig.add_subplot(121, projection='3d', facecolor=BG_COLOR)
    ax3d.set_facecolor(BG_COLOR)

    # 右边: 数据曲线
    ax_roll  = fig.add_subplot(4, 2, 2, facecolor=BG_COLOR)
    ax_pitch = fig.add_subplot(4, 2, 4, facecolor=BG_COLOR)
    ax_yaw   = fig.add_subplot(4, 2, 6, facecolor=BG_COLOR)
    ax_accel = fig.add_subplot(4, 2, 8, facecolor=BG_COLOR)

    fig.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.05, wspace=0.3, hspace=0.5)

    fig.suptitle('Pacer IMU 可视化', fontsize=14, color=TEXT_COLOR, fontweight='bold')

    # 3D 轴设置
    lim = 1.0
    ax3d.set_xlim(-lim, lim)
    ax3d.set_ylim(-lim, lim)
    ax3d.set_zlim(-lim, lim)
    ax3d.set_xlabel('X (前)', color=TEXT_COLOR, fontsize=8)
    ax3d.set_ylabel('Y (右)', color=TEXT_COLOR, fontsize=8)
    ax3d.set_zlabel('Z (下)', color=TEXT_COLOR, fontsize=8)
    ax3d.tick_params(colors=TEXT_COLOR, labelsize=6)
    ax3d.xaxis.pane.fill = False
    ax3d.yaxis.pane.fill = False
    ax3d.zaxis.pane.fill = False
    ax3d.xaxis.pane.set_edgecolor(GRID_COLOR)
    ax3d.yaxis.pane.set_edgecolor(GRID_COLOR)
    ax3d.zaxis.pane.set_edgecolor(GRID_COLOR)
    ax3d.grid(True, alpha=0.2)

    # 初始化绘图对象
    arm_lines = []
    for i, (arm, color) in enumerate(zip(arms, ARM_COLORS)):
        line, = ax3d.plot([], [], [], color=color, linewidth=2.5)
        arm_lines.append(line)

    plate_line, = ax3d.plot([], [], [], color='#888888', linewidth=1.5)

    prop_lines = []
    for i, color in enumerate(PROP_COLORS):
        line, = ax3d.plot([], [], [], color=color, linewidth=1.0, alpha=0.6)
        prop_lines.append(line)

    nose_line, = ax3d.plot([], [], [], color='#FF00FF', linewidth=3)
    up_line, = ax3d.plot([], [], [], color='#00FFFF', linewidth=2, linestyle='--')

    # 电机圆点
    motor_dots, = ax3d.plot([], [], [], 'o', color='white', markersize=6)

    # 信息文本
    info_text = ax3d.text2D(0.02, 0.95, "", transform=ax3d.transAxes,
                            color=TEXT_COLOR, fontsize=9, family='monospace',
                            verticalalignment='top')

    # 曲线历史
    history_len = 500
    hist_t = deque(maxlen=history_len)
    hist_roll = deque(maxlen=history_len)
    hist_pitch = deque(maxlen=history_len)
    hist_yaw = deque(maxlen=history_len)
    hist_ax = deque(maxlen=history_len)
    hist_ay = deque(maxlen=history_len)
    hist_az = deque(maxlen=history_len)

    # 曲线初始化
    for ax, label, color in [(ax_roll, 'Roll (°)', '#FF6B6B'),
                               (ax_pitch, 'Pitch (°)', '#4ECDC4'),
                               (ax_yaw, 'Yaw (°)', '#FFE66D'),
                               (ax_accel, 'Accel (m/s²)', '#A8E6CF')]:
        ax.set_ylabel(label, color=color, fontsize=7)
        ax.tick_params(colors=TEXT_COLOR, labelsize=6)
        ax.grid(True, alpha=0.15)

    roll_line,  = ax_roll.plot([], [], color='#FF6B6B', linewidth=1)
    pitch_line, = ax_pitch.plot([], [], color='#4ECDC4', linewidth=1)
    yaw_line,   = ax_yaw.plot([], [], color='#FFE66D', linewidth=1)
    ax_line,    = ax_accel.plot([], [], color='#FF6B6B', linewidth=1, label='X')
    ay_line,    = ax_accel.plot([], [], color='#4ECDC4', linewidth=1, label='Y')
    az_line,    = ax_accel.plot([], [], color='#FFE66D', linewidth=1, label='Z')
    ax_accel.legend(fontsize=6, loc='upper right', framealpha=0.3)

    # ---------- 帧索引 ----------
    frame_idx = [0]
    start_time = [time.time()]
    prop_phase = [0.0]

    def get_sample():
        if mode == 'csv':
            if frame_idx[0] >= len(source):
                frame_idx[0] = 0  # 循环回放
                # 重置历史
                hist_t.clear(); hist_roll.clear(); hist_pitch.clear()
                hist_yaw.clear(); hist_ax.clear(); hist_ay.clear(); hist_az.clear()
            s = source[frame_idx[0]]
            # 播放速度控制
            expected_t = (time.time() - start_time[0]) * playback_speed
            while frame_idx[0] < len(source) - 1 and source[frame_idx[0]].t < expected_t:
                frame_idx[0] += 1
            return s
        else:
            return source.latest()

    def update(frame):
        s = get_sample()
        if s is None:
            return []

        roll, pitch, yaw = s.roll, s.pitch, s.yaw

        # 螺旋桨旋转 (视觉效果)
        prop_phase[0] += 0.8  # 每帧旋转量

        # 计算旋转矩阵
        R = euler_to_rotation(roll, pitch, yaw)

        # 旋转各部件
        rotated_motors = rotate_points(motors, R)

        # 更新机臂
        for i, line in enumerate(arm_lines):
            pts = np.array([[0,0,0], rotated_motors[i]])
            line.set_data_3d(pts[:,0], pts[:,1], pts[:,2])

        # 更新中心板
        rp = rotate_points(plate, R)
        plate_line.set_data_3d(rp[:,0], rp[:,1], rp[:,2])

        # 更新桨叶 (旋转 + 倾斜)
        for i, (prop, line) in enumerate(zip(props, prop_lines)):
            # 桨叶自旋
            spin = prop_phase[0] * (1 if i in [0, 3] else -1)  # CW/CCW
            cs, sn = math.cos(spin), math.sin(spin)
            spun = prop.copy()
            spun[:,0] = prop[:,0] * cs - prop[:,1] * sn
            spun[:,1] = prop[:,0] * sn + prop[:,1] * cs
            # 平移到电机位置
            spun = spun - motors[i] + rotated_motors[i]
            # 应用姿态旋转
            spun = rotate_points(spun, R)
            line.set_data_3d(spun[:,0], spun[:,1], spun[:,2])

        # 机头方向
        rn = rotate_points(nose, R)
        nose_line.set_data_3d(rn[:,0], rn[:,1], rn[:,2])

        # 上方向
        ru = rotate_points(up_arrow, R)
        up_line.set_data_3d(ru[:,0], ru[:,1], ru[:,2])

        # 电机点
        motor_dots.set_data_3d(rotated_motors[:,0], rotated_motors[:,1], rotated_motors[:,2])

        # 更新信息
        accel_mag = math.sqrt(s.ax**2 + s.ay**2 + s.az**2)
        gyro_mag = math.sqrt(s.gx**2 + s.gy**2 + s.gz**2)
        info = (f"姿态:  R={roll:+7.2f}°  P={pitch:+7.2f}°  Y={yaw:+7.2f}°\n"
                f"陀螺:  X={s.gx:+7.2f}  Y={s.gy:+7.2f}  Z={s.gz:+7.2f} °/s\n"
                f"加速度: X={s.ax:+6.3f}  Y={s.ay:+6.3f}  Z={s.az:+6.3f} m/s²\n"
                f"合成: |a|={accel_mag:.2f} m/s²  |ω|={gyro_mag:.1f} °/s\n"
                f"温度: {s.temp:.1f}°C")
        if mode == 'csv':
            info += f"\n帧: {frame_idx[0]}/{len(source)}  t={s.t:.2f}s"
        else:
            info += f"\n实时  t={s.t:.2f}s  缓冲: {len(source.samples)}"
        info_text.set_text(info)

        # 更新曲线
        hist_t.append(s.t)
        hist_roll.append(roll)
        hist_pitch.append(pitch)
        hist_yaw.append(yaw)
        hist_ax.append(s.ax)
        hist_ay.append(s.ay)
        hist_az.append(s.az)

        t_list = list(hist_t)
        t_arr = np.array(t_list) - t_list[0] if t_list else [0]

        roll_line.set_data(t_arr, list(hist_roll))
        pitch_line.set_data(t_arr, list(hist_pitch))
        yaw_line.set_data(t_arr, list(hist_yaw))
        ax_line.set_data(t_arr, list(hist_ax))
        ay_line.set_data(t_arr, list(hist_ay))
        az_line.set_data(t_arr, list(hist_az))

        # 自适应 Y 范围
        for ax, hist in [(ax_roll, hist_roll), (ax_pitch, hist_pitch),
                         (ax_yaw, hist_yaw)]:
            if len(hist) > 2:
                lo, hi = min(hist), max(hist)
                margin = max(1, (hi - lo) * 0.1)
                ax.set_ylim(lo - margin, hi + margin)
            ax.set_xlim(t_arr[0], t_arr[-1] if len(t_arr) > 1 else 1)

        if len(hist_ax) > 2:
            all_a = list(hist_ax) + list(hist_ay) + list(hist_az)
            lo, hi = min(all_a), max(all_a)
            margin = max(0.5, (hi - lo) * 0.1)
            ax_accel.set_ylim(lo - margin, hi + margin)
        ax_accel.set_xlim(t_arr[0], t_arr[-1] if len(t_arr) > 1 else 1)

        # 3D 视角跟随
        ax3d.view_init(elev=25 - pitch * 0.3, azim=-yaw)

        return (arm_lines + [plate_line] + prop_lines + [nose_line, up_line, motor_dots,
                roll_line, pitch_line, yaw_line, ax_line, ay_line, az_line])

    ani = FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)

    plt.show()

    if mode == 'udp':
        source.close()


# ============================================================
# 树莓派端发送器 (配套)
# ============================================================

SENDER_CODE = r'''
#!/usr/bin/env python3
"""
树莓派端 IMU 数据发送器 — 配合 imu_viz.py 使用

读取 ICM20948 数据 (通过 pacer 的 --imu-stream 输出解析,
或直接用 smbus2 读寄存器), 通过 UDP 发送给可视化端。

用法 (树莓派上运行):
  python3 imu_sender.py --host <电脑IP>:8888

  或从 pacer --imu-stream 的 stdout 解析:
  sudo ./pacer --imu-stream | python3 imu_sender.py --host <电脑IP>:8888
"""

import sys, socket, json, time, re, argparse, struct

def parse_stream(host_port):
    """解析 pacer --imu-stream 的 stdout, 通过 UDP 发送"""
    host, port = host_port.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, int(port))

    print(f"发送 IMU 数据到 {host}:{port}")
    print("等待 pacer --imu-stream 输入...")

    t0 = time.time()
    for line in sys.stdin:
        # 解析格式: time ax ay az  gx gy gz  Roll Pitch Yaw  Temp
        # 示例: 12.345   0.123  -0.001   9.802   0.50  -0.30   0.10   1.20  -0.50  180.00   35.2
        parts = line.strip().split()
        nums = []
        for p in parts:
            try:
                nums.append(float(p))
            except ValueError:
                pass
        if len(nums) >= 10:
            msg = {
                't': nums[0],
                'ax': nums[1], 'ay': nums[2], 'az': nums[3],
                'gx': nums[4], 'gy': nums[5], 'gz': nums[6],
                'roll': nums[7], 'pitch': nums[8], 'yaw': nums[9],
                'temp': nums[10] if len(nums) > 10 else 0,
            }
            sock.sendto(json.dumps(msg).encode(), addr)

def send_binary(host_port):
    """二进制发送 (11 个 float32 = 44 字节)"""
    host, port = host_port.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, int(port))

    print(f"二进制模式 → {host}:{port}")
    t0 = time.time()
    for line in sys.stdin:
        parts = line.strip().split()
        nums = []
        for p in parts:
            try:
                nums.append(float(p))
            except ValueError:
                pass
        if len(nums) >= 11:
            # t, ax, ay, az, gx, gy, gz, mx, my, mz, temp
            pkt = struct.pack('<11f', *nums[:11])
            sock.sendto(pkt, addr)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--host', required=True, help='目标地址 IP:PORT')
    p.add_argument('--binary', action='store_true', help='二进制模式')
    args = p.parse_args()

    if args.binary:
        send_binary(args.host)
    else:
        parse_stream(args.host)
'''


# ============================================================
# 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Pacer IMU 3D 可视化',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 imu_viz.py --csv imu_log.csv              # CSV 回放
  python3 imu_viz.py --csv imu_log.csv --speed 0.5  # 慢放
  python3 imu_viz.py --udp 0.0.0.0:8888             # UDP 实时
  python3 imu_viz.py --udp 0.0.0.0:8888 --record live.csv  # 边看边录

树莓派端:
  sudo ./pacer --imu-stream | python3 imu_sender.py --host <电脑IP>:8888
        """)
    parser.add_argument('--csv', help='CSV 文件回放')
    parser.add_argument('--udp', help='UDP 监听地址 IP:PORT')
    parser.add_argument('--record', help='UDP 模式下同时录 CSV')
    parser.add_argument('--speed', type=float, default=1.0, help='CSV 回放速度 (0.1~10)')
    parser.add_argument('--hz', type=int, default=400, help='CSV 采样率 (默认 400)')
    parser.add_argument('--sender', action='store_true', help='生成树莓派端发送脚本 imu_sender.py')

    args = parser.parse_args()

    if args.sender:
        with open('imu_sender.py', 'w') as f:
            f.write(SENDER_CODE)
        os.chmod('imu_sender.py', 0o755)
        print("已生成 imu_sender.py — 上传到树莓派使用")
        return

    if not args.csv and not args.udp:
        parser.print_help()
        print("\n错误: 需要 --csv 或 --udp")
        sys.exit(1)

    print("Pacer IMU 3D 可视化")
    print("=" * 50)

    try:
        if args.csv:
            print(f"加载 CSV: {args.csv}")
            source = CSVSource(args.csv, sample_hz=args.hz)
            print(f"共 {len(source)} 帧 ({len(source)/args.hz:.1f} 秒)")
            print(f"回放速度: {args.speed}x")
            run_viz(source, mode='csv', playback_speed=args.speed)

        elif args.udp:
            print(f"UDP 监听: {args.udp}")
            if args.record:
                print(f"同时录制 → {args.record}")
            source = UDPSource(args.udp, csv_out=args.record)
            print("等待数据...")
            print("树莓派发送: sudo ./pacer --imu-stream | python3 imu_sender.py --host <本机IP>:<端口>")
            run_viz(source, mode='udp')

    except KeyboardInterrupt:
        print("\n退出")
    except FileNotFoundError as e:
        print(f"文件不存在: {e}")
    except ImportError as e:
        print(f"缺少依赖: {e}")
        print("安装: pip install matplotlib numpy")


if __name__ == '__main__':
    main()
