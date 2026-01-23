import serial
import struct
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

# 配置
# DEFAULT_PORT = 'COM27'
DEFAULT_PORT = 'COM27' # User might need to change this, but safe default
BAUDRATE = 115200
MOTOR_NUM = 7

class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboMaster Dart Motor Control")
        # 增加宽度以容纳左侧边栏
        self.root.geometry("1100x700")
        
        self.ser = None
        self.connected = False
        self.lock = threading.Lock()
        
        # 变量
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.motor_id_var = tk.IntVar(value=0)
        self.mode_var = tk.StringVar(value="Disable")
        self.value_var = tk.IntVar(value=0)
        self.time_var = tk.IntVar(value=1000) # Added: Time for SpeedTime mode
        self.friction_speed_var = tk.IntVar(value=4000) # Added: Friction wheel speed 1 (default 4000)
        self.friction_speed_2_var = tk.IntVar(value=4000) # Added: Friction wheel speed 2 (default 4000)
        self.yaw_angle_var = tk.DoubleVar(value=0) # Slide range -245000 to 245000
        self.log_text = None
        
        # 电机状态数据存储
        # motor_data[id] = { 'angle': int, 'rpm': int, 'torque': int, 'temp': int, 'enabled': bool, 'stalled': bool, 'mode': int, 'total_angle': int }
        self.motor_data = [
            {'angle': 0, 'rpm': 0, 'torque': 0, 'temp': 0, 'enabled': False, 'stalled': False, 'mode': 0, 'total_angle': 0}
            for _ in range(MOTOR_NUM)
        ]
        self.motor_labels = [] # 存储UI标签引用以便更新
        
        # 模式映射
        self.modes = {
            "Disable (0x00)": 0,
            "Current (0x01)": 1,
            "Angle (0x02)": 2,
            "Speed (0x03)": 3,
            "Torque (0x04)": 4,
            "RunToStall (0x05)": 5,
            "RunToAngle (0x06)": 6,
            "SpeedTime (0x07)": 7
        }
        
        self.create_ui()
        
        # 启动UI定时刷新任务
        self.root.after(100, self.update_monitor_ui)
        
    def create_ui(self):
        # 主布局：左右分栏
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)
        
        # 左侧：电机状态监控栏
        frame_monitor = ttk.LabelFrame(paned, text="Motor Status Monitor", width=420)
        paned.add(frame_monitor, weight=1)
        
        # 创建左侧监控表格
        self.create_monitor_panel(frame_monitor)
        
        # 右侧：原有控制面板
        frame_control = ttk.Frame(paned)
        paned.add(frame_control, weight=2)
        
        self.create_control_panel(frame_control)

    def create_monitor_panel(self, parent):
        # 使用 Canvas + Scrollbar 以防展示不全
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 标题行
        headers = ["ID", "Stat", "Mode", "Angle", "RPM", "Torq", "Tmp", "TotAng"]
        for col, text in enumerate(headers):
            lbl = ttk.Label(scrollable_frame, text=text, font=('Arial', 9, 'bold'))
            lbl.grid(row=0, column=col, padx=4, pady=5, sticky="w")
            
        # 电机数据行
        motor_names = ["Fric1", "Fric2", "Fric3", "Fric4", "GM6020", "Load", "Lift"]
        
        for i in range(MOTOR_NUM):
            # ID / Name
            ttk.Label(scrollable_frame, text=f"{i} ({motor_names[i]})", font=('Arial', 9)).grid(row=i+1, column=0, padx=2, pady=2, sticky="w")
            
            # Status (Enabled/Stalled)
            lbl_stat = ttk.Label(scrollable_frame, text="DIS", font=('Arial', 9), foreground="gray", width=5)
            lbl_stat.grid(row=i+1, column=1, padx=2, pady=2)
            
            # Mode
            lbl_mode = ttk.Label(scrollable_frame, text="0", font=('Arial', 9), width=3)
            lbl_mode.grid(row=i+1, column=2, padx=2, pady=2)
            
            # Angle
            lbl_angle = ttk.Label(scrollable_frame, text="0", font=('Arial', 9), width=6)
            lbl_angle.grid(row=i+1, column=3, padx=2, pady=2)
            
            # RPM
            lbl_rpm = ttk.Label(scrollable_frame, text="0", font=('Arial', 9), width=5)
            lbl_rpm.grid(row=i+1, column=4, padx=2, pady=2)
            
            # Torque
            lbl_trq = ttk.Label(scrollable_frame, text="0", font=('Arial', 9), width=5)
            lbl_trq.grid(row=i+1, column=5, padx=2, pady=2)
            
            # Temp
            lbl_tmp = ttk.Label(scrollable_frame, text="0°C", font=('Arial', 9), width=4)
            lbl_tmp.grid(row=i+1, column=6, padx=2, pady=2)

            # Total Angle
            lbl_total_angle = ttk.Label(scrollable_frame, text="0", font=('Arial', 9), width=8)
            lbl_total_angle.grid(row=i+1, column=7, padx=2, pady=2)
            
            self.motor_labels.append({
                'stat': lbl_stat, 'mode': lbl_mode, 'angle': lbl_angle, 
                'rpm': lbl_rpm, 'trq': lbl_trq, 'tmp': lbl_tmp, 'total_angle': lbl_total_angle
            })

    def create_individual_controls(self, parent):
        frame_indiv = ttk.LabelFrame(parent, text="Individual Motor Control (Speed Mode 0x03)")
        frame_indiv.pack(fill="x", padx=10, pady=5)
        
        # Grid headers
        ttk.Label(frame_indiv, text="ID").grid(row=0, column=0, padx=2)
        ttk.Label(frame_indiv, text="Set Speed").grid(row=0, column=1, padx=2)
        ttk.Label(frame_indiv, text="Control").grid(row=0, column=2, padx=2, columnspan=3)
        
        self.indiv_speed_vars = []
        # Defaults: 0-3=4000, 4=300, 6=3000. (5 default to 3000 like 6 or safe value)
        defaults = {0:4000, 1:4000, 2:4000, 3:4000, 4:300, 5:1000, 6:3000}
        
        for i in range(MOTOR_NUM):
            # ID
            ttk.Label(frame_indiv, text=f"M{i}").grid(row=i+1, column=0, padx=2, pady=2)
            
            # Speed Input
            var = tk.IntVar(value=defaults.get(i, 1000))
            self.indiv_speed_vars.append(var)
            ttk.Entry(frame_indiv, textvariable=var, width=8).grid(row=i+1, column=1, padx=2)
            
            # Buttons
            # REV (-speed)
            btn_rev = ttk.Button(frame_indiv, text="REV", width=5, 
                command=lambda idx=i: self.set_motor_speed(idx, -1))
            btn_rev.grid(row=i+1, column=2, padx=1)
            
            # STOP (0)
            btn_stop = ttk.Button(frame_indiv, text="STOP", width=5, 
                command=lambda idx=i: self.set_motor_speed(idx, 0))
            btn_stop.grid(row=i+1, column=3, padx=1)
            
            # FWD (+speed)
            btn_fwd = ttk.Button(frame_indiv, text="FWD", width=5, 
                command=lambda idx=i: self.set_motor_speed(idx, 1))
            btn_fwd.grid(row=i+1, column=4, padx=1)

    def set_motor_speed(self, motor_idx, direction):
        # direction: -1 (Rev), 0 (Stop), 1 (Fwd)
        try:
            speed_mag = abs(self.indiv_speed_vars[motor_idx].get())
            final_speed = speed_mag * direction
            
            # Construct packet: ID, Mode=3 (Speed), Value=final_speed
            header = struct.pack('BBB', 0x00, motor_idx, 3) 
            data = struct.pack('>h', final_speed)
            
            if self.ser and self.connected:
                self.ser.write(header + data)
                self.log(f"M{motor_idx} Speed Set: {final_speed} (Mode 3)")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def create_control_panel(self, parent):
        # 1. 串口设置区域
        frame_conn = ttk.LabelFrame(parent, text="Connection")
        frame_conn.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(frame_conn, text="Port:").pack(side="left", padx=5)
        ttk.Entry(frame_conn, textvariable=self.port_var, width=10).pack(side="left", padx=5)
        self.btn_connect = ttk.Button(frame_conn, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)
        
        # New: System Control (0x01)
        frame_sys = ttk.LabelFrame(parent, text="System Control (0x01)")
        frame_sys.pack(fill="x", padx=10, pady=5)
        
        btn_halt = ttk.Button(frame_sys, text="EMERGENCY STOP (0x00)", command=self.action_emergency_stop)
        btn_halt.pack(side="left", padx=5, fill="x", expand=True)
        # Style the stop button red if possible, or just text
        
        ttk.Label(frame_sys, text="Set Task:").pack(side="left", padx=5)
        self.run_task_var = tk.IntVar(value=1)
        ttk.Entry(frame_sys, textvariable=self.run_task_var, width=5).pack(side="left", padx=2)
        ttk.Button(frame_sys, text="Set RunningTask", command=self.action_set_task).pack(side="left", padx=5)

        # 2. 快捷控制区域 (New Features)
        frame_actions = ttk.LabelFrame(parent, text="Quick Actions")
        frame_actions.pack(fill="x", padx=10, pady=5)
        
        # Dart Launch Controls
        # Row 1: Friction Control
        frame_launch_1 = ttk.Frame(frame_actions)
        frame_launch_1.pack(fill="x", padx=5, pady=2)
        ttk.Label(frame_launch_1, text="Friction Control:").pack(side="left")
        
        # Friction Speed Input
        ttk.Label(frame_launch_1, text="Speed 1:").pack(side="left", padx=2)
        ttk.Entry(frame_launch_1, textvariable=self.friction_speed_var, width=6).pack(side="left", padx=2)

        ttk.Label(frame_launch_1, text="Speed 2:").pack(side="left", padx=2)
        ttk.Entry(frame_launch_1, textvariable=self.friction_speed_2_var, width=6).pack(side="left", padx=2)
        
        ttk.Button(frame_launch_1, text="Prepare Launch (Friction ON)", command=self.action_prepare_launch).pack(side="left", padx=5)
        ttk.Button(frame_launch_1, text="Stop Friction (OFF)", command=self.action_stop_friction).pack(side="left", padx=5)

        # Row 2: Feed & Launch
        frame_launch_2 = ttk.Frame(frame_actions)
        frame_launch_2.pack(fill="x", padx=5, pady=2)
        ttk.Label(frame_launch_2, text="Launch Actions:").pack(side="left")
        ttk.Button(frame_launch_2, text="Feed (In)", command=self.action_feed_in).pack(side="left", padx=5)
        ttk.Button(frame_launch_2, text="Feed (Out)", command=self.action_feed_out).pack(side="left", padx=5)
        
        ttk.Button(frame_launch_2, text="ONE CLICK LAUNCH", command=self.action_one_click_launch).pack(side="left", padx=5)

        # Loading Mechanism (Lift - ID 6)
        frame_load = ttk.Frame(frame_actions)
        frame_load.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_load, text="Reload (Lift ID 6):").pack(side="left")
        ttk.Button(frame_load, text="Reload 1 (Angle 4805000)", command=lambda: self.action_lift_to(4805000)).pack(side="left", padx=5)
        ttk.Button(frame_load, text="Reload 2 (Angle 8000000)", command=lambda: self.action_lift_to(8000000)).pack(side="left", padx=5)
        ttk.Button(frame_load, text="Reset Load (Stall -6000)", command=self.action_reset_load).pack(side="left", padx=5)
        
        # Yaw Control (ID 4)
        frame_yaw = ttk.Frame(frame_actions)
        frame_yaw.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_yaw, text="Yaw Control (ID 4):").pack(side="left")
        self.yaw_scale = ttk.Scale(frame_yaw, from_=-245000, to=245000, variable=self.yaw_angle_var, orient="horizontal", length=300)
        self.yaw_scale.pack(side="left", padx=5)
        self.lbl_yaw_val = ttk.Label(frame_yaw, text="0")
        self.lbl_yaw_val.pack(side="left", padx=5)
        
        # Yaw Input Entry
        entry_yaw = ttk.Entry(frame_yaw, textvariable=self.yaw_angle_var, width=10)
        entry_yaw.pack(side="left", padx=5)
        entry_yaw.bind('<Return>', lambda e: self.send_yaw_command())

        ttk.Button(frame_yaw, text="Set", command=self.send_yaw_command).pack(side="left", padx=5)
        
        # Slider value update label
        self.yaw_scale.bind("<Motion>", self.update_yaw_label)
        # Send on release
        self.yaw_scale.bind("<ButtonRelease-1>", self.send_yaw_command)
        
        # 3. 独立电机控制区域 (Individual Control)
        self.create_individual_controls(parent)
        
        # 4. 通用调试区域 (General Debug)
        frame_ctrl = ttk.LabelFrame(parent, text="General Debug Panel")
        frame_ctrl.pack(fill="x", padx=10, pady=5)
        
        # 电机编号选择
        frame_id = ttk.Frame(frame_ctrl)
        frame_id.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_id, text="Motor ID (0-6):").pack(side="left")
        for i in range(7):
            ttk.Radiobutton(frame_id, text=str(i), variable=self.motor_id_var, value=i).pack(side="left", padx=2)
            
        # 模式选择
        frame_mode = ttk.Frame(frame_ctrl)
        frame_mode.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_mode, text="Mode:").pack(side="left")
        mode_cb = ttk.Combobox(frame_mode, textvariable=self.mode_var, values=list(self.modes.keys()), state="readonly")
        mode_cb.pack(side="left", padx=5)
        mode_cb.current(0)
        
        # 数值输入
        frame_val = ttk.Frame(frame_ctrl)
        frame_val.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_val, text="Value (int16):").pack(side="left")
        entry_val = ttk.Entry(frame_val, textvariable=self.value_var, width=10)
        entry_val.pack(side="left", padx=5)
        entry_val.bind('<Return>', lambda e: self.send_packet())
        
        ttk.Label(frame_val, text="Time (ms):").pack(side="left", padx=5)
        entry_time = ttk.Entry(frame_val, textvariable=self.time_var, width=10)
        entry_time.pack(side="left", padx=5)
        
        scale_val = ttk.Scale(frame_val, from_=-10000, to=10000, variable=self.value_var, orient="horizontal", length=200)
        scale_val.pack(side="left", padx=5)
        
        # 发送按钮
        frame_btns = ttk.Frame(frame_ctrl)
        frame_btns.pack(fill="x", padx=5, pady=5)
        ttk.Button(frame_btns, text="SEND COMMAND", command=self.send_packet).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(frame_btns, text="STOP ALL (Send Disable)", command=self.stop_motor).pack(side="left", fill="x", expand=True, padx=5)
        
        # 4. 日志区域
        frame_log = ttk.LabelFrame(parent, text="Log (TX/Raw RX)")
        frame_log.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = tk.Text(frame_log, height=10, state="disabled")
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(frame_log, orient="vertical", command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)
        
    def update_yaw_label(self, event=None):
        # Display as -245000 to +245000, centered at 245000
        raw_val = self.yaw_angle_var.get()
        display_val = int(raw_val)
        self.lbl_yaw_val.config(text=f"{display_val}")
        
    def toggle_connection(self):
        if not self.connected:
            try:
                self.ser = serial.Serial(self.port_var.get(), BAUDRATE, timeout=0.1)
                self.connected = True
                self.btn_connect.config(text="Disconnect")
                self.log("Connected to " + self.port_var.get())
                self.rx_thread = threading.Thread(target=self.rx_task, daemon=True)
                self.rx_thread.start()
            except serial.SerialException as e:
                messagebox.showerror("Error", str(e))
        else:
            self.connected = False
            if self.ser:
                self.ser.close()
            self.btn_connect.config(text="Connect")
            self.log("Disconnected")

    def rx_task(self):
        # 接收缓冲区
        rx_buffer = b''
        PACKET_LEN = 14 # Updated to 14 bytes to include total_angle
        
        while self.connected:
            try:
                if self.ser and self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    rx_buffer += data
                    
                    # 尝试解析完整数据包
                    while len(rx_buffer) >= PACKET_LEN:
                        # 寻找帧头 0x81
                        if rx_buffer[0] != 0x81:
                            # 优化: 使用 index 快速跳过无效数据，而不是切片遍历
                            try:
                                idx = rx_buffer.index(0x81)
                                rx_buffer = rx_buffer[idx:]
                            except ValueError:
                                # 缓冲区内没有帧头，清空（或保留最后几个字节以防分包，这里简化处理）
                                rx_buffer = b''
                                break
                            continue
                        
                        # 再次检查长度（可能跳过数据后不足一包）
                        if len(rx_buffer) < PACKET_LEN:
                            break

                        # 取出一个包
                        packet = rx_buffer[:PACKET_LEN]
                        rx_buffer = rx_buffer[PACKET_LEN:]
                        
                        self.parse_feedback(packet)
                else:
                    time.sleep(0.002) # 稍微增加休眠以降低CPU占用
            except Exception as e:
                print(f"RX Error: {e}")
                self.connected = False
                break
                
    def parse_feedback(self, packet):
        try:
            # Unpack: 0x81 (ignored), ID, AngH, AngL, RpmH, RpmL, TrqH, TrqL, Tmp, Flags, TotAng(4 bytes)
            
            motor_id = packet[1]
            if motor_id >= MOTOR_NUM: return
            
            # Big Endian integers
            angle = (packet[2] << 8) | packet[3]
            rpm = (packet[4] << 8) | packet[5]
            torque = (packet[6] << 8) | packet[7]
            
            # Signed conversions (int16)
            if rpm > 32767: rpm -= 65536
            if torque > 32767: torque -= 65536
            
            temp = packet[8] # int8
            if temp > 127: temp -= 256
            
            flags = packet[9]
            enabled = (flags >> 7) & 0x01
            stalled = (flags >> 6) & 0x01
            mode = flags & 0x3F # bits 0-5
            
            # Total Angle (int32, big endian)
            total_angle = (packet[10] << 24) | (packet[11] << 16) | (packet[12] << 8) | packet[13]
            # Handle int32 sign
            if total_angle >= 0x80000000:
                total_angle -= 0x100000000

            # 更新数据模型 - 使用 update 避免频繁创建新字典对象
            with self.lock:
                self.motor_data[motor_id].update({
                    'angle': angle,
                    'rpm': rpm,
                    'torque': torque,
                    'temp': temp,
                    'enabled': enabled,
                    'stalled': stalled,
                    'mode': mode,
                    'total_angle': total_angle
                })
                
        except Exception as e:
            print(f"Parse Error: {e}")

    def update_monitor_ui(self):
        # 在主线程更新UI
        with self.lock:
            for i in range(MOTOR_NUM):
                data = self.motor_data[i]
                
                if i < len(self.motor_labels):
                    labels = self.motor_labels[i]
                    
                    # 状态颜色
                    if data['stalled']:
                        status_text = "STALL"
                        status_color = "red"
                    elif data['enabled']:
                        status_text = "ENA"
                        status_color = "green"
                    else:
                        status_text = "DIS"
                        status_color = "gray"
                        
                    labels['stat'].config(text=status_text, foreground=status_color)
                    labels['mode'].config(text=str(data['mode']))
                    labels['angle'].config(text=str(data['angle']))
                    labels['rpm'].config(text=str(data['rpm']))
                    labels['trq'].config(text=str(data['torque']))
                    
                    # 温度报警色
                    temp_color = "black"
                    if data['temp'] > 50: temp_color = "orange"
                    if data['temp'] > 70: temp_color = "red"
                    labels['tmp'].config(text=f"{data['temp']}°C", foreground=temp_color)
                    
                    # Update Total Angle Label
                    if 'total_angle' in labels:
                        labels['total_angle'].config(text=str(data.get('total_angle', 0)))

        # 循环调用自己
        self.root.after(100, self.update_monitor_ui)

    def send_raw_packet(self, mid, mode, data_bytes):
        if not self.connected or not self.ser:
            # self.log("Error: Not Connected", "red") 
            return
            
        # Byte 0: 0x00
        # Byte 1: ID
        # Byte 2: Mode
        # Remaining: data_bytes
        header = struct.pack('BBB', 0x00, mid, mode)
        packet = header + data_bytes
        
        try:
            self.ser.write(packet)
            # TX Log (Optional: reduce log spam)
            # self.log(f"TX: {packet.hex().upper()}", "blue")
        except Exception as e:
            self.log(f"Send Error: {e}", "red")

    def send_packet(self):
        try:
            mid = self.motor_id_var.get()
            mode_str = self.mode_var.get()
            mode = self.modes[mode_str]
            val = int(self.value_var.get())
            val = max(-32768, min(32767, val))
            
            # Simple packet for standard modes + mode 5
            # Mode 5 uses same structure (val interpreted as speed)
            if mode == 7:
                # Mode 7: SpeedTimeMode
                # Speed (int16), Time (uint32)
                time_ms = int(self.time_var.get())
                packet_data = struct.pack('>hI', val, time_ms)
                self.send_raw_packet(mid, mode, packet_data)
                self.log(f"Sent ID:{mid} Mode:{mode} Speed:{val} Time:{time_ms}")
            else:
                packet_data = struct.pack('>h', val)
                self.send_raw_packet(mid, mode, packet_data)
                self.log(f"Sent ID:{mid} Mode:{mode} Val:{val}")
            
        except ValueError:
            messagebox.showerror("Error", "Invalid value format")

    def action_prepare_launch(self):
        # Motor 0,2: Speed 1 (1st stage) - Motor 0 Neg, Motor 2 Pos
        # Motor 1,3: Speed 2 (2nd stage) - Motor 1 Neg, Motor 3 Pos
        try:
            target_speed_1 = int(self.friction_speed_var.get())
        except ValueError:
            target_speed_1 = 4000
            self.log("Invalid speed 1, using default 4000", "red")

        try:
            target_speed_2 = int(self.friction_speed_2_var.get())
        except ValueError:
            target_speed_2 = 4000
            self.log("Invalid speed 2, using default 4000", "red")
            
        speed1_neg = -abs(target_speed_1)
        speed1_pos = abs(target_speed_1)
        
        speed2_neg = -abs(target_speed_2)
        speed2_pos = abs(target_speed_2)
        
        # Mode 3 = Speed Mode
        data1_neg = struct.pack('>h', speed1_neg)
        data1_pos = struct.pack('>h', speed1_pos)
        
        data2_neg = struct.pack('>h', speed2_neg)
        data2_pos = struct.pack('>h', speed2_pos)
        
        # Motor 0 (Stage 1 Neg)
        self.send_raw_packet(0, 3, data1_neg)
        time.sleep(0.01)
        
        # Motor 1 (Stage 2 Neg)
        self.send_raw_packet(1, 3, data2_neg)
        time.sleep(0.01)
        
        # Motor 2 (Stage 1 Pos)
        self.send_raw_packet(2, 3, data1_pos)
        time.sleep(0.01)
        
        # Motor 3 (Stage 2 Pos)
        self.send_raw_packet(3, 3, data2_pos)
        
        self.log(f"Friction ON. Stage 1: {target_speed_1}, Stage 2: {target_speed_2}", "green")

    def action_stop_friction(self):
        # Stop motors 0,1,2,3 (Set Speed to 0)
        speed_zero = 0
        data_zero = struct.pack('>h', speed_zero)
        
        for mid in range(4):
            self.send_raw_packet(mid, 3, data_zero) # Mode 3 (Speed) -> 0
            time.sleep(0.01)

    def action_one_click_launch(self):
        # 1. Prepare Launch (Friction ON)
        self.log(">>> [One-Click] Starting Launch Sequence...", "blue")
        self.action_prepare_launch()
        
        # 2. Set Task (triggers launch logic on MCU)
        # Add a small delay to ensure friction commands are processed if needed, 
        # though send_raw_packet is synchronous.
        time.sleep(0.05) 
        self.action_set_task()
        self.log(">>> [One-Click] Launch Sequence Triggered", "blue")

    def action_feed_in(self):
        # 0,1 are +100, 2,3 are -100
        # Mode 3 (Speed)
        speed_pos = 100
        speed_neg = -100
        data_pos = struct.pack('>h', speed_pos)
        data_neg = struct.pack('>h', speed_neg)
        
        self.send_raw_packet(0, 3, data_pos)
        time.sleep(0.01)
        self.send_raw_packet(1, 3, data_pos)
        time.sleep(0.01)
        self.send_raw_packet(2, 3, data_neg)
        time.sleep(0.01)
        self.send_raw_packet(3, 3, data_neg)
        self.log("Feed IN (0,1=+100, 2,3=-100)", "green")

    def action_feed_out(self):
        # Opposite: 0,1 are -100, 2,3 are +100
        speed_pos = 100
        speed_neg = -100
        data_pos = struct.pack('>h', speed_pos)
        data_neg = struct.pack('>h', speed_neg)
        
        self.send_raw_packet(0, 3, data_neg)
        time.sleep(0.01)
        self.send_raw_packet(1, 3, data_neg)
        time.sleep(0.01)
        self.send_raw_packet(2, 3, data_pos)
        time.sleep(0.01)
        self.send_raw_packet(3, 3, data_pos)
        self.log("Feed OUT (0,1=-100, 2,3=+100)", "green")

    def action_lift_to(self, angle):
        # Motor 6 (Load) -> Mode 6 (RunToAngle)
        # Angle: double (Big Endian)
        # Speed: int16 (Big Endian) -> Let's use 3000 as default speed
        
        speed = 5000
        mid = 6 # Lift
        mode = 6 # RunToAngle
        
        # struct.pack('>d', angle) uses Big Endian double
        data = struct.pack('>dh', float(angle), speed)
        
        self.send_raw_packet(mid, mode, data)

    def action_reset_load(self):
        # Motor 6 (Load) -> Mode 5 (RunToStall)
        # Speed: -6000
        speed = -6000
        mid = 6
        mode = 5
        
        data = struct.pack('>h', speed)
        self.send_raw_packet(mid, mode, data)

    def action_emergency_stop(self):
        # Header 0x01, Cmd 0x00
        if not self.connected or not self.ser: return
        packet = struct.pack('BB', 0x01, 0x00)
        try:
            self.ser.write(packet)
            self.log("SENT EMERGENCY STOP (0x01, 0x00)", "red")
        except Exception as e:
            self.log(f"Stop Error: {e}", "red")

    def action_set_task(self):
        # Header 0x01, Cmd = TaskID
        try:
            task_id = self.run_task_var.get()
            if task_id == 0:
                self.log("Task ID must be != 0 for execution", "orange")
                # Though protocol allows sending whatever, user logic implies 0 is idle/reset in checking loops maybe? 
                # Actually user code checks: if(RunningTask==1)... set to 0. 
                # So we just send whatever user types.
            
            if not self.connected or not self.ser: return
            
            # Ensure byte range
            task_id = max(0, min(255, task_id))
            
            packet = struct.pack('BB', 0x01, task_id)
            self.ser.write(packet)
            self.log(f"Set RunningTask = {task_id}")
        except ValueError:
            self.log("Invalid Task ID", "red")
        except Exception as e:
            self.log(f"Send Error: {e}", "red")

    def send_yaw_command(self, event=None):
        # Motor 4 (Yaw) -> Mode 6 (RunToAngle)
        angle = self.yaw_angle_var.get() + 245000
        speed = 300 # Default speed
        mid = 4
        mode = 6
        
        data = struct.pack('>dh', float(angle), speed)
        self.send_raw_packet(mid, mode, data)
        self.log(f"Set Yaw: {int(self.yaw_angle_var.get())} (Raw: {int(angle)})", "purple")

    def stop_motor(self):
        current_mode = self.mode_var.get()
        self.mode_var.set("Disable (0x00)")
        self.value_var.set(0)
        self.send_packet()

    def log(self, msg, color="black"):
        if self.log_text:
            self.log_text.config(state="normal")
            self.log_text.insert("end", msg + "\n")
            line_count = int(self.log_text.index('end-1c').split('.')[0])
            self.log_text.tag_add("color", f"{line_count}.0", f"{line_count}.end")
            self.log_text.tag_config("color", foreground=color)
            self.log_text.see("end")
            self.log_text.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlApp(root)
    root.mainloop()
