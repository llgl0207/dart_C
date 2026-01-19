import serial
import struct
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

# 配置
DEFAULT_PORT = 'COM27'
BAUDRATE = 115200

class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboMaster Dart Motor Control")
        self.root.geometry("700x650")
        
        self.ser = None
        self.connected = False
        
        # 变量
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.motor_id_var = tk.IntVar(value=0)
        self.mode_var = tk.StringVar(value="Disable")
        self.value_var = tk.IntVar(value=0)
        self.yaw_angle_var = tk.DoubleVar(value=245000)
        self.log_text = None
        
        # 模式映射
        self.modes = {
            "Disable (0x00)": 0,
            "Current (0x01)": 1,
            "Angle (0x02)": 2,
            "Speed (0x03)": 3,
            "Torque (0x04)": 4,
            "RunToStall (0x05)": 5,
            # Mode 6 is hidden from dropdown as it requires special payload
        }
        
        self.create_ui()
        
    def create_ui(self):
        # 1. 串口设置区域
        frame_conn = ttk.LabelFrame(self.root, text="Connection")
        frame_conn.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(frame_conn, text="Port:").pack(side="left", padx=5)
        ttk.Entry(frame_conn, textvariable=self.port_var, width=10).pack(side="left", padx=5)
        self.btn_connect = ttk.Button(frame_conn, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)
        
        # 2. 快捷控制区域 (New Features)
        frame_actions = ttk.LabelFrame(self.root, text="Quick Actions")
        frame_actions.pack(fill="x", padx=10, pady=5)
        
        # Dart Launch Controls
        frame_launch = ttk.Frame(frame_actions)
        frame_launch.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_launch, text="Dart Launch:").pack(side="left")
        ttk.Button(frame_launch, text="Prepare Launch (Friction ON)", command=self.action_prepare_launch).pack(side="left", padx=5)
        ttk.Button(frame_launch, text="Stop Friction (OFF)", command=self.action_stop_friction).pack(side="left", padx=5)
        
        # Loading Mechanism (Lift - ID 6)
        frame_load = ttk.Frame(frame_actions)
        frame_load.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_load, text="Reload (Lift ID 6):").pack(side="left")
        ttk.Button(frame_load, text="Reload 1 (Angle 4805000)", command=lambda: self.action_lift_to(4805000)).pack(side="left", padx=5)
        ttk.Button(frame_load, text="Reload 2 (Angle 8000000)", command=lambda: self.action_lift_to(8000000)).pack(side="left", padx=5)
        ttk.Button(frame_load, text="Reset Load (Angle 0)", command=lambda: self.action_lift_to(0)).pack(side="left", padx=5)
        
        # Yaw Control (ID 4)
        frame_yaw = ttk.Frame(frame_actions)
        frame_yaw.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_yaw, text="Yaw Control (ID 4):").pack(side="left")
        self.yaw_scale = ttk.Scale(frame_yaw, from_=-245000, to=245000, variable=self.yaw_angle_var, orient="horizontal", length=300)
        self.yaw_scale.pack(side="left", padx=5)
        self.lbl_yaw_val = ttk.Label(frame_yaw, text="0")
        self.lbl_yaw_val.pack(side="left", padx=5)
        
        # Slider value update label
        self.yaw_scale.bind("<Motion>", self.update_yaw_label)
        # Send on release
        self.yaw_scale.bind("<ButtonRelease-1>", self.send_yaw_command)
        
        # 3. 通用调试区域 (General Debug)
        frame_ctrl = ttk.LabelFrame(self.root, text="General Debug Panel")
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
        
        scale_val = ttk.Scale(frame_val, from_=-10000, to=10000, variable=self.value_var, orient="horizontal", length=200)
        scale_val.pack(side="left", padx=5)
        
        # 发送按钮
        frame_btns = ttk.Frame(frame_ctrl)
        frame_btns.pack(fill="x", padx=5, pady=5)
        ttk.Button(frame_btns, text="SEND COMMAND", command=self.send_packet).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(frame_btns, text="STOP ALL (Send Disable)", command=self.stop_motor).pack(side="left", fill="x", expand=True, padx=5)
        
        # 4. 日志区域
        frame_log = ttk.LabelFrame(self.root, text="Log (TX/RX)")
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
        while self.connected:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read_all()
                    if data:
                        self.root.after(0, self.log, f"RX: {data.hex().upper()}", "green")
                time.sleep(0.01)
            except Exception as e:
                self.connected = False
                break

    def send_raw_packet(self, mid, mode, data_bytes):
        if not self.connected or not self.ser:
            self.log("Error: Not Connected", "red")
            return
            
        # Byte 0: 0x00
        # Byte 1: ID
        # Byte 2: Mode
        # Remaining: data_bytes
        header = struct.pack('BBB', 0x00, mid, mode)
        packet = header + data_bytes
        
        try:
            self.ser.write(packet)
            self.log(f"TX: {packet.hex().upper()}", "blue")
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
            packet_data = struct.pack('>h', val)
            self.send_raw_packet(mid, mode, packet_data)
            
        except ValueError:
            messagebox.showerror("Error", "Invalid value format")

    def action_prepare_launch(self):
        # Motor 0,1: Speed -4000
        # Motor 2,3: Speed +4000
        speed_neg = -4000
        speed_pos = 4000
        
        # Mode 3 = Speed Mode
        data_neg = struct.pack('>h', speed_neg)
        data_pos = struct.pack('>h', speed_pos)
        
        self.send_raw_packet(0, 3, data_neg)
        time.sleep(0.01)
        self.send_raw_packet(1, 3, data_neg)
        time.sleep(0.01)
        self.send_raw_packet(2, 3, data_pos)
        time.sleep(0.01)
        self.send_raw_packet(3, 3, data_pos)

    def action_stop_friction(self):
        # Stop motors 0,1,2,3 (Set Speed to 0)
        speed_zero = 0
        data_zero = struct.pack('>h', speed_zero)
        
        for mid in range(4):
            self.send_raw_packet(mid, 3, data_zero) # Mode 3 (Speed) -> 0
            time.sleep(0.01)

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
