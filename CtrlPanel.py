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
        self.root.geometry("600x450")
        
        self.ser = None
        self.connected = False
        
        # 变量
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.motor_id_var = tk.IntVar(value=0)
        self.mode_var = tk.StringVar(value="Disable")
        self.value_var = tk.IntVar(value=0)
        self.log_text = None
        
        # 模式映射
        self.modes = {
            "Disable (0x00)": 0,
            "Current (0x01)": 1,
            "Angle (0x02)": 2,
            "Speed (0x03)": 3,
            "Torque (0x04)": 4
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
        
        # 2. 控制区域
        frame_ctrl = ttk.LabelFrame(self.root, text="Control Panel")
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
        
        # 数值输入 (Entry + Slider)
        frame_val = ttk.Frame(frame_ctrl)
        frame_val.pack(fill="x", padx=5, pady=5)
        ttk.Label(frame_val, text="Value (int16):").pack(side="left")
        
        entry_val = ttk.Entry(frame_val, textvariable=self.value_var, width=10)
        entry_val.pack(side="left", padx=5)
        
        # 绑定回车发送
        entry_val.bind('<Return>', lambda e: self.send_packet())
        
        scale_val = ttk.Scale(frame_val, from_=-10000, to=10000, variable=self.value_var, orient="horizontal", length=300)
        scale_val.pack(side="left", padx=5)
        
        # 发送按钮
        frame_btns = ttk.Frame(frame_ctrl)
        frame_btns.pack(fill="x", padx=5, pady=5)
        ttk.Button(frame_btns, text="SEND COMMAND", command=self.send_packet).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(frame_btns, text="STOP ALL (Send Disable)", command=self.stop_motor).pack(side="left", fill="x", expand=True, padx=5)
        
        # 3. 日志区域
        frame_log = ttk.LabelFrame(self.root, text="Log (TX/RX)")
        frame_log.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = tk.Text(frame_log, height=10, state="disabled")
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(frame_log, orient="vertical", command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)
        
    def toggle_connection(self):
        if not self.connected:
            try:
                self.ser = serial.Serial(self.port_var.get(), BAUDRATE, timeout=0.1)
                self.connected = True
                self.btn_connect.config(text="Disconnect")
                self.log("Connected to " + self.port_var.get())
                
                # 开启接收线程
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
                        # 在主线程更新UI
                        self.root.after(0, self.log, f"RX: {data.hex().upper()}", "green")
                time.sleep(0.01)
            except Exception as e:
                self.connected = False
                self.root.after(0, self.btn_connect.config, {"text": "Connect"})
                self.root.after(0, self.log, f"Serial Error: {e}", "red")
                break

    def send_packet(self):
        if not self.connected or not self.ser:
            messagebox.showwarning("Warning", "Please connect serial port first!")
            return
            
        try:
            mid = self.motor_id_var.get()
            mode_str = self.mode_var.get()
            mode = self.modes[mode_str]
            val = int(self.value_var.get())
            
            # 限制范围 int16
            val = max(-32768, min(32767, val))
            
            # 构造协议包
            # Byte 0: 0x00 (Header)
            # Byte 1: ID
            # Byte 2: Mode
            # Byte 3-4: Value (Big Endian int16)
            packet = struct.pack('>BBBh', 0x00, mid, mode, val)
            
            self.ser.write(packet)
            self.log(f"TX: {packet.hex().upper()} (ID={mid}, Mode={mode}, Val={val})", "blue")
            
        except ValueError:
            messagebox.showerror("Error", "Invalid value format")

    def stop_motor(self):
        # 强制将当前选中的电机模式设为Disable
        current_mode = self.mode_var.get()
        self.mode_var.set("Disable (0x00)")
        self.value_var.set(0)
        self.send_packet()
        # 恢复之前的模式选择显示的字串(可选，这里就不恢复了)

    def log(self, msg, color="black"):
        self.log_text.config(state="normal")
        self.log_text.insert("end", msg + "\n")
        
        # 简单的语法高亮
        line_count = int(self.log_text.index('end-1c').split('.')[0])
        self.log_text.tag_add("color", f"{line_count}.0", f"{line_count}.end")
        self.log_text.tag_config("color", foreground=color)
        
        self.log_text.see("end")
        self.log_text.config(state="disabled")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlApp(root)
    root.mainloop()
