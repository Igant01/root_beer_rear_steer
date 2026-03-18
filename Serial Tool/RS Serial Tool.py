import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

class RearSteerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Root Beer Buggy - Rear Steer Control")
        self.root.geometry("600x800")
        self.root.resizable(False, False)
        
        self.ser = None
        self.connected = False
        self.read_thread = None
        self.running = False
        self.shutting_down = False  # Flag to prevent callbacks during shutdown
        self.normal_operation_mode = False  # False = USB Control, True = Normal Operation
        self.sweep_running = False  # Track sweep state
        self.write_protection_locked = True  # Track write protection state
        self.motor_enabled = False  # Track motor enabled state
        
        # PID Parameters
        self.torque_kp_var = tk.DoubleVar(value=1.0)
        self.torque_ki_var = tk.DoubleVar(value=0.1)
        self.torque_kd_var = tk.DoubleVar(value=0.01)
        self.velocity_kp_var = tk.DoubleVar(value=0.5)
        self.velocity_ki_var = tk.DoubleVar(value=0.05)
        self.velocity_kd_var = tk.DoubleVar(value=0.005)
        self.position_kp_var = tk.DoubleVar(value=0.3)
        self.position_ki_var = tk.DoubleVar(value=0.03)
        self.position_kd_var = tk.DoubleVar(value=0.003)
        
        # Plot sample count setting - same number of samples for all loops
        self.plot_sample_count_var = tk.IntVar(value=200)
        # Time interval per sample (ms) for each loop
        self.torque_interval_var = tk.DoubleVar(value=1.0)    # 1ms
        self.velocity_interval_var = tk.DoubleVar(value=50.0)  # 50ms
        self.position_interval_var = tk.DoubleVar(value=200.0) # 200ms
        
        # Plot data buffers (timestamps in ms from Teensy)
        self.torque_time_data = deque(maxlen=2000)
        self.torque_target_data = deque(maxlen=2000)
        self.torque_actual_data = deque(maxlen=2000)
        self.velocity_time_data = deque(maxlen=2000)
        self.velocity_target_data = deque(maxlen=2000)
        self.velocity_actual_data = deque(maxlen=2000)
        self.position_time_data = deque(maxlen=2000)
        self.position_target_data = deque(maxlen=2000)
        self.position_actual_data = deque(maxlen=2000)
        
        # Data collection flags
        self.collecting_torque_data = False
        self.collecting_velocity_data = False
        self.collecting_position_data = False
        
        # Plot windows
        self.torque_plot_window = None
        self.velocity_plot_window = None
        self.position_plot_window = None
        
        self.setup_ui()
        self.update_ports()
        
    def setup_ui(self):
        """Setup the user interface"""
        
        # Create main canvas with scrollbar
        main_canvas = tk.Canvas(self.root)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=main_canvas.yview)
        scrollable_frame = ttk.Frame(main_canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: main_canvas.configure(scrollregion=main_canvas.bbox("all"))
        )
        
        main_canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        main_canvas.configure(yscrollcommand=scrollbar.set)
        
        # Allow mousewheel scrolling (Windows/MacOS)
        def _on_mousewheel(event):
            main_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        # Allow mousewheel scrolling (Linux)
        def _on_scroll_up(event):
            main_canvas.yview_scroll(-1, "units")
        
        def _on_scroll_down(event):
            main_canvas.yview_scroll(1, "units")
        
        main_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        main_canvas.bind_all("<Button-4>", _on_scroll_up)
        main_canvas.bind_all("<Button-5>", _on_scroll_down)
        
        main_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(scrollable_frame, text="Connection", padding="10")
        conn_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(conn_frame, text="Serial Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        
        ttk.Button(conn_frame, text="Refresh Ports", command=self.update_ports).grid(row=0, column=2, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=1, column=0, pady=5)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=1, column=1, columnspan=2, sticky="w", padx=5)
        
        # SAS Calibration Frame (combined with pot values and limits)
        sas_frame = ttk.LabelFrame(scrollable_frame, text="SAS Calibration & Configuration", padding="10")
        sas_frame.pack(fill="x", padx=10, pady=10)
        
        # Front SAS Section
        front_frame = ttk.LabelFrame(sas_frame, text="Front SAS", padding="10")
        front_frame.pack(fill="x", padx=5, pady=5)
        
        # Front calibration buttons
        front_button_frame = ttk.Frame(front_frame)
        front_button_frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(front_button_frame, text="Set Left", command=self.set_front_left).pack(side="left", padx=5)
        ttk.Button(front_button_frame, text="Set Right", command=self.set_front_right).pack(side="left", padx=5)
        
        # Front raw pot values (real-time)
        front_pot_frame = ttk.Frame(front_frame)
        front_pot_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(front_pot_frame, text="Pot A:").grid(row=0, column=0, sticky="w", padx=5)
        self.front_a_label = ttk.Label(front_pot_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.front_a_label.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(front_pot_frame, text="Pot B:").grid(row=0, column=2, sticky="w", padx=5)
        self.front_b_label = ttk.Label(front_pot_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.front_b_label.grid(row=0, column=3, sticky="w", padx=5)
        
        # Front limits (working values only)
        front_limits_frame = ttk.Frame(front_frame)
        front_limits_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(front_limits_frame, text="Left Limit:").grid(row=0, column=0, sticky="w", padx=5)
        self.front_leftlimit_label = ttk.Label(front_limits_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.front_leftlimit_label.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(front_limits_frame, text="Right Limit:").grid(row=0, column=2, sticky="w", padx=5)
        self.front_rightlimit_label = ttk.Label(front_limits_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.front_rightlimit_label.grid(row=0, column=3, sticky="w", padx=5)
        
        # Rear SAS Section
        rear_frame = ttk.LabelFrame(sas_frame, text="Rear SAS", padding="10")
        rear_frame.pack(fill="x", padx=5, pady=5)
        
        # Rear calibration buttons
        rear_button_frame = ttk.Frame(rear_frame)
        rear_button_frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(rear_button_frame, text="Set Left", command=self.set_rear_left).pack(side="left", padx=5)
        ttk.Button(rear_button_frame, text="Set Right", command=self.set_rear_right).pack(side="left", padx=5)
        ttk.Button(rear_button_frame, text="Auto-Calibrate", command=self.auto_calibrate).pack(side="left", padx=5)
        self.calibration_status_label = ttk.Label(rear_button_frame, text="", foreground="blue")
        self.calibration_status_label.pack(side="left", padx=5)
        
        # Rear raw pot values (real-time)
        rear_pot_frame = ttk.Frame(rear_frame)
        rear_pot_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(rear_pot_frame, text="Pot A:").grid(row=0, column=0, sticky="w", padx=5)
        self.rear_a_label = ttk.Label(rear_pot_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.rear_a_label.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(rear_pot_frame, text="Pot B:").grid(row=0, column=2, sticky="w", padx=5)
        self.rear_b_label = ttk.Label(rear_pot_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.rear_b_label.grid(row=0, column=3, sticky="w", padx=5)
        
        # Rear limits (working values only)
        rear_limits_frame = ttk.Frame(rear_frame)
        rear_limits_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(rear_limits_frame, text="Left Limit:").grid(row=0, column=0, sticky="w", padx=5)
        self.rear_leftlimit_label = ttk.Label(rear_limits_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.rear_leftlimit_label.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(rear_limits_frame, text="Right Limit:").grid(row=0, column=2, sticky="w", padx=5)
        self.rear_rightlimit_label = ttk.Label(rear_limits_frame, text="--", foreground="blue", font=("Arial", 9, "bold"))
        self.rear_rightlimit_label.grid(row=0, column=3, sticky="w", padx=5)
        
        # EEPROM Values Display Frame
        eeprom_frame = ttk.LabelFrame(scrollable_frame, text="EEPROM Values (Saved)", padding="10")
        eeprom_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(eeprom_frame, text="Rear - Left:").grid(row=0, column=0, sticky="w", padx=5)
        self.eeprom_rear_left_label = ttk.Label(eeprom_frame, text="--", foreground="darkgreen", font=("Arial", 9, "bold"))
        self.eeprom_rear_left_label.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(eeprom_frame, text="Right:").grid(row=0, column=2, sticky="w", padx=5)
        self.eeprom_rear_right_label = ttk.Label(eeprom_frame, text="--", foreground="darkgreen", font=("Arial", 9, "bold"))
        self.eeprom_rear_right_label.grid(row=0, column=3, sticky="w", padx=5)
        
        ttk.Label(eeprom_frame, text="Front - Left:").grid(row=1, column=0, sticky="w", padx=5)
        self.eeprom_front_left_label = ttk.Label(eeprom_frame, text="--", foreground="darkgreen", font=("Arial", 9, "bold"))
        self.eeprom_front_left_label.grid(row=1, column=1, sticky="w", padx=5)
        ttk.Label(eeprom_frame, text="Right:").grid(row=1, column=2, sticky="w", padx=5)
        self.eeprom_front_right_label = ttk.Label(eeprom_frame, text="--", foreground="darkgreen", font=("Arial", 9, "bold"))
        self.eeprom_front_right_label.grid(row=1, column=3, sticky="w", padx=5)
        
        ttk.Button(eeprom_frame, text="Refresh EEPROM Values", command=self.view_eeprom).grid(row=2, column=0, columnspan=1, pady=5, padx=5)
        ttk.Button(eeprom_frame, text="Write to EEPROM", command=self.write_to_eeprom).grid(row=2, column=1, columnspan=1, pady=5, padx=5)
        
        # Write Protection Frame
        write_prot_frame = ttk.LabelFrame(scrollable_frame, text="EEPROM Write Protection", padding="10")
        write_prot_frame.pack(fill="x", padx=10, pady=10)
        
        prot_status_frame = ttk.Frame(write_prot_frame)
        prot_status_frame.pack(fill="x", padx=5, pady=5)
        self.write_protection_label = ttk.Label(prot_status_frame, text="Write Protection: LOCKED", foreground="red", font=("Arial", 10, "bold"))
        self.write_protection_label.pack(side="left", padx=5)
        ttk.Button(prot_status_frame, text="Toggle", command=self.toggle_write_protection).pack(side="left", padx=5)
        
        # Operation Mode Frame
        mode_frame = ttk.LabelFrame(scrollable_frame, text="Operation Mode", padding="10")
        mode_frame.pack(fill="x", padx=10, pady=10)
        
        mode_control_frame = ttk.Frame(mode_frame)
        mode_control_frame.pack(fill="x", padx=5, pady=5)
        
        self.mode_label = ttk.Label(mode_control_frame, text="Mode: USB Control", foreground="red", font=("Arial", 10, "bold"))
        self.mode_label.pack(side="left", padx=5)
        
        self.toggle_mode_btn = ttk.Button(mode_control_frame, text="Switch to Normal Operation", command=self.toggle_mode)
        self.toggle_mode_btn.pack(side="left", padx=5)
        
        # Experimental Frame
        exp_frame = ttk.LabelFrame(scrollable_frame, text="Experimental", padding="10")
        exp_frame.pack(fill="x", padx=10, pady=10)
        
        exp_button_frame = ttk.Frame(exp_frame)
        exp_button_frame.pack(fill="x", padx=5, pady=5)
        
        self.sweep_btn = ttk.Button(exp_button_frame, text="Start Sweep & Log", command=self.sweep_and_log)
        self.sweep_btn.pack(side="left", padx=5)
        self.sweep_status_label = ttk.Label(exp_button_frame, text="", foreground="blue")
        self.sweep_status_label.pack(side="left", padx=5)
        
        motor_frame = ttk.LabelFrame(scrollable_frame, text="Motor Control", padding="10")
        motor_frame.pack(fill="x", padx=10, pady=10)
        
        # Enable/Disable
        control_subframe = ttk.Frame(motor_frame)
        control_subframe.pack(fill="x", padx=5, pady=5)
        
        self.enable_btn = ttk.Button(control_subframe, text="Enable Motor", command=self.enable_motor)
        self.enable_btn.pack(side="left", padx=5)
        
        self.disable_btn = ttk.Button(control_subframe, text="Disable Motor", command=self.disable_motor)
        self.disable_btn.pack(side="left", padx=5)
        
        # Motor Position Slider
        slider_frame = ttk.Frame(motor_frame)
        slider_frame.pack(fill="x", padx=5, pady=10)
        
        ttk.Label(slider_frame, text="Left").pack(side="left", padx=5)
        
        self.motor_slider = ttk.Scale(
            slider_frame, 
            from_=0, 
            to=1023, 
            orient="horizontal",
            command=self.on_slider_change
        )
        self.motor_slider.set(512)
        self.motor_slider.pack(side="left", fill="x", expand=True, padx=5)
        # Bind release event to return to center
        self.motor_slider.bind("<ButtonRelease-1>", lambda e: self.root.after(100, lambda: self.motor_slider.set(512)))
        
        ttk.Label(slider_frame, text="Right").pack(side="left", padx=5)
        
        # Motor Value Display
        value_frame = ttk.Frame(motor_frame)
        value_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(value_frame, text="Motor Value:").pack(side="left", padx=5)
        self.motor_value_label = ttk.Label(value_frame, text="512", foreground="blue", font=("Arial", 12, "bold"))
        self.motor_value_label.pack(side="left", padx=5)
        
        # PID Tuning Frame
        pid_frame = ttk.LabelFrame(scrollable_frame, text="PID Tuning", padding="10")
        pid_frame.pack(fill="x", padx=10, pady=10)
        
        # Torque PID Section
        torque_frame = ttk.LabelFrame(pid_frame, text="Torque Loop (Kp, Ki, Kd)", padding="5")
        torque_frame.pack(fill="x", padx=5, pady=5)
        
        torque_input_frame = ttk.Frame(torque_frame)
        torque_input_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(torque_input_frame, text="Kp:").pack(side="left", padx=5)
        ttk.Entry(torque_input_frame, textvariable=self.torque_kp_var, width=10).pack(side="left", padx=2)
        ttk.Label(torque_input_frame, text="Ki:").pack(side="left", padx=5)
        ttk.Entry(torque_input_frame, textvariable=self.torque_ki_var, width=10).pack(side="left", padx=2)
        ttk.Label(torque_input_frame, text="Kd:").pack(side="left", padx=5)
        ttk.Entry(torque_input_frame, textvariable=self.torque_kd_var, width=10).pack(side="left", padx=2)
        
        torque_button_frame = ttk.Frame(torque_frame)
        torque_button_frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(torque_button_frame, text="Send", command=self.send_torque_pid).pack(side="left", padx=3)
        
        # Velocity PID Section
        velocity_frame = ttk.LabelFrame(pid_frame, text="Velocity Loop (Kp, Ki, Kd)", padding="5")
        velocity_frame.pack(fill="x", padx=5, pady=5)
        
        velocity_input_frame = ttk.Frame(velocity_frame)
        velocity_input_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(velocity_input_frame, text="Kp:").pack(side="left", padx=5)
        ttk.Entry(velocity_input_frame, textvariable=self.velocity_kp_var, width=10).pack(side="left", padx=2)
        ttk.Label(velocity_input_frame, text="Ki:").pack(side="left", padx=5)
        ttk.Entry(velocity_input_frame, textvariable=self.velocity_ki_var, width=10).pack(side="left", padx=2)
        ttk.Label(velocity_input_frame, text="Kd:").pack(side="left", padx=5)
        ttk.Entry(velocity_input_frame, textvariable=self.velocity_kd_var, width=10).pack(side="left", padx=2)
        
        velocity_button_frame = ttk.Frame(velocity_frame)
        velocity_button_frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(velocity_button_frame, text="Send", command=self.send_velocity_pid).pack(side="left", padx=3)
        
        # Position PID Section
        position_frame = ttk.LabelFrame(pid_frame, text="Position Loop (Kp, Ki, Kd)", padding="5")
        position_frame.pack(fill="x", padx=5, pady=5)
        
        position_input_frame = ttk.Frame(position_frame)
        position_input_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(position_input_frame, text="Kp:").pack(side="left", padx=5)
        ttk.Entry(position_input_frame, textvariable=self.position_kp_var, width=10).pack(side="left", padx=2)
        ttk.Label(position_input_frame, text="Ki:").pack(side="left", padx=5)
        ttk.Entry(position_input_frame, textvariable=self.position_ki_var, width=10).pack(side="left", padx=2)
        ttk.Label(position_input_frame, text="Kd:").pack(side="left", padx=5)
        ttk.Entry(position_input_frame, textvariable=self.position_kd_var, width=10).pack(side="left", padx=2)
        
        position_button_frame = ttk.Frame(position_frame)
        position_button_frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(position_button_frame, text="Send", command=self.send_position_pid).pack(side="left", padx=3)
        
        # PID EEPROM Controls
        pid_eeprom_frame = ttk.Frame(pid_frame)
        pid_eeprom_frame.pack(fill="x", padx=5, pady=10)
        ttk.Button(pid_eeprom_frame, text="Write PID to EEPROM", command=self.write_pid_eeprom).pack(side="left", padx=3)
        ttk.Button(pid_eeprom_frame, text="Read PID from Device", command=self.read_pid_from_device).pack(side="left", padx=3)
        ttk.Button(pid_eeprom_frame, text="Open Loop Plots", command=self.open_loop_plots).pack(side="left", padx=3)
        
        # Log Frame
        log_frame = ttk.LabelFrame(scrollable_frame, text="Log", padding="10")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Text widget with scrollbar
        log_text_frame = ttk.Frame(log_frame)
        log_text_frame.pack(fill="both", expand=True)
        
        self.log_text = tk.Text(log_text_frame, height=8, width=60, state="disabled")
        self.log_text.pack(side="left", fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(log_text_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).pack(pady=5)
        
    def update_ports(self):
        """Update available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports if ports else ["No ports available"]
        if ports:
            self.port_combo.current(0)
            
    def connect(self):
        """Connect to serial port"""
        if self.connected:
            self.disconnect()
            return
            
        port = self.port_var.get()
        if not port or "No ports" in port:
            messagebox.showerror("Error", "Please select a valid serial port")
            return
            
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.connected = True
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Status: Connected", foreground="green")
            self.log("Connected to " + port)
            
            # Request working limits after connection
            self.root.after(200, self.send_command, "V")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log("Connection failed: " + str(e))
            
    def disconnect(self):
        """Disconnect from serial port"""
        # Close port first to interrupt any blocking read operations
        if self.ser and self.ser.is_open:
            self.ser.close()
        # Then set flags to stop the thread
        self.connected = False
        self.running = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.log("Disconnected")
        
    def read_serial(self):
        """Read data from serial port in background thread"""
        request_timer = 0
        while self.running and self.connected:
            try:
                # Check again to exit quickly if stopped
                if not self.running or not self.connected:
                    break
                    
                if self.ser and self.ser.in_waiting:
                    response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.log("← " + response)
                        self.parse_response(response)
                
                # Periodically request pot values (every 500ms) - but not during sweep
                if not self.sweep_running:
                    request_timer += 10
                    if request_timer >= 500:
                        self.send_command("P")
                        request_timer = 0
            except Exception as e:
                self.log("Read error: " + str(e))
                break
            time.sleep(0.01)
            
    def parse_response(self, response):
        """Parse response from device"""
        parts = response.split(',')
        if len(parts) < 1:
            return
            
        cmd = parts[0]
        
        if cmd == 'P' and len(parts) == 5:
            # Raw pot values: P,rearA,rearB,frontA,frontB
            self.rear_a_label.config(text=parts[1])
            self.rear_b_label.config(text=parts[2])
            self.front_a_label.config(text=parts[3])
            self.front_b_label.config(text=parts[4])
        elif cmd == 'T' and len(parts) == 2:
            # Mode toggle: T,ON or T,OFF
            # ON = normal operation (runFlag = true), OFF = USB control (runFlag = false)
            is_normal_mode = parts[1] == "ON"
            self.normal_operation_mode = is_normal_mode
            
            mode_text = "Normal Operation" if is_normal_mode else "USB Control"
            mode_color = "green" if is_normal_mode else "red"
            button_text = "Switch to USB Control" if is_normal_mode else "Switch to Normal Operation"
            
            self.mode_label.config(text=f"Mode: {mode_text}", foreground=mode_color)
            self.toggle_mode_btn.config(text=button_text)
        elif cmd == 'WT' and len(parts) == 2:
            # Write protection toggle: WT,LOCKED or WT,UNLOCKED
            status = parts[1]
            color = "red" if status == "LOCKED" else "green"
            self.write_protection_locked = (status == "LOCKED")
            self.write_protection_label.config(text=f"Write Protection: {status}", foreground=color)
        elif cmd == 'CAL':
            # Calibration status messages
            if len(parts) >= 2:
                if parts[1] == 'OK':
                    self.calibration_status_label.config(text="Calibration Complete!", foreground="green")
                elif parts[1] == 'START':
                    self.calibration_status_label.config(text="Calibrating...", foreground="orange")
                elif parts[1] == 'LEFT' and len(parts) >= 2:
                    self.calibration_status_label.config(text="Finding left limit...", foreground="orange")
                elif parts[1] == 'RIGHT' and len(parts) >= 2:
                    self.calibration_status_label.config(text="Finding right limit...", foreground="orange")
                elif 'FAIL' in parts[1]:
                    self.calibration_status_label.config(text="Calibration Failed!", foreground="red")
        elif cmd == 'SWEEP':
            # Sweep status messages
            if len(parts) >= 2:
                if parts[1] == 'OK':
                    self.sweep_status_label.config(text="Sweep Complete!", foreground="green")
                    self.sweep_btn.config(text="Start Sweep & Log")
                    self.sweep_running = False
                elif parts[1] == 'START':
                    self.sweep_status_label.config(text="Sweep Started...", foreground="orange")
                elif parts[1] == 'STOPPED':
                    self.sweep_status_label.config(text="Sweep Stopped!", foreground="orange")
                    self.sweep_btn.config(text="Start Sweep & Log")
                    self.sweep_running = False
                elif parts[1] == 'STOPPING':
                    self.sweep_status_label.config(text="Stopping sweep...", foreground="orange")
                elif parts[1] == 'DATA':
                    self.sweep_status_label.config(text="Recording sweep data...", foreground="blue")
                # Ignore individual sweep data points (motorVal,rearA,rearB,frontA,frontB) in log
        elif cmd == 'V' and len(parts) >= 8:
            # View working limits: V,rear,left,right,match,front,left,right,match
            # Display working RAM values in calibration section
            self.rear_leftlimit_label.config(text=parts[2])
            self.rear_rightlimit_label.config(text=parts[3])
            self.front_leftlimit_label.config(text=parts[6])
            self.front_rightlimit_label.config(text=parts[7])
        elif cmd == 'VE' and len(parts) >= 7:
            # View EEPROM values: VE,rear,left,right,front,left,right
            self.eeprom_rear_left_label.config(text=parts[2])
            self.eeprom_rear_right_label.config(text=parts[3])
            self.eeprom_front_left_label.config(text=parts[5])
            self.eeprom_front_right_label.config(text=parts[6])
        
        # PID Parameter Responses
        elif cmd == 'IC' and len(parts) == 4:
            # Torque PID: IC,Kp,Ki,Kd
            self.torque_kp_var.set(float(parts[1]))
            self.torque_ki_var.set(float(parts[2]))
            self.torque_kd_var.set(float(parts[3]))
            self.log("Updated torque PID from device")
        elif cmd == 'IV' and len(parts) == 4:
            # Velocity PID: IV,Kp,Ki,Kd
            self.velocity_kp_var.set(float(parts[1]))
            self.velocity_ki_var.set(float(parts[2]))
            self.velocity_kd_var.set(float(parts[3]))
            self.log("Updated velocity PID from device")
        elif cmd == 'IP' and len(parts) == 4:
            # Position PID: IP,Kp,Ki,Kd
            self.position_kp_var.set(float(parts[1]))
            self.position_ki_var.set(float(parts[2]))
            self.position_kd_var.set(float(parts[3]))
            self.log("Updated position PID from device")
        elif cmd == 'IAT' and len(parts) == 4:
            # All torque PID: IAT,Kp,Ki,Kd
            self.torque_kp_var.set(float(parts[1]))
            self.torque_ki_var.set(float(parts[2]))
            self.torque_kd_var.set(float(parts[3]))
        elif cmd == 'IAV' and len(parts) == 4:
            # All velocity PID: IAV,Kp,Ki,Kd
            self.velocity_kp_var.set(float(parts[1]))
            self.velocity_ki_var.set(float(parts[2]))
            self.velocity_kd_var.set(float(parts[3]))
        elif cmd == 'IAP' and len(parts) == 4:
            # All position PID: IAP,Kp,Ki,Kd
            self.position_kp_var.set(float(parts[1]))
            self.position_ki_var.set(float(parts[2]))
            self.position_kd_var.set(float(parts[3]))
            self.log("Updated all PID parameters from device")
        elif cmd == 'HC' and len(parts) == 2:
            # Torque PID set: HC,OK
            self.log("Torque PID sent to device")
        elif cmd == 'HV' and len(parts) == 2:
            # Velocity PID set: HV,OK
            self.log("Velocity PID sent to device")
        elif cmd == 'HP' and len(parts) == 2:
            # Position PID set: HP,OK
            self.log("Position PID sent to device")
        elif cmd == 'XPID':
            # PID EEPROM write: XPID,OK or XPID,LOCKED
            if len(parts) >= 2:
                if parts[1] == 'OK':
                    self.log("PID parameters saved to EEPROM")
                else:
                    self.log(f"PID write failed: {parts[1]}")
        elif cmd == 'UC' and len(parts) == 2:
            # Torque data streaming toggle: UC,ON or UC,OFF
            status = parts[1]
            self.log(f"Torque data streaming: {status}")
        elif cmd == 'UV' and len(parts) == 2:
            # Velocity data streaming toggle: UV,ON or UV,OFF
            status = parts[1]
            self.log(f"Velocity data streaming: {status}")
        elif cmd == 'UP' and len(parts) == 2:
            # Position data streaming toggle: UP,ON or UP,OFF
            status = parts[1]
            self.log(f"Position data streaming: {status}")
        
        # Plot data streaming: TC,target,actual,timestamp (torque), VC,target,actual,timestamp (velocity), LP,target,actual,timestamp (position)
        elif cmd == 'TC' and len(parts) == 4:
            # Torque loop data: TC,target,actual,timestamp
            try:
                target = float(parts[1])
                actual = float(parts[2])
                timestamp = float(parts[3])
                self.torque_target_data.append(target)
                self.torque_actual_data.append(actual)
                self.torque_time_data.append(timestamp)
            except (ValueError, IndexError):
                pass
        elif cmd == 'VC' and len(parts) == 4:
            # Velocity loop data: VC,target,actual,timestamp
            try:
                target = float(parts[1])
                actual = float(parts[2])
                timestamp = float(parts[3])
                self.velocity_target_data.append(target)
                self.velocity_actual_data.append(actual)
                self.velocity_time_data.append(timestamp)
            except (ValueError, IndexError):
                pass
        elif cmd == 'LP' and len(parts) == 4:
            # Position loop data: LP,target,actual,timestamp
            try:
                target = float(parts[1])
                actual = float(parts[2])
                timestamp = float(parts[3])
                self.position_target_data.append(target)
                self.position_actual_data.append(actual)
                self.position_time_data.append(timestamp)
            except (ValueError, IndexError):
                pass
            
    def send_command(self, cmd):
        """Send command to serial port"""
        if not self.connected or not self.ser:
            messagebox.showerror("Error", "Not connected to serial port")
            return False
            
        try:
            self.ser.write((cmd + "\n").encode())
            self.log("→ " + cmd)
            return True
        except Exception as e:
            messagebox.showerror("Send Error", str(e))
            self.log("Send error: " + str(e))
            return False
            
    def set_rear_left(self):
        """Set rear SAS left position"""
        self.send_command("RL")
        time.sleep(0.05)
        self.send_command("V")
        
    def set_rear_right(self):
        """Set rear SAS right position"""
        self.send_command("RR")
        time.sleep(0.05)
        self.send_command("V")
        
    def set_front_left(self):
        """Set front SAS left position"""
        self.send_command("FL")
        time.sleep(0.05)
        self.send_command("V")
        
    def set_front_right(self):
        """Set front SAS right position"""
        self.send_command("FR")
        time.sleep(0.05)
        self.send_command("V")
        
    def auto_calibrate(self):
        """Start automatic calibration of rear SAS"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        self.calibration_status_label.config(text="Calibrating...", foreground="orange")
        self.send_command("CA")
        
    def sweep_and_log(self):
        """Toggle sweep through full steering range"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        
        if self.sweep_running:
            # Stop sweep
            self.send_command("Z")
            self.sweep_running = False
        else:
            # Start sweep
            self.sweep_status_label.config(text="Sweeping...", foreground="orange")
            self.sweep_btn.config(text="Stop Sweep")
            self.sweep_running = True
            self.send_command("Z")
            
    def view_limits(self):
        """View calibration limit values"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        self.send_command("V")
        
    def enable_motor(self):
        """Enable motor"""
        self.motor_enabled = True
        self.motor_slider.config(state="normal")
        self.send_command("E")
        
    def disable_motor(self):
        """Disable motor"""
        self.motor_enabled = False
        self.motor_slider.config(state="disabled")
        self.send_command("D")
        
    def on_slider_change(self, value):
        """Handle slider change"""
        if not self.motor_enabled:
            return
        motor_value = int(float(value))
        self.motor_value_label.config(text=str(motor_value))
        self.send_command(f"M{motor_value}")
        
    def center_motor(self):
        """Center motor (512)"""
        self.motor_slider.set(512)
        
    def toggle_mode(self):
        """Toggle between USB control and normal operation"""
        self.send_command("T")
        
    def toggle_write_protection(self):
        """Toggle EEPROM write protection"""
        self.send_command("WT")
    
    def write_to_eeprom(self):
        """Write current calibration values to EEPROM"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        
        if self.write_protection_locked:
            messagebox.showerror("Error", "Write protection is LOCKED. Toggle it to write.")
            return
        
        self.send_command("WS")
        self.log("EEPROM write requested")
        
        # Request updated values to show match status
        time.sleep(0.1)
        self.send_command("V")
    
    def view_limits(self):
        """View working calibration limit values"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        self.send_command("V")
    
    def view_eeprom(self):
        """View EEPROM saved values"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        self.send_command("VE")
    
    # PID Tuning Methods
    def send_torque_pid(self):
        """Send torque PID parameters to device"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        kp = self.torque_kp_var.get()
        ki = self.torque_ki_var.get()
        kd = self.torque_kd_var.get()
        cmd = f"HC{kp},{ki},{kd}"
        self.send_command(cmd)
    
    def send_velocity_pid(self):
        """Send velocity PID parameters to device"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        kp = self.velocity_kp_var.get()
        ki = self.velocity_ki_var.get()
        kd = self.velocity_kd_var.get()
        cmd = f"HV{kp},{ki},{kd}"
        self.send_command(cmd)
    
    def send_position_pid(self):
        """Send position PID parameters to device"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        kp = self.position_kp_var.get()
        ki = self.position_ki_var.get()
        kd = self.position_kd_var.get()
        cmd = f"HP{kp},{ki},{kd}"
        self.send_command(cmd)
    
    def read_pid_from_device(self):
        """Read current PID parameters from device"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        self.send_command("IA")
    
    def write_pid_eeprom(self):
        """Write PID parameters to EEPROM"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        
        if self.write_protection_locked:
            messagebox.showerror("Error", "Write protection is LOCKED. Toggle it to write.")
            return
        
        self.send_command("XPID")
        self.log("PID EEPROM write requested")
    
    # Plotting Methods
    def open_loop_plots(self):
        """Open a single window with all three loop plots stacked vertically"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        
        # Check if any are already collecting
        if self.collecting_torque_data or self.collecting_velocity_data or self.collecting_position_data:
            messagebox.showinfo("Info", "Plots are already open")
            return
        
        # Start data collection for all three loops
        self.collecting_torque_data = True
        self.collecting_velocity_data = True
        self.collecting_position_data = True
        # Clear old data buffers
        self.torque_time_data.clear()
        self.torque_target_data.clear()
        self.torque_actual_data.clear()
        self.velocity_time_data.clear()
        self.velocity_target_data.clear()
        self.velocity_actual_data.clear()
        self.position_time_data.clear()
        self.position_target_data.clear()
        self.position_actual_data.clear()
        self.send_command("UC")
        self.send_command("UV")
        self.send_command("UP")
        
        # Create new window
        plot_window = tk.Toplevel(self.root)
        plot_window.title("Control Loop Analysis")
        plot_window.geometry("900x950")
        
        # Create control frame at top for plot settings
        control_frame = ttk.LabelFrame(plot_window, text="Plot Settings", padding="10")
        control_frame.pack(fill="x", padx=10, pady=10)
        
        # Row 1: Sample count
        row1 = ttk.Frame(control_frame)
        row1.pack(fill="x", padx=5, pady=2)
        tk.Label(row1, text="Samples:", font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=10)
        ttk.Entry(row1, textvariable=self.plot_sample_count_var, width=8).pack(side="left", padx=2)
        
        # Row 2: Time intervals
        row2 = ttk.Frame(control_frame)
        row2.pack(fill="x", padx=5, pady=2)
        tk.Label(row2, text="Intervals (ms):", font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=10)
        tk.Label(row2, text="Torque:", font=("TkDefaultFont", 9)).pack(side="left", padx=(10,2))
        ttk.Entry(row2, textvariable=self.torque_interval_var, width=6).pack(side="left", padx=2)
        tk.Label(row2, text="Velocity:", font=("TkDefaultFont", 9)).pack(side="left", padx=(10,2))
        ttk.Entry(row2, textvariable=self.velocity_interval_var, width=6).pack(side="left", padx=2)
        tk.Label(row2, text="Position:", font=("TkDefaultFont", 9)).pack(side="left", padx=(10,2))
        ttk.Entry(row2, textvariable=self.position_interval_var, width=6).pack(side="left", padx=2)
        
        # Create matplotlib figure with 3 subplots stacked vertically
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 10), dpi=100)
        # Add bottom margin and increased vertical spacing to prevent time labels from overlapping
        fig.subplots_adjust(bottom=0.1, hspace=0.55)
        
        # TkAgg canvas
        canvas = FigureCanvasTkAgg(fig, master=plot_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Animation function
        def update_plots(frame):
            # Get sample count (with bounds checking)
            try:
                num_samples = max(10, self.plot_sample_count_var.get())
            except (tk.TclError, ValueError):
                num_samples = 200
            
            # Get time intervals (ms)
            try:
                torque_dt = max(0.001, self.torque_interval_var.get())
                velocity_dt = max(0.001, self.velocity_interval_var.get())
                position_dt = max(0.001, self.position_interval_var.get())
            except (tk.TclError, ValueError):
                torque_dt, velocity_dt, position_dt = 1.0, 2.0, 5.0
            
            def get_last_n(time_data, target_data, actual_data, n):
                """Get the last N samples from each buffer"""
                t_list = list(time_data)
                tar_list = list(target_data)
                act_list = list(actual_data)
                return t_list[-n:], tar_list[-n:], act_list[-n:]
            
            # Torque loop (timestamps in ms)
            ax1.clear()
            ax1.set_axisbelow(True)
            ax1.grid(True, alpha=0.6, linestyle='-', linewidth=0.7, color='gray')
            ax1.set_xlabel("Time (ms)", fontsize=11, fontweight='bold')
            ax1.set_ylabel("Torque Response", fontsize=11, fontweight='bold')
            ax1.set_title(f"Torque Control Loop @ {torque_dt}ms ({num_samples} samples = {num_samples*torque_dt:.0f}ms)", fontsize=12, fontweight='bold', pad=10)
            ax1.xaxis.set_major_locator(plt.MaxNLocator(nbins=6))
            
            torque_t, torque_tar, torque_act = get_last_n(
                self.torque_time_data, self.torque_target_data, self.torque_actual_data, num_samples)
            
            if len(torque_t) > 0:
                ax1.plot(torque_t, torque_tar, 
                        label="Target", linestyle='--', color='blue', linewidth=2.5)
                ax1.plot(torque_t, torque_act, 
                        label="Actual", color='red', linewidth=2.5)
                ax1.legend(loc='upper right', fontsize=9)
                ax1.autoscale(enable=True, axis='y')
            else:
                ax1.set_xlim(0, 1)
                ax1.set_ylim(0, 1)
            
            # Velocity loop (timestamps in ms)
            ax2.clear()
            ax2.set_axisbelow(True)
            ax2.grid(True, alpha=0.6, linestyle='-', linewidth=0.7, color='gray')
            ax2.set_xlabel("Time (ms)", fontsize=11, fontweight='bold')
            ax2.set_ylabel("Velocity Response", fontsize=11, fontweight='bold')
            ax2.set_title(f"Velocity Control Loop @ {velocity_dt}ms ({num_samples} samples = {num_samples*velocity_dt:.0f}ms)", fontsize=12, fontweight='bold', pad=10)
            ax2.xaxis.set_major_locator(plt.MaxNLocator(nbins=6))
            
            velocity_t, velocity_tar, velocity_act = get_last_n(
                self.velocity_time_data, self.velocity_target_data, self.velocity_actual_data, num_samples)
            
            if len(velocity_t) > 0:
                ax2.plot(velocity_t, velocity_tar, 
                        label="Target", linestyle='--', color='blue', linewidth=2.5)
                ax2.plot(velocity_t, velocity_act, 
                        label="Actual", color='green', linewidth=2.5)
                ax2.legend(loc='upper right', fontsize=9)
                ax2.autoscale(enable=True, axis='y')
            else:
                ax2.set_xlim(0, 1)
                ax2.set_ylim(0, 1)
            
            # Position loop (timestamps in ms)
            ax3.clear()
            ax3.set_axisbelow(True)
            ax3.grid(True, alpha=0.6, linestyle='-', linewidth=0.7, color='gray')
            ax3.set_xlabel("Time (ms)", fontsize=11, fontweight='bold')
            ax3.set_ylabel("Position Response", fontsize=11, fontweight='bold')
            ax3.set_title(f"Position Control Loop @ {position_dt}ms ({num_samples} samples = {num_samples*position_dt:.0f}ms)", fontsize=12, fontweight='bold', pad=10)
            ax3.xaxis.set_major_locator(plt.MaxNLocator(nbins=6))
            
            position_t, position_tar, position_act = get_last_n(
                self.position_time_data, self.position_target_data, self.position_actual_data, num_samples)
            
            if len(position_t) > 0:
                ax3.plot(position_t, position_tar, 
                        label="Target", linestyle='--', color='blue', linewidth=2.5)
                ax3.plot(position_t, position_act, 
                        label="Actual", color='purple', linewidth=2.5)
                ax3.legend(loc='upper right', fontsize=9)
                ax3.autoscale(enable=True, axis='y')
            else:
                ax3.set_xlim(0, 1)
                ax3.set_ylim(0, 1)
            
            canvas.draw()
        
        # Start animation
        ani = FuncAnimation(fig, update_plots, interval=100, cache_frame_data=False)
        # Store animation on the window to prevent garbage collection
        plot_window.ani = ani
        
        # Handle window close
        def on_close():
            self.collecting_torque_data = False
            self.collecting_velocity_data = False
            self.collecting_position_data = False
            # Send stop-all command (single command, no toggle ambiguity)
            self.send_command("US")
            plot_window.destroy()
        
        plot_window.protocol("WM_DELETE_WINDOW", on_close)
        
    def log(self, message):
        """Add message to log (thread-safe)"""
        if self.shutting_down:
            return  # Don't try to log during shutdown
        try:
            if self.root.winfo_exists():
                def add_log():
                    if self.log_text.winfo_exists():
                        self.log_text.config(state="normal")
                        self.log_text.insert("end", message + "\n")
                        self.log_text.see("end")
                        self.log_text.config(state="disabled")
                self.root.after(0, add_log)
        except Exception:
            pass  # Silently fail if widget no longer exists
        
    def clear_log(self):
        """Clear log"""
        self.log_text.config(state="normal")
        self.log_text.delete(1.0, "end")
        self.log_text.config(state="disabled")
        
    def on_closing(self):
        """Handle window closing"""
        self.shutting_down = True  # Prevent new callbacks from being scheduled
        # Close any open plot windows
        plt.close('all')
        if self.connected:
            self.disconnect()
        # Wait for read thread to finish (with timeout)
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        self.root.destroy()
        import sys
        sys.exit(0)

if __name__ == "__main__":
    root = tk.Tk()
    gui = RearSteerGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()
