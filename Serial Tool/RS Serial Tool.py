import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

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
        self.normal_operation_mode = False  # False = USB Control, True = Normal Operation
        self.sweep_running = False  # Track sweep state
        self.write_protection_locked = True  # Track write protection state
        self.motor_enabled = False  # Track motor enabled state
        
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
        
        # Write Protection Status
        self.write_protection_label = ttk.Label(eeprom_frame, text="Write Protection: LOCKED", foreground="red", font=("Arial", 10, "bold"))
        self.write_protection_label.grid(row=2, column=0, columnspan=4, pady=5)
        
        ttk.Button(eeprom_frame, text="Refresh EEPROM Values", command=self.view_eeprom).grid(row=3, column=0, columnspan=1, pady=5, padx=5)
        ttk.Button(eeprom_frame, text="Write to EEPROM", command=self.write_to_eeprom).grid(row=3, column=1, columnspan=1, pady=5, padx=5)
        ttk.Button(eeprom_frame, text="Toggle Write Protection", command=self.toggle_write_protection).grid(row=3, column=2, columnspan=2, pady=5, padx=5)
        
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
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.log("Disconnected")
        
    def read_serial(self):
        """Read data from serial port in background thread"""
        request_timer = 0
        while self.running and self.connected:
            try:
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
        
    def log(self, message):
        """Add message to log"""
        self.log_text.config(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")
        
    def clear_log(self):
        """Clear log"""
        self.log_text.config(state="normal")
        self.log_text.delete(1.0, "end")
        self.log_text.config(state="disabled")
        
    def on_closing(self):
        """Handle window closing"""
        if self.connected:
            self.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    gui = RearSteerGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()
