import tkinter as tk
from tkinter import ttk, messagebox
import math
import traceback
import csv
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from motor_utils import get_motor_specs  # Import the function

class RobotArmCalculator:
    def __init__(self, root):
        self.root = root
        self.root.title("6DOF Robotic Arm Torque and Power Calculator")
        self.root.geometry("1200x900")
        
        # Global constants
        self.g = 9.80665  # Gravitational acceleration
        self.PI = math.pi
        
        # Initialize variables
        self.init_variables()
        
        # Store results for table
        self.all_results = {}
        
        # Store motor specs (normal and with SF)
        self.motor_specs_normal = {}
        self.motor_specs_sf = {}
        
        # Create GUI
        self.create_gui()
        
        # Bind events for real-time calculation
        self.bind_events()
        
        # Initial calculation and diagram update
        self.calculate_all()
    
    def init_variables(self):
        """Initialize all StringVar variables for GUI"""
        # Payload variables
        self.payload_mass = tk.StringVar(value="5.0")
        
        # Link density (shared)
        self.link_density = tk.StringVar(value="7850.0")  # Steel density kg/m³
        
        # Motor 6 variables
        self.link6_length = tk.StringVar(value="0.2")
        self.link6_radius = tk.StringVar(value="0.02")
        self.motor6_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m6 = tk.StringVar(value="50")
        self.safety_factor_m6 = tk.StringVar(value="1.5")
        self.motor6_length = tk.StringVar(value="0.1")
        
        # Motor 5 variables
        self.link5_length = tk.StringVar(value="0.3")
        self.link5_radius = tk.StringVar(value="0.025")
        self.motor5_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m5 = tk.StringVar(value="50")
        self.safety_factor_m5 = tk.StringVar(value="1.5")
        self.motor5_length = tk.StringVar(value="0.12")
        
        # Motor 4 variables
        self.link4_length = tk.StringVar(value="0.25")
        self.link4_radius = tk.StringVar(value="0.025")
        self.motor4_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m4 = tk.StringVar(value="50")
        self.safety_factor_m4 = tk.StringVar(value="1.5")
        self.motor4_length = tk.StringVar(value="0.12")
        
        # Motor 3 variables
        self.link3_length = tk.StringVar(value="0.25")
        self.link3_radius = tk.StringVar(value="0.03")
        self.motor3_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m3 = tk.StringVar(value="50")
        self.safety_factor_m3 = tk.StringVar(value="1.5")
        self.motor3_length = tk.StringVar(value="0.15")
        
        # Motor 2 variables
        self.link2_length = tk.StringVar(value="0.3")
        self.link2_radius = tk.StringVar(value="0.035")
        self.motor2_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m2 = tk.StringVar(value="50")
        self.safety_factor_m2 = tk.StringVar(value="1.5")
        
        # Motor 1 variables
        self.motor1_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m1 = tk.StringVar(value="50")
        self.safety_factor_m1 = tk.StringVar(value="1.5")
        
        # Link lengths for diagram
        self.link_lengths = [
            self.link2_length,  # First link (vertical)
            self.link3_length,
            self.link4_length,
            self.link5_length,
            self.link6_length
        ]
    
    def create_gui(self):
        """Create the GUI layout"""
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        input_frame = ttk.Frame(notebook)
        notebook.add(input_frame, text="Inputs")
        
        results_frame = ttk.Frame(notebook)
        notebook.add(results_frame, text="Results")
        
        table_frame = ttk.Frame(notebook)
        notebook.add(table_frame, text="Table")
        
        diagram_frame = ttk.Frame(notebook)
        notebook.add(diagram_frame, text="Diagram")
        
        self.create_input_tab(input_frame)
        self.create_results_tab(results_frame)
        self.create_table_tab(table_frame)
        self.create_diagram_tab(diagram_frame)
    
    def create_input_tab(self, parent):
        """Create input fields tab"""
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
        
        global_frame = ttk.LabelFrame(scrollable_frame, text="Global Parameters", padding=10)
        global_frame.pack(fill="x", pady=5)
        
        ttk.Label(global_frame, text="Payload Mass (kg):").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(global_frame, textvariable=self.payload_mass, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(global_frame, text="Link Density (kg/m³):").grid(row=0, column=2, sticky="w", padx=5)
        ttk.Entry(global_frame, textvariable=self.link_density, width=15).grid(row=0, column=3, padx=5)
        
        m6_frame = ttk.LabelFrame(scrollable_frame, text="Motor 6 Parameters", padding=10)
        m6_frame.pack(fill="x", pady=5)
        self.create_motor_inputs(m6_frame, 6, {
            "Link Length (m)": self.link6_length,
            "Link Radius (m)": self.link6_radius,
            "Motor RPM": self.motor6_rpm,
            "Reduction Ratio": self.reduction_ratio_m6,
            "Safety Factor": self.safety_factor_m6,
            "Motor Length (m)": self.motor6_length
        })
        
        m5_frame = ttk.LabelFrame(scrollable_frame, text="Motor 5 Parameters", padding=10)
        m5_frame.pack(fill="x", pady=5)
        self.create_motor_inputs(m5_frame, 5, {
            "Link Length (m)": self.link5_length,
            "Link Radius (m)": self.link5_radius,
            "Motor RPM": self.motor5_rpm,
            "Reduction Ratio": self.reduction_ratio_m5,
            "Safety Factor": self.safety_factor_m5,
            "Motor Length (m)": self.motor5_length
        })
        
        m4_frame = ttk.LabelFrame(scrollable_frame, text="Motor 4 Parameters", padding=10)
        m4_frame.pack(fill="x", pady=5)
        self.create_motor_inputs(m4_frame, 4, {
            "Link Length (m)": self.link4_length,
            "Link Radius (m)": self.link4_radius,
            "Motor RPM": self.motor4_rpm,
            "Reduction Ratio": self.reduction_ratio_m4,
            "Safety Factor": self.safety_factor_m4,
            "Motor Length (m)": self.motor4_length
        })
        
        m3_frame = ttk.LabelFrame(scrollable_frame, text="Motor 3 Parameters", padding=10)
        m3_frame.pack(fill="x", pady=5)
        self.create_motor_inputs(m3_frame, 3, {
            "Link Length (m)": self.link3_length,
            "Link Radius (m)": self.link3_radius,
            "Motor RPM": self.motor3_rpm,
            "Reduction Ratio": self.reduction_ratio_m3,
            "Safety Factor": self.safety_factor_m3,
            "Motor Length (m)": self.motor3_length
        })
        
        m2_frame = ttk.LabelFrame(scrollable_frame, text="Motor 2 Parameters", padding=10)
        m2_frame.pack(fill="x", pady=5)
        self.create_motor_inputs(m2_frame, 2, {
            "Link Length (m)": self.link2_length,
            "Link Radius (m)": self.link2_radius,
            "Motor RPM": self.motor2_rpm,
            "Reduction Ratio": self.reduction_ratio_m2,
            "Safety Factor": self.safety_factor_m2
        })
        
        m1_frame = ttk.LabelFrame(scrollable_frame, text="Motor 1 Parameters (Base)", padding=10)
        m1_frame.pack(fill="x", pady=5)
        self.create_motor_inputs(m1_frame, 1, {
            "Motor RPM": self.motor1_rpm,
            "Reduction Ratio": self.reduction_ratio_m1,
            "Safety Factor": self.safety_factor_m1
        })
    
    def create_motor_inputs(self, parent, motor_num, params):
        """Create input fields for a motor"""
        row = 0
        col = 0
        for label, var in params.items():
            ttk.Label(parent, text=f"{label}:").grid(row=row, column=col, sticky="w", padx=5, pady=2)
            ttk.Entry(parent, textvariable=var, width=15).grid(row=row, column=col+1, padx=5, pady=2)
            col += 2
            if col >= 6:
                col = 0
                row += 1
    
    def create_results_tab(self, parent):
        """Create results display tab"""
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
        
        self.results_frames = {}
        for motor_num in range(1, 7):
            frame = ttk.LabelFrame(scrollable_frame, text=f"Motor {motor_num} Results", padding=10)
            frame.pack(fill="x", pady=5)
            self.results_frames[motor_num] = frame
            self.create_result_labels(frame, motor_num)
    
    def create_table_tab(self, parent):
        """Create table tab with three tables"""
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill="both", expand=True)
        
        # Torque and Power Results Table
        old_table_frame = ttk.LabelFrame(frame, text="Torque and Power Results", padding=10)
        old_table_frame.pack(fill="x", pady=5)
        
        old_columns = (
            "Motor",
            "Total Torque (N⋅m)",
            "Total Torque with SF (N⋅m)",
            "Torque Before Reduction (N⋅m)",
            "Torque Before Reduction with SF (N⋅m)",
            "Power (W)",
            "Power with SF (W)"
        )
        self.old_tree = ttk.Treeview(old_table_frame, columns=old_columns, show="headings")
        
        for col in old_columns:
            self.old_tree.heading(col, text=col)
            self.old_tree.column(col, width=150, anchor="center")
        
        self.old_tree.pack(fill="x", pady=5)
        
        # Motor Specifications Table (Normal Torque and Power)
        new_table_frame = ttk.LabelFrame(frame, text="Motor Specifications", padding=10)
        new_table_frame.pack(fill="x", pady=5)
        
        new_columns = (
            "Motor",
            "Power Rating (W)",
            "Flange Size (mm)",
            "Voltage Type",
            "Model Name",
            "Company Name",
            "Price ($)",
            "Motor Weight (kg)"
        )
        self.new_tree = ttk.Treeview(new_table_frame, columns=new_columns, show="headings")
        
        for col in new_columns:
            self.new_tree.heading(col, text=col)
            self.new_tree.column(col, width=150, anchor="center")
        
        self.new_tree.pack(fill="x", pady=5)
        
        # Motor Specifications with Safety Factor Table
        sf_table_frame = ttk.LabelFrame(frame, text="Motor Specifications with Safety Factor", padding=10)
        sf_table_frame.pack(fill="x", pady=5)
        
        sf_columns = (
            "Motor",
            "Power Rating (W)",
            "Flange Size (mm)",
            "Voltage Type",
            "Model Name",
            "Company Name",
            "Price ($)",
            "Motor Weight (kg)"
        )
        self.sf_tree = ttk.Treeview(sf_table_frame, columns=sf_columns, show="headings")
        
        for col in sf_columns:
            self.sf_tree.heading(col, text=col)
            self.sf_tree.column(col, width=150, anchor="center")
        
        self.sf_tree.pack(fill="x", pady=5)
    
    def create_diagram_tab(self, parent):
        """Create diagram tab with arm plot"""
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill="both", expand=True)
        
        self.fig, self.ax = plt.subplots(figsize=(5, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.update_diagram()
    
    def create_result_labels(self, parent, motor_num):
        """Create result display labels for a motor"""
        results = [
            "Motor",
            "Power Rating (W)",
            "Flange Size (mm)",
            "Voltage Type",
            "Model Name",
            "Company Name",
            "Price ($)",
            "Motor Weight (kg)",
            "Total Torque (N⋅m)",
            "Total Torque with SF (N⋅m)",
            "Torque Before Reduction (N⋅m)",
            "Torque Before Reduction with SF (N⋅m)",
            "Power (W)",
            "Power with SF (W)"
        ]
        
        for i, result in enumerate(results):
            ttk.Label(parent, text=f"{result}:").grid(row=i//2, column=(i%2)*2, sticky="w", padx=5, pady=2)
            label = ttk.Label(parent, text="N/A", background="white", relief="sunken")
            label.grid(row=i//2, column=(i%2)*2+1, sticky="w", padx=5, pady=2)
            setattr(self, f"result_m{motor_num}_{i}", label)
    
    def bind_events(self):
        """Bind events for real-time calculation"""
        variables = [
            self.payload_mass, self.link_density,
            self.link6_length, self.link6_radius, self.motor6_rpm, self.reduction_ratio_m6, self.safety_factor_m6,
            self.motor6_length,
            self.link5_length, self.link5_radius, self.motor5_rpm, self.reduction_ratio_m5, self.safety_factor_m5,
            self.motor5_length,
            self.link4_length, self.link4_radius, self.motor4_rpm, self.reduction_ratio_m4, self.safety_factor_m4,
            self.motor4_length,
            self.link3_length, self.link3_radius, self.motor3_rpm, self.reduction_ratio_m3, self.safety_factor_m3,
            self.motor3_length,
            self.link2_length, self.link2_radius, self.motor2_rpm, self.reduction_ratio_m2, self.safety_factor_m2,
            self.motor1_rpm, self.reduction_ratio_m1, self.safety_factor_m1
        ]
        
        for var in variables:
            var.trace_add("write", self.on_value_change)
    
    def on_value_change(self, *args):
        """Handle value change event"""
        self.calculate_all()
    
    def get_float_value(self, var, default=0.0):
        """Safely get float value from StringVar"""
        try:
            value = float(var.get())
            return value if value >= 0 else default
        except (ValueError, tk.TclError):
            return default
    
    def get_int_value(self, var, default=0):
        """Safely get int value from StringVar"""
        try:
            value = int(var.get())
            return value if value >= 0 else default
        except (ValueError, tk.TclError):
            return default
    
    def update_diagram(self):
        """Update the arm diagram"""
        try:
            self.ax.clear()
            x = [0.0]
            y = [0.0]
            
            try:
                length = max(self.get_float_value(self.link2_length, 0.1), 0.1)
            except tk.TclError:
                length = 0.1
            x.append(x[-1])
            y.append(y[-1] + length)
            
            for link_var in [self.link3_length, self.link4_length, self.link5_length, self.link6_length]:
                try:
                    length = max(self.get_float_value(link_var, 0.1), 0.1)
                except tk.TclError:
                    length = 0.1
                x.append(x[-1] + length)
                y.append(y[-1])
            
            self.ax.plot(x, y, marker="o", linestyle="-", color="blue", linewidth=2, markersize=6)
            
            for i, (xi, yi) in enumerate(zip(x, y)):
                self.ax.text(xi, yi + 0.07, f"J{i+1}", fontsize=7, ha="center")
            
            total_length = sum(max(self.get_float_value(l, 0.1), 0.1) for l in self.link_lengths)
            first_link_length = max(self.get_float_value(self.link2_length, 0.1), 0.1)
            self.ax.set_xlim(-0.3, total_length + 0.3)
            self.ax.set_ylim(-0.3, first_link_length + 0.3)
            self.ax.set_title("6 DOF Arm", fontsize=9, pad=5)
            self.ax.grid(True, linestyle="--", alpha=0.7)
            self.ax.axis("equal")
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"Error updating diagram: {e}")
    
    def calculate_motor6_normal(self):
        """Calculate Motor 6 torque and power requirements (without SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link6_length = self.get_float_value(self.link6_length)
            link_density = self.get_float_value(self.link_density)
            link6_radius = self.get_float_value(self.link6_radius)
            motor6_rpm = self.get_int_value(self.motor6_rpm)
            reduction_ratio_m6 = self.get_int_value(self.reduction_ratio_m6)
            safety_factor_m6 = self.get_float_value(self.safety_factor_m6)
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            
            torque_due_to_payload = link6_length * payload_weight
            torque_due_to_link6 = (link6_length / 2) * link6_weight
            
            total_torque = torque_due_to_payload + torque_due_to_link6
            
            if reduction_ratio_m6 != 0:
                total_torque_before_reduction = total_torque / reduction_ratio_m6
            else:
                total_torque_before_reduction = 0
            
            if motor6_rpm != 0:
                power = total_torque_before_reduction * motor6_rpm * 1000 / 9550
            else:
                power = 0
            
            self.motor_specs_normal[6] = get_motor_specs(6, total_torque, power)
            
            return {
                'total_torque': total_torque,
                'torque_before_reduction': total_torque_before_reduction,
                'power': power,
                'safety_factor': safety_factor_m6
            }
            
        except Exception as e:
            print(f"Error in Motor 6 normal calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor6_sf(self):
        """Calculate Motor 6 torque and power requirements (with SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link6_length = self.get_float_value(self.link6_length)
            link_density = self.get_float_value(self.link_density)
            link6_radius = self.get_float_value(self.link6_radius)
            motor6_rpm = self.get_int_value(self.motor6_rpm)
            reduction_ratio_m6 = self.get_int_value(self.reduction_ratio_m6)
            safety_factor_m6 = self.get_float_value(self.safety_factor_m6)
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            
            torque_due_to_payload = link6_length * payload_weight
            torque_due_to_link6 = (link6_length / 2) * link6_weight
            
            total_torque = torque_due_to_payload + torque_due_to_link6
            total_torque_sf = total_torque * safety_factor_m6
            
            if reduction_ratio_m6 != 0:
                total_torque_before_reduction_sf = (total_torque / reduction_ratio_m6) * safety_factor_m6
            else:
                total_torque_before_reduction_sf = 0
            
            if motor6_rpm != 0:
                power_sf = (total_torque / reduction_ratio_m6) * motor6_rpm * 1000 / 9550 * safety_factor_m6
            else:
                power_sf = 0
            
            self.motor_specs_sf[6] = get_motor_specs(6, total_torque_sf, power_sf)
            
            return {
                'total_torque_sf': total_torque_sf,
                'torque_before_reduction_sf': total_torque_before_reduction_sf,
                'power_sf': power_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 6 SF calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor5_normal(self):
        """Calculate Motor 5 torque and power requirements (without SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_rpm = self.get_int_value(self.motor5_rpm)
            reduction_ratio_m5 = self.get_int_value(self.reduction_ratio_m5)
            safety_factor_m5 = self.get_float_value(self.safety_factor_m5)
            
            motor6_weight_kg = self.motor_specs_normal.get(6, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            motor6_weight = self.g * motor6_weight_kg
            
            torque_due_to_payload = (link5_length + motor6_length + link6_length) * payload_weight
            torque_due_to_link6 = (link5_length + motor6_length + link6_length/2) * link6_weight
            torque_due_to_motor6 = (link5_length + motor6_length/2) * motor6_weight
            torque_due_to_link5 = (link5_length/2) * link5_weight
            
            total_torque = torque_due_to_payload + torque_due_to_link6 + torque_due_to_motor6 + torque_due_to_link5
            
            if reduction_ratio_m5 != 0:
                total_torque_before_reduction = total_torque / reduction_ratio_m5
            else:
                total_torque_before_reduction = 0
            
            if motor5_rpm != 0:
                power = total_torque_before_reduction * motor5_rpm * 1000 / 9550
            else:
                power = 0
            
            self.motor_specs_normal[5] = get_motor_specs(5, total_torque, power)
            
            return {
                'total_torque': total_torque,
                'torque_before_reduction': total_torque_before_reduction,
                'power': power,
                'safety_factor': safety_factor_m5
            }
            
        except Exception as e:
            print(f"Error in Motor 5 normal calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor5_sf(self):
        """Calculate Motor 5 torque and power requirements (with SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_rpm = self.get_int_value(self.motor5_rpm)
            reduction_ratio_m5 = self.get_int_value(self.reduction_ratio_m5)
            safety_factor_m5 = self.get_float_value(self.safety_factor_m5)
            
            motor6_weight_kg = self.motor_specs_sf.get(6, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            motor6_weight = self.g * motor6_weight_kg
            
            torque_due_to_payload = (link5_length + motor6_length + link6_length) * payload_weight
            torque_due_to_link6 = (link5_length + motor6_length + link6_length/2) * link6_weight
            torque_due_to_motor6 = (link5_length + motor6_length/2) * motor6_weight
            torque_due_to_link5 = (link5_length/2) * link5_weight
            
            total_torque = torque_due_to_payload + torque_due_to_link6 + torque_due_to_motor6 + torque_due_to_link5
            total_torque_sf = total_torque * safety_factor_m5
            
            if reduction_ratio_m5 != 0:
                total_torque_before_reduction_sf = (total_torque / reduction_ratio_m5) * safety_factor_m5
            else:
                total_torque_before_reduction_sf = 0
            
            if motor5_rpm != 0:
                power_sf = (total_torque / reduction_ratio_m5) * motor5_rpm * 1000 / 9550 * safety_factor_m5
            else:
                power_sf = 0
            
            self.motor_specs_sf[5] = get_motor_specs(5, total_torque_sf, power_sf)
            
            return {
                'total_torque_sf': total_torque_sf,
                'torque_before_reduction_sf': total_torque_before_reduction_sf,
                'power_sf': power_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 5 SF calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor4_normal(self):
        """Calculate Motor 4 torque and power requirements (without SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            link4_length = self.get_float_value(self.link4_length)
            link4_radius = self.get_float_value(self.link4_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_length = self.get_float_value(self.motor5_length)
            motor4_rpm = self.get_int_value(self.motor4_rpm)
            reduction_ratio_m4 = self.get_int_value(self.reduction_ratio_m4)
            safety_factor_m4 = self.get_float_value(self.safety_factor_m4)
            
            motor6_weight_kg = self.motor_specs_normal.get(6, {'motor_weight': 0.0})['motor_weight']
            motor5_weight_kg = self.motor_specs_normal.get(5, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            link4_weight = self.g * link_density * self.PI * link4_radius**2 * link4_length
            motor6_weight = self.g * motor6_weight_kg
            motor5_weight = self.g * motor5_weight_kg
            
            torque_payload = (link4_length + link5_length + link6_length + motor5_length + motor6_length) * payload_weight
            torque_link6 = (link4_length + link5_length + motor5_length + link6_length/2) * link6_weight
            torque_link5 = (link4_length + link5_length/2) * link5_weight
            torque_link4 = (link4_length/2) * link4_weight
            torque_m6 = (link4_length + link5_length + motor5_length + motor6_length/2) * motor6_weight
            torque_m5 = (link4_length + motor5_length/2) * motor5_weight
            
            total_torque = torque_payload + torque_link6 + torque_link5 + torque_link4 + torque_m6 + torque_m5
            
            if reduction_ratio_m4 != 0:
                total_torque_before_reduction = total_torque / reduction_ratio_m4
            else:
                total_torque_before_reduction = 0
            
            if motor4_rpm != 0:
                power = total_torque_before_reduction * motor4_rpm * 1000 / 9550
            else:
                power = 0
            
            self.motor_specs_normal[4] = get_motor_specs(4, total_torque, power)
            
            return {
                'total_torque': total_torque,
                'torque_before_reduction': total_torque_before_reduction,
                'power': power,
                'safety_factor': safety_factor_m4
            }
            
        except Exception as e:
            print(f"Error in Motor 4 normal calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor4_sf(self):
        """Calculate Motor 4 torque and power requirements (with SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            link4_length = self.get_float_value(self.link4_length)
            link4_radius = self.get_float_value(self.link4_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_length = self.get_float_value(self.motor5_length)
            motor4_rpm = self.get_int_value(self.motor4_rpm)
            reduction_ratio_m4 = self.get_int_value(self.reduction_ratio_m4)
            safety_factor_m4 = self.get_float_value(self.safety_factor_m4)
            
            motor6_weight_kg = self.motor_specs_sf.get(6, {'motor_weight': 0.0})['motor_weight']
            motor5_weight_kg = self.motor_specs_sf.get(5, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            link4_weight = self.g * link_density * self.PI * link4_radius**2 * link4_length
            motor6_weight = self.g * motor6_weight_kg
            motor5_weight = self.g * motor5_weight_kg
            
            torque_payload = (link4_length + link5_length + link6_length + motor5_length + motor6_length) * payload_weight
            torque_link6 = (link4_length + link5_length + motor5_length + link6_length/2) * link6_weight
            torque_link5 = (link4_length + link5_length/2) * link5_weight
            torque_link4 = (link4_length/2) * link4_weight
            torque_m6 = (link4_length + link5_length + motor5_length + motor6_length/2) * motor6_weight
            torque_m5 = (link4_length + motor5_length/2) * motor5_weight
            
            total_torque = torque_payload + torque_link6 + torque_link5 + torque_link4 + torque_m6 + torque_m5
            total_torque_sf = total_torque * safety_factor_m4
            
            if reduction_ratio_m4 != 0:
                total_torque_before_reduction_sf = (total_torque / reduction_ratio_m4) * safety_factor_m4
            else:
                total_torque_before_reduction_sf = 0
            
            if motor4_rpm != 0:
                power_sf = (total_torque / reduction_ratio_m4) * motor4_rpm * 1000 / 9550 * safety_factor_m4
            else:
                power_sf = 0
            
            self.motor_specs_sf[4] = get_motor_specs(4, total_torque_sf, power_sf)
            
            return {
                'total_torque_sf': total_torque_sf,
                'torque_before_reduction_sf': total_torque_before_reduction_sf,
                'power_sf': power_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 4 SF calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor3_normal(self):
        """Calculate Motor 3 torque and power requirements (without SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            link4_length = self.get_float_value(self.link4_length)
            link4_radius = self.get_float_value(self.link4_radius)
            link3_length = self.get_float_value(self.link3_length)
            link3_radius = self.get_float_value(self.link3_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_length = self.get_float_value(self.motor5_length)
            motor4_length = self.get_float_value(self.motor4_length)
            motor3_rpm = self.get_int_value(self.motor3_rpm)
            reduction_ratio_m3 = self.get_int_value(self.reduction_ratio_m3)
            safety_factor_m3 = self.get_float_value(self.safety_factor_m3)
            
            motor6_weight_kg = self.motor_specs_normal.get(6, {'motor_weight': 0.0})['motor_weight']
            motor5_weight_kg = self.motor_specs_normal.get(5, {'motor_weight': 0.0})['motor_weight']
            motor4_weight_kg = self.motor_specs_normal.get(4, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            link4_weight = self.g * link_density * self.PI * link4_radius**2 * link4_length
            link3_weight = self.g * link_density * self.PI * link3_radius**2 * link3_length
            motor6_weight = self.g * motor6_weight_kg
            motor5_weight = self.g * motor5_weight_kg
            motor4_weight = self.g * motor4_weight_kg
            
            torque_payload = (link3_length + link4_length + link5_length + link6_length + motor4_length + motor5_length + motor6_length) * payload_weight
            torque_link6 = (link3_length + link4_length + link5_length + motor4_length + motor5_length + link6_length/2) * link6_weight
            torque_link5 = (link3_length + link4_length + motor4_length + link5_length/2) * link5_weight
            torque_link4 = (link3_length + motor4_length + link4_length/2) * link4_weight
            torque_link3 = (link3_length/2) * link3_weight
            torque_m6 = (link3_length + link4_length + link5_length + motor4_length + motor5_length + motor6_length/2) * motor6_weight
            torque_m5 = (link3_length + link4_length + motor4_length + motor5_length/2) * motor5_weight
            torque_m4 = (link3_length + motor4_length/2) * motor4_weight
            
            total_torque = torque_payload + torque_link6 + torque_link5 + torque_link4 + torque_link3 + torque_m6 + torque_m5 + torque_m4
            
            if reduction_ratio_m3 != 0:
                total_torque_before_reduction = total_torque / reduction_ratio_m3
            else:
                total_torque_before_reduction = 0
            
            if motor3_rpm != 0:
                power = total_torque_before_reduction * motor3_rpm * 1000 / 9550
            else:
                power = 0
            
            self.motor_specs_normal[3] = get_motor_specs(3, total_torque, power)
            
            return {
                'total_torque': total_torque,
                'torque_before_reduction': total_torque_before_reduction,
                'power': power,
                'safety_factor': safety_factor_m3
            }
            
        except Exception as e:
            print(f"Error in Motor 3 normal calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor3_sf(self):
        """Calculate Motor 3 torque and power requirements (with SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            link4_length = self.get_float_value(self.link4_length)
            link4_radius = self.get_float_value(self.link4_radius)
            link3_length = self.get_float_value(self.link3_length)
            link3_radius = self.get_float_value(self.link3_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_length = self.get_float_value(self.motor5_length)
            motor4_length = self.get_float_value(self.motor4_length)
            motor3_rpm = self.get_int_value(self.motor3_rpm)
            reduction_ratio_m3 = self.get_int_value(self.reduction_ratio_m3)
            safety_factor_m3 = self.get_float_value(self.safety_factor_m3)
            
            motor6_weight_kg = self.motor_specs_sf.get(6, {'motor_weight': 0.0})['motor_weight']
            motor5_weight_kg = self.motor_specs_sf.get(5, {'motor_weight': 0.0})['motor_weight']
            motor4_weight_kg = self.motor_specs_sf.get(4, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            link4_weight = self.g * link_density * self.PI * link4_radius**2 * link4_length
            link3_weight = self.g * link_density * self.PI * link3_radius**2 * link3_length
            motor6_weight = self.g * motor6_weight_kg
            motor5_weight = self.g * motor5_weight_kg
            motor4_weight = self.g * motor4_weight_kg
            
            torque_payload = (link3_length + link4_length + link5_length + link6_length + motor4_length + motor5_length + motor6_length) * payload_weight
            torque_link6 = (link3_length + link4_length + link5_length + motor4_length + motor5_length + link6_length/2) * link6_weight
            torque_link5 = (link3_length + link4_length + motor4_length + link5_length/2) * link5_weight
            torque_link4 = (link3_length + motor4_length + link4_length/2) * link4_weight
            torque_link3 = (link3_length/2) * link3_weight
            torque_m6 = (link3_length + link4_length + link5_length + motor4_length + motor5_length + motor6_length/2) * motor6_weight
            torque_m5 = (link3_length + link4_length + motor4_length + motor5_length/2) * motor5_weight
            torque_m4 = (link3_length + motor4_length/2) * motor4_weight
            
            total_torque = torque_payload + torque_link6 + torque_link5 + torque_link4 + torque_link3 + torque_m6 + torque_m5 + torque_m4
            total_torque_sf = total_torque * safety_factor_m3
            
            if reduction_ratio_m3 != 0:
                total_torque_before_reduction_sf = (total_torque / reduction_ratio_m3) * safety_factor_m3
            else:
                total_torque_before_reduction_sf = 0
            
            if motor3_rpm != 0:
                power_sf = (total_torque / reduction_ratio_m3) * motor3_rpm * 1000 / 9550 * safety_factor_m3
            else:
                power_sf = 0
            
            self.motor_specs_sf[3] = get_motor_specs(3, total_torque_sf, power_sf)
            
            return {
                'total_torque_sf': total_torque_sf,
                'torque_before_reduction_sf': total_torque_before_reduction_sf,
                'power_sf': power_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 3 SF calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor2_normal(self):
        """Calculate Motor 2 torque and power requirements (without SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            link4_length = self.get_float_value(self.link4_length)
            link4_radius = self.get_float_value(self.link4_radius)
            link3_length = self.get_float_value(self.link3_length)
            link3_radius = self.get_float_value(self.link3_radius)
            link2_length = self.get_float_value(self.link2_length)
            link2_radius = self.get_float_value(self.link2_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_length = self.get_float_value(self.motor5_length)
            motor4_length = self.get_float_value(self.motor4_length)
            motor3_length = self.get_float_value(self.motor3_length)
            motor2_rpm = self.get_int_value(self.motor2_rpm)
            reduction_ratio_m2 = self.get_int_value(self.reduction_ratio_m2)
            safety_factor_m2 = self.get_float_value(self.safety_factor_m2)
            
            motor6_weight_kg = self.motor_specs_normal.get(6, {'motor_weight': 0.0})['motor_weight']
            motor5_weight_kg = self.motor_specs_normal.get(5, {'motor_weight': 0.0})['motor_weight']
            motor4_weight_kg = self.motor_specs_normal.get(4, {'motor_weight': 0.0})['motor_weight']
            motor3_weight_kg = self.motor_specs_normal.get(3, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            link4_weight = self.g * link_density * self.PI * link4_radius**2 * link4_length
            link3_weight = self.g * link_density * self.PI * link3_radius**2 * link3_length
            link2_weight = self.g * link_density * self.PI * link2_radius**2 * link2_length
            motor6_weight = self.g * motor6_weight_kg
            motor5_weight = self.g * motor5_weight_kg
            motor4_weight = self.g * motor4_weight_kg
            motor3_weight = self.g * motor3_weight_kg
            
            torque_payload = (link2_length + link3_length + link4_length + link5_length + link6_length + motor3_length + motor4_length + motor5_length + motor6_length) * payload_weight
            torque_link6 = (link2_length + link3_length + link4_length + link5_length + motor3_length + motor4_length + motor5_length + link6_length/2) * link6_weight
            torque_link5 = (link2_length + link3_length + link4_length + motor3_length + motor4_length + link5_length/2) * link5_weight
            torque_link4 = (link2_length + link3_length + motor3_length + link4_length/2) * link4_weight
            torque_link3 = (link2_length + motor3_length + link3_length/2) * link3_weight
            torque_link2 = (link2_length/2) * link2_weight
            torque_m6 = (link2_length + link3_length + link4_length + link5_length + motor3_length + motor4_length + motor5_length + motor6_length/2) * motor6_weight
            torque_m5 = (link2_length + link3_length + link4_length + motor3_length + motor4_length + motor5_length/2) * motor5_weight
            torque_m4 = (link2_length + link3_length + motor3_length + motor4_length/2) * motor4_weight
            torque_m3 = (link2_length + motor3_length/2) * motor3_weight
            
            total_torque = torque_payload + torque_link6 + torque_link5 + torque_link4 + torque_link3 + torque_link2 + torque_m6 + torque_m5 + torque_m4 + torque_m3
            
            if reduction_ratio_m2 != 0:
                total_torque_before_reduction = total_torque / reduction_ratio_m2
            else:
                total_torque_before_reduction = 0
            
            if motor2_rpm != 0:
                power = total_torque_before_reduction * motor2_rpm * 1000 / 9550
            else:
                power = 0
            
            self.motor_specs_normal[2] = get_motor_specs(2, total_torque, power)
            
            return {
                'total_torque': total_torque,
                'torque_before_reduction': total_torque_before_reduction,
                'power': power,
                'safety_factor': safety_factor_m2
            }
            
        except Exception as e:
            print(f"Error in Motor 2 normal calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor2_sf(self):
        """Calculate Motor 2 torque and power requirements (with SF)"""
        try:
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            link6_length = self.get_float_value(self.link6_length)
            link6_radius = self.get_float_value(self.link6_radius)
            link5_length = self.get_float_value(self.link5_length)
            link5_radius = self.get_float_value(self.link5_radius)
            link4_length = self.get_float_value(self.link4_length)
            link4_radius = self.get_float_value(self.link4_radius)
            link3_length = self.get_float_value(self.link3_length)
            link3_radius = self.get_float_value(self.link3_radius)
            link2_length = self.get_float_value(self.link2_length)
            link2_radius = self.get_float_value(self.link2_radius)
            motor6_length = self.get_float_value(self.motor6_length)
            motor5_length = self.get_float_value(self.motor5_length)
            motor4_length = self.get_float_value(self.motor4_length)
            motor3_length = self.get_float_value(self.motor3_length)
            motor2_rpm = self.get_int_value(self.motor2_rpm)
            reduction_ratio_m2 = self.get_int_value(self.reduction_ratio_m2)
            safety_factor_m2 = self.get_float_value(self.safety_factor_m2)
            
            motor6_weight_kg = self.motor_specs_sf.get(6, {'motor_weight': 0.0})['motor_weight']
            motor5_weight_kg = self.motor_specs_sf.get(5, {'motor_weight': 0.0})['motor_weight']
            motor4_weight_kg = self.motor_specs_sf.get(4, {'motor_weight': 0.0})['motor_weight']
            motor3_weight_kg = self.motor_specs_sf.get(3, {'motor_weight': 0.0})['motor_weight']
            
            payload_weight = self.g * payload_mass
            link6_weight = self.g * link_density * self.PI * link6_radius**2 * link6_length
            link5_weight = self.g * link_density * self.PI * link5_radius**2 * link5_length
            link4_weight = self.g * link_density * self.PI * link4_radius**2 * link4_length
            link3_weight = self.g * link_density * self.PI * link3_radius**2 * link3_length
            link2_weight = self.g * link_density * self.PI * link2_radius**2 * link2_length
            motor6_weight = self.g * motor6_weight_kg
            motor5_weight = self.g * motor5_weight_kg
            motor4_weight = self.g * motor4_weight_kg
            motor3_weight = self.g * motor3_weight_kg
            
            torque_payload = (link2_length + link3_length + link4_length + link5_length + link6_length + motor3_length + motor4_length + motor5_length + motor6_length) * payload_weight
            torque_link6 = (link2_length + link3_length + link4_length + link5_length + motor3_length + motor4_length + motor5_length + link6_length/2) * link6_weight
            torque_link5 = (link2_length + link3_length + link4_length + motor3_length + motor4_length + link5_length/2) * link5_weight
            torque_link4 = (link2_length + link3_length + motor3_length + link4_length/2) * link4_weight
            torque_link3 = (link2_length + motor3_length + link3_length/2) * link3_weight
            torque_link2 = (link2_length/2) * link2_weight
            torque_m6 = (link2_length + link3_length + link4_length + link5_length + motor3_length + motor4_length + motor5_length + motor6_length/2) * motor6_weight
            torque_m5 = (link2_length + link3_length + link4_length + motor3_length + motor4_length + motor5_length/2) * motor5_weight
            torque_m4 = (link2_length + link3_length + motor3_length + motor4_length/2) * motor4_weight
            torque_m3 = (link2_length + motor3_length/2) * motor3_weight
            
            total_torque = torque_payload + torque_link6 + torque_link5 + torque_link4 + torque_link3 + torque_link2 + torque_m6 + torque_m5 + torque_m4 + torque_m3
            total_torque_sf = total_torque * safety_factor_m2
            
            if reduction_ratio_m2 != 0:
                total_torque_before_reduction_sf = (total_torque / reduction_ratio_m2) * safety_factor_m2
            else:
                total_torque_before_reduction_sf = 0
            
            if motor2_rpm != 0:
                power_sf = (total_torque / reduction_ratio_m2) * motor2_rpm * 1000 / 9550 * safety_factor_m2
            else:
                power_sf = 0
            
            self.motor_specs_sf[2] = get_motor_specs(2, total_torque_sf, power_sf)
            
            return {
                'total_torque_sf': total_torque_sf,
                'torque_before_reduction_sf': total_torque_before_reduction_sf,
                'power_sf': power_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 2 SF calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor1_normal(self):
        """Calculate Motor 1 torque and power requirements (without SF)"""
        try:
            motor2_results = self.calculate_motor2_normal()
            motor1_rpm = self.get_int_value(self.motor1_rpm)
            reduction_ratio_m1 = self.get_int_value(self.reduction_ratio_m1)
            safety_factor_m1 = self.get_float_value(self.safety_factor_m1)
            
            total_torque = motor2_results['total_torque']
            
            if reduction_ratio_m1 != 0:
                total_torque_before_reduction = total_torque / reduction_ratio_m1
            else:
                total_torque_before_reduction = 0
            
            if motor1_rpm != 0:
                power = total_torque_before_reduction * motor1_rpm * 1000 / 9550
            else:
                power = 0
            
            self.motor_specs_normal[1] = get_motor_specs(1, total_torque, power)
            
            return {
                'total_torque': total_torque,
                'torque_before_reduction': total_torque_before_reduction,
                'power': power,
                'safety_factor': safety_factor_m1
            }
            
        except Exception as e:
            print(f"Error in Motor 1 normal calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor1_sf(self):
        """Calculate Motor 1 torque and power requirements (with SF)"""
        try:
            motor2_results = self.calculate_motor2_sf()
            motor1_rpm = self.get_int_value(self.motor1_rpm)
            reduction_ratio_m1 = self.get_int_value(self.reduction_ratio_m1)
            safety_factor_m1 = self.get_float_value(self.safety_factor_m1)
            
            total_torque_sf = motor2_results['total_torque_sf']
            
            if reduction_ratio_m1 != 0:
                total_torque_before_reduction_sf = total_torque_sf / reduction_ratio_m1
            else:
                total_torque_before_reduction_sf = 0
            
            if motor1_rpm != 0:
                power_sf = (total_torque_sf / reduction_ratio_m1) * motor1_rpm * 1000 / 9550
            else:
                power_sf = 0
            
            self.motor_specs_sf[1] = get_motor_specs(1, total_torque_sf, power_sf)
            
            return {
                'total_torque_sf': total_torque_sf,
                'torque_before_reduction_sf': total_torque_before_reduction_sf,
                'power_sf': power_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 1 SF calculation: {e}")
            return self.get_zero_results()
    
    def get_zero_results(self):
        """Return zero results in case of calculation error"""
        return {
            'total_torque': 0.0,
            'total_torque_sf': 0.0,
            'torque_before_reduction': 0.0,
            'torque_before_reduction_sf': 0.0,
            'power': 0.0,
            'power_sf': 0.0,
            'safety_factor': 1.0
        }
    
    def update_results_display(self, motor_num, results_normal, results_sf):
        """Update the results display for a specific motor"""
        try:
            specs = self.motor_specs_normal.get(motor_num, {
                'motor': f"Motor {motor_num}",
                'power_rating': 0.0,
                'flange_size': 0.0,
                'voltage_type': 'N/A',
                'model_name': 'N/A',
                'company_name': 'N/A',
                'price': 0.0,
                'motor_weight': 0.0
            })
            values = [
                specs['motor'],
                specs['power_rating'],
                specs['flange_size'],
                specs['voltage_type'],
                specs['model_name'],
                specs['company_name'],
                specs['price'],
                specs['motor_weight'],
                results_normal['total_torque'],
                results_sf['total_torque_sf'],
                results_normal['torque_before_reduction'],
                results_sf['torque_before_reduction_sf'],
                results_normal['power'],
                results_sf['power_sf']
            ]
            
            for i, value in enumerate(values):
                label = getattr(self, f"result_m{motor_num}_{i}")
                if isinstance(value, float):
                    label.config(text=f"{value:.3f}")
                else:
                    label.config(text=str(value))
                
            self.all_results[motor_num] = values
                
        except Exception as e:
            print(f"Error updating display for Motor {motor_num}: {e}")
    
    def update_table_display(self):
        """Update the table displays with results"""
        try:
            # Update Torque and Power Results Table
            for item in self.old_tree.get_children():
                self.old_tree.delete(item)
            
            for motor_num in range(1, 7):
                values = self.all_results.get(motor_num, [f"Motor {motor_num}"] + [0.0] * 13)[8:14]
                self.old_tree.insert("", "end", values=(
                    f"Motor {motor_num}",
                    f"{values[0]:.3f}",
                    f"{values[1]:.3f}",
                    f"{values[2]:.3f}",
                    f"{values[3]:.3f}",
                    f"{values[4]:.3f}",
                    f"{values[5]:.3f}"
                ))
            
            # Update Motor Specifications Table (Normal Torque and Power)
            for item in self.new_tree.get_children():
                self.new_tree.delete(item)
            
            for motor_num in range(1, 7):
                specs = self.motor_specs_normal.get(motor_num, {
                    'motor': f"Motor {motor_num}",
                    'power_rating': 0.0,
                    'flange_size': 0.0,
                    'voltage_type': 'N/A',
                    'model_name': 'N/A',
                    'company_name': 'N/A',
                    'price': 0.0,
                    'motor_weight': 0.0
                })
                values = [
                    specs['motor'],
                    specs['power_rating'],
                    specs['flange_size'],
                    specs['voltage_type'],
                    specs['model_name'],
                    specs['company_name'],
                    specs['price'],
                    specs['motor_weight']
                ]
                self.new_tree.insert("", "end", values=tuple([str(v) for v in values]))
            
            # Update Motor Specifications with Safety Factor Table
            for item in self.sf_tree.get_children():
                self.sf_tree.delete(item)
            
            for motor_num in range(1, 7):
                specs = self.motor_specs_sf.get(motor_num, {
                    'motor': f"Motor {motor_num}",
                    'power_rating': 0.0,
                    'flange_size': 0.0,
                    'voltage_type': 'N/A',
                    'model_name': 'N/A',
                    'company_name': 'N/A',
                    'price': 0.0,
                    'motor_weight': 0.0
                })
                values = [
                    specs['motor'],
                    specs['power_rating'],
                    specs['flange_size'],
                    specs['voltage_type'],
                    specs['model_name'],
                    specs['company_name'],
                    specs['price'],
                    specs['motor_weight']
                ]
                self.sf_tree.insert("", "end", values=tuple([f"{v:.3f}" if isinstance(v, float) else str(v) for v in values]))
                
        except Exception as e:
            print(f"Error updating table display: {e}")
    
    def export_to_csv(self):
        """Export all tables' data to a single CSV file"""
        try:
            old_headers = [
                "Motor",
                "Total Torque (N⋅m)",
                "Total Torque with SF (N⋅m)",
                "Torque Before Reduction (N⋅m)",
                "Torque Before Reduction with SF (N⋅m)",
                "Power (W)",
                "Power with SF (W)"
            ]
            
            new_headers = [
                "Motor",
                "Power Rating (W)",
                "Flange Size (mm)",
                "Voltage Type",
                "Model Name",
                "Company Name",
                "Price ($)",
                "Motor Weight (kg)"
            ]
            
            sf_headers = [
                "Motor",
                "Power Rating (W)",
                "Flange Size (mm)",
                "Voltage Type",
                "Model Name",
                "Company Name",
                "Price ($)",
                "Motor Weight (kg)"
            ]
            
            data = []
            # Add Torque and Power Results Table
            data.append(["Torque and Power Results"])
            data.append(old_headers)
            for motor_num in range(1, 7):
                values = self.all_results.get(motor_num, [f"Motor {motor_num}"] + [0.0] * 13)[8:14]
                data.append([
                    f"Motor {motor_num}",
                    f"{values[0]:.3f}",
                    f"{values[1]:.3f}",
                    f"{values[2]:.3f}",
                    f"{values[3]:.3f}",
                    f"{values[4]:.3f}",
                    f"{values[5]:.3f}"
                ])
            
            # Add separator
            data.append([])
            
            # Add Motor Specifications Table (Normal Torque and Power)
            data.append(["Motor Specifications"])
            data.append(new_headers)
            for motor_num in range(1, 7):
                specs = self.motor_specs_normal.get(motor_num, {
                    'motor': f"Motor {motor_num}",
                    'power_rating': 0.0,
                    'flange_size': 0.0,
                    'voltage_type': 'N/A',
                    'model_name': 'N/A',
                    'company_name': 'N/A',
                    'price': 0.0,
                    'motor_weight': 0.0
                })
                values = [
                    specs['motor'],
                    specs['power_rating'],
                    specs['flange_size'],
                    specs['voltage_type'],
                    specs['model_name'],
                    specs['company_name'],
                    specs['price'],
                    specs['motor_weight']
                ]
                data.append([str(v) for v in values])
            
            # Add separator
            data.append([])
            
            # Add Motor Specifications with Safety Factor Table
            data.append(["Motor Specifications with Safety Factor"])
            data.append(sf_headers)
            for motor_num in range(1, 7):
                specs = self.motor_specs_sf.get(motor_num, {
                    'motor': f"Motor {motor_num}",
                    'power_rating': 0.0,
                    'flange_size': 0.0,
                    'voltage_type': 'N/A',
                    'model_name': 'N/A',
                    'company_name': 'N/A',
                    'price': 0.0,
                    'motor_weight': 0.0
                })
                values = [
                    specs['motor'],
                    specs['power_rating'],
                    specs['flange_size'],
                    specs['voltage_type'],
                    specs['model_name'],
                    specs['company_name'],
                    specs['price'],
                    specs['motor_weight']
                ]
                data.append([f"{v:.3f}" if isinstance(v, float) else str(v) for v in values])
            
            with open("robot_arm_results.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(data)
            
            messagebox.showinfo("Success", "Results exported to robot_arm_results.csv")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to export CSV: {str(e)}")
    
    def calculate_all(self):
        """Calculate torque and power for all motors and update diagram"""
        try:
            # First pass: Calculate normal torque and power
            motor_normal_functions = [
                (6, self.calculate_motor6_normal),
                (5, self.calculate_motor5_normal),
                (4, self.calculate_motor4_normal),
                (3, self.calculate_motor3_normal),
                (2, self.calculate_motor2_normal),
                (1, self.calculate_motor1_normal)
            ]
            
            self.motor_specs_normal = {}  # Reset normal motor specs
            normal_results = {}
            for motor_num, calc_function in motor_normal_functions:
                results = calc_function()
                normal_results[motor_num] = results
            
            # Second pass: Calculate SF-adjusted torque and power
            motor_sf_functions = [
                (6, self.calculate_motor6_sf),
                (5, self.calculate_motor5_sf),
                (4, self.calculate_motor4_sf),
                (3, self.calculate_motor3_sf),
                (2, self.calculate_motor2_sf),
                (1, self.calculate_motor1_sf)
            ]
            
            self.motor_specs_sf = {}  # Reset SF motor specs
            sf_results = {}
            for motor_num, calc_function in motor_sf_functions:
                results = calc_function()
                sf_results[motor_num] = results
            
            # Update displays
            for motor_num in range(1, 7):
                self.update_results_display(motor_num, normal_results[motor_num], sf_results[motor_num])
            
            self.update_table_display()
            self.update_diagram()
                
        except Exception as e:
            print(f"Error in calculate_all: {e}")
            traceback.print_exc()

def main():
    """Main function to run the application"""
    try:
        root = tk.Tk()
        app = RobotArmCalculator(root)
        
        root.minsize(1000, 700)
        
        root.update_idletasks()
        x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
        y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
        root.geometry(f"+{x}+{y}")
        
        menubar = tk.Menu(root)
        root.config(menu=menubar)
        
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Reset to Defaults", command=lambda: reset_to_defaults(app))
        file_menu.add_command(label="Export to CSV", command=app.export_to_csv)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=root.quit)
        
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=lambda: show_about_dialog(root))
        
        root.mainloop()
        
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {str(e)}")
        traceback.print_exc()

def reset_to_defaults(app):
    """Reset all values to defaults"""
    try:
        app.payload_mass.set("5.0")
        app.link_density.set("7850.0")
        app.link6_length.set("0.2")
        app.link6_radius.set("0.02")
        app.motor6_rpm.set("3000")
        app.reduction_ratio_m6.set("50")
        app.safety_factor_m6.set("1.5")
        app.motor6_length.set("0.1")
        app.link5_length.set("0.3")
        app.link5_radius.set("0.025")
        app.motor5_rpm.set("3000")
        app.reduction_ratio_m5.set("50")
        app.safety_factor_m5.set("1.5")
        app.motor5_length.set("0.12")
        app.link4_length.set("0.25")
        app.link4_radius.set("0.025")
        app.motor4_rpm.set("3000")
        app.reduction_ratio_m4.set("50")
        app.safety_factor_m4.set("1.5")
        app.motor4_length.set("0.12")
        app.link3_length.set("0.25")
        app.link3_radius.set("0.03")
        app.motor3_rpm.set("3000")
        app.reduction_ratio_m3.set("50")
        app.safety_factor_m3.set("1.5")
        app.motor3_length.set("0.15")
        app.link2_length.set("0.3")
        app.link2_radius.set("0.035")
        app.motor2_rpm.set("3000")
        app.reduction_ratio_m2.set("50")
        app.safety_factor_m2.set("1.5")
        app.motor1_rpm.set("3000")
        app.reduction_ratio_m1.set("50")
        app.safety_factor_m1.set("1.5")
        app.calculate_all()
    except Exception as e:
        messagebox.showerror("Error", f"Failed to reset to defaults: {str(e)}")

def show_about_dialog(root):
    """Show about dialog"""
    messagebox.showinfo(
        "About",
        "6DOF Robotic Arm Torque and Power Calculator\nVersion 1.0\nDeveloped for robotic arm design."
    )

if __name__ == "__main__":
    main()