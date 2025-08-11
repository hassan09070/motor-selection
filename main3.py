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
        
        # Store motor specs
        self.motor_specs = {}
        
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
        
        # Link lengths (L1, L2, L3, L4, L5, L6)
        self.link1_length = tk.StringVar(value="0.2")
        self.link1_radius = tk.StringVar(value="0.04")
        self.link2_length = tk.StringVar(value="0.3")
        self.link2_radius = tk.StringVar(value="0.035")
        self.link3_length = tk.StringVar(value="0.25")
        self.link3_radius = tk.StringVar(value="0.03")
        self.link4_length = tk.StringVar(value="0.25")
        self.link4_radius = tk.StringVar(value="0.025")
        self.link5_length = tk.StringVar(value="0.3")
        self.link5_radius = tk.StringVar(value="0.025")
        self.link6_length = tk.StringVar(value="0.2")
        self.link6_radius = tk.StringVar(value="0.02")
        
        # Motor pivot positions (M1, M2, M3, M4, M5, M6)
        self.motor1_pivot = tk.StringVar(value="0.0")   # Base motor
        self.motor2_pivot = tk.StringVar(value="0.2")   # After link 1
        self.motor3_pivot = tk.StringVar(value="0.5")   # After link 1 + link 2
        self.motor4_pivot = tk.StringVar(value="0.75")  # After link 1 + link 2 + link 3
        self.motor5_pivot = tk.StringVar(value="1.0")   # After link 1 + link 2 + link 3 + link 4
        self.motor6_pivot = tk.StringVar(value="1.3")   # After link 1 + link 2 + link 3 + link 4 + link 5
        
        # Motor body lengths (a1, a2, a3, a4, a5, a6)
        self.motor1_length = tk.StringVar(value="0.15")
        self.motor2_length = tk.StringVar(value="0.12")
        self.motor3_length = tk.StringVar(value="0.15")
        self.motor4_length = tk.StringVar(value="0.12")
        self.motor5_length = tk.StringVar(value="0.12")
        self.motor6_length = tk.StringVar(value="0.1")
        
        # Motor parameters for each motor
        self.motor1_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m1 = tk.StringVar(value="50")
        self.safety_factor_m1 = tk.StringVar(value="1.5")
        
        self.motor2_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m2 = tk.StringVar(value="50")
        self.safety_factor_m2 = tk.StringVar(value="1.5")
        
        self.motor3_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m3 = tk.StringVar(value="50")
        self.safety_factor_m3 = tk.StringVar(value="1.5")
        
        self.motor4_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m4 = tk.StringVar(value="50")
        self.safety_factor_m4 = tk.StringVar(value="1.5")
        
        self.motor5_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m5 = tk.StringVar(value="50")
        self.safety_factor_m5 = tk.StringVar(value="1.5")
        
        self.motor6_rpm = tk.StringVar(value="3000")
        self.reduction_ratio_m6 = tk.StringVar(value="50")
        self.safety_factor_m6 = tk.StringVar(value="1.5")
        
        # Link lengths for diagram
        self.link_lengths = [
            self.link1_length,
            self.link2_length,
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
        
        # Global parameters
        global_frame = ttk.LabelFrame(scrollable_frame, text="Global Parameters", padding=10)
        global_frame.pack(fill="x", pady=5)
        
        ttk.Label(global_frame, text="Payload Mass (kg):").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(global_frame, textvariable=self.payload_mass, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(global_frame, text="Link Density (kg/m³):").grid(row=0, column=2, sticky="w", padx=5)
        ttk.Entry(global_frame, textvariable=self.link_density, width=15).grid(row=0, column=3, padx=5)
        
        # Link parameters
        links_frame = ttk.LabelFrame(scrollable_frame, text="Link Parameters", padding=10)
        links_frame.pack(fill="x", pady=5)
        
        link_params = [
            ("Link 1", self.link1_length, self.link1_radius),
            ("Link 2", self.link2_length, self.link2_radius),
            ("Link 3", self.link3_length, self.link3_radius),
            ("Link 4", self.link4_length, self.link4_radius),
            ("Link 5", self.link5_length, self.link5_radius),
            ("Link 6", self.link6_length, self.link6_radius),
        ]
        
        for i, (label, length_var, radius_var) in enumerate(link_params):
            row = i // 3
            col = (i % 3) * 4
            ttk.Label(links_frame, text=f"{label} Length (m):").grid(row=row, column=col, sticky="w", padx=5, pady=2)
            ttk.Entry(links_frame, textvariable=length_var, width=15).grid(row=row, column=col+1, padx=5, pady=2)
            ttk.Label(links_frame, text=f"{label} Radius (m):").grid(row=row, column=col+2, sticky="w", padx=5, pady=2)
            ttk.Entry(links_frame, textvariable=radius_var, width=15).grid(row=row, column=col+3, padx=5, pady=2)
        
        # Motor pivot positions
        pivot_frame = ttk.LabelFrame(scrollable_frame, text="Motor Pivot Positions", padding=10)
        pivot_frame.pack(fill="x", pady=5)
        
        pivot_vars = [
            ("Motor 1 Pivot (m):", self.motor1_pivot),
            ("Motor 2 Pivot (m):", self.motor2_pivot),
            ("Motor 3 Pivot (m):", self.motor3_pivot),
            ("Motor 4 Pivot (m):", self.motor4_pivot),
            ("Motor 5 Pivot (m):", self.motor5_pivot),
            ("Motor 6 Pivot (m):", self.motor6_pivot),
        ]
        
        for i, (label, var) in enumerate(pivot_vars):
            row = i // 3
            col = (i % 3) * 2
            ttk.Label(pivot_frame, text=label).grid(row=row, column=col, sticky="w", padx=5, pady=2)
            ttk.Entry(pivot_frame, textvariable=var, width=15).grid(row=row, column=col+1, padx=5, pady=2)
        
        # Motor parameters
        for motor_num in range(1, 7):
            motor_frame = ttk.LabelFrame(scrollable_frame, text=f"Motor {motor_num} Parameters", padding=10)
            motor_frame.pack(fill="x", pady=5)
            self.create_motor_inputs(motor_frame, motor_num)
    
    def create_motor_inputs(self, parent, motor_num):
        """Create input fields for a motor"""
        # Get the appropriate variables for this motor
        motor_vars = {
            1: {
                "Motor Body Length (m)": self.motor1_length,
                "Motor RPM": self.motor1_rpm,
                "Reduction Ratio": self.reduction_ratio_m1,
                "Safety Factor": self.safety_factor_m1
            },
            2: {
                "Motor Body Length (m)": self.motor2_length,
                "Motor RPM": self.motor2_rpm,
                "Reduction Ratio": self.reduction_ratio_m2,
                "Safety Factor": self.safety_factor_m2
            },
            3: {
                "Motor Body Length (m)": self.motor3_length,
                "Motor RPM": self.motor3_rpm,
                "Reduction Ratio": self.reduction_ratio_m3,
                "Safety Factor": self.safety_factor_m3
            },
            4: {
                "Motor Body Length (m)": self.motor4_length,
                "Motor RPM": self.motor4_rpm,
                "Reduction Ratio": self.reduction_ratio_m4,
                "Safety Factor": self.safety_factor_m4
            },
            5: {
                "Motor Body Length (m)": self.motor5_length,
                "Motor RPM": self.motor5_rpm,
                "Reduction Ratio": self.reduction_ratio_m5,
                "Safety Factor": self.safety_factor_m5
            },
            6: {
                "Motor Body Length (m)": self.motor6_length,
                "Motor RPM": self.motor6_rpm,
                "Reduction Ratio": self.reduction_ratio_m6,
                "Safety Factor": self.safety_factor_m6
            }
        }
        
        params = motor_vars[motor_num]
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
            self.link1_length, self.link1_radius, self.link2_length, self.link2_radius,
            self.link3_length, self.link3_radius, self.link4_length, self.link4_radius,
            self.link5_length, self.link5_radius, self.link6_length, self.link6_radius,
            self.motor1_pivot, self.motor2_pivot, self.motor3_pivot, self.motor4_pivot,
            self.motor5_pivot, self.motor6_pivot,
            self.motor1_length, self.motor2_length, self.motor3_length,
            self.motor4_length, self.motor5_length, self.motor6_length,
            self.motor1_rpm, self.reduction_ratio_m1, self.safety_factor_m1,
            self.motor2_rpm, self.reduction_ratio_m2, self.safety_factor_m2,
            self.motor3_rpm, self.reduction_ratio_m3, self.safety_factor_m3,
            self.motor4_rpm, self.reduction_ratio_m4, self.safety_factor_m4,
            self.motor5_rpm, self.reduction_ratio_m5, self.safety_factor_m5,
            self.motor6_rpm, self.reduction_ratio_m6, self.safety_factor_m6
        ]
        
        for var in variables:
            var.trace_add("write", self.on_value_change)
    
    def on_value_change(self, *args):
        """Handle value change event"""
        self.calculate_all()
    
    def get_float_value(self, var, default=0.0):
        """Safely get float value from StringVar"""
        try:
            return float(var.get())
        except (ValueError, tk.TclError):
            return default
    
    def get_int_value(self, var, default=0):
        """Safely get int value from StringVar"""
        try:
            return int(var.get())
        except (ValueError, tk.TclError):
            return default
    
    def get_joint_positions(self):
        """Calculate joint positions S1, S2, S3, S4, S5, S6"""
        L1 = self.get_float_value(self.link1_length)
        L2 = self.get_float_value(self.link2_length)
        L3 = self.get_float_value(self.link3_length)
        L4 = self.get_float_value(self.link4_length)
        L5 = self.get_float_value(self.link5_length)
        L6 = self.get_float_value(self.link6_length)
        
        S1 = L1
        S2 = L1 + L2
        S3 = L1 + L2 + L3
        S4 = L1 + L2 + L3 + L4
        S5 = L1 + L2 + L3 + L4 + L5
        S6 = L1 + L2 + L3 + L4 + L5 + L6
        
        return S1, S2, S3, S4, S5, S6
    
    def get_link_weights(self):
        """Calculate link weights"""
        density = self.get_float_value(self.link_density)
        
        weights = {}
        for i in range(1, 7):
            length_var = getattr(self, f'link{i}_length')
            radius_var = getattr(self, f'link{i}_radius')
            length = self.get_float_value(length_var)
            radius = self.get_float_value(radius_var)
            weights[i] = self.g * density * self.PI * radius**2 * length
        
        return weights
    
    def get_motor_weights(self):
        """Get motor weights from motor specs"""
        weights = {}
        for i in range(1, 7):
            if i in self.motor_specs:
                weights[i] = self.g * self.motor_specs[i].get('motor_weight', 0.0)
            else:
                weights[i] = 0.0
        return weights
    
    def update_diagram(self):
        """Update the arm diagram"""
        try:
            self.ax.clear()
            x = [0.0]
            y = [0.0]
            
            # Draw links based on joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Vertical link 1
            x.append(x[-1])
            y.append(y[-1] + self.get_float_value(self.link1_length, 0.1))
            
            # Horizontal links 2-6
            for length_var in [self.link2_length, self.link3_length, self.link4_length, self.link5_length, self.link6_length]:
                length = max(self.get_float_value(length_var, 0.1), 0.1)
                x.append(x[-1] + length)
                y.append(y[-1])
            
            self.ax.plot(x, y, marker="o", linestyle="-", color="blue", linewidth=2, markersize=6)
            
            for i, (xi, yi) in enumerate(zip(x, y)):
                self.ax.text(xi, yi + 0.07, f"J{i+1}", fontsize=7, ha="center")
            
            total_horizontal_length = sum(max(self.get_float_value(l, 0.1), 0.1) for l in self.link_lengths[1:])
            vertical_length = max(self.get_float_value(self.link1_length, 0.1), 0.1)
            self.ax.set_xlim(-0.3, total_horizontal_length + 0.3)
            self.ax.set_ylim(-0.3, vertical_length + 0.3)
            self.ax.set_title("6 DOF Arm", fontsize=9, pad=5)
            self.ax.grid(True, linestyle="--", alpha=0.7)
            self.ax.axis("equal")
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"Error updating diagram: {e}")
    
    def calculate_motor6_torque_and_power(self):
        """Calculate Motor 6 torque and power requirements"""
        try:
            # Get parameters
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            L6 = self.get_float_value(self.link6_length)
            r6 = self.get_float_value(self.link6_radius)
            M6 = self.get_float_value(self.motor6_pivot)
            rpm6 = self.get_int_value(self.motor6_rpm)
            R6 = self.get_int_value(self.reduction_ratio_m6)
            SF6 = self.get_float_value(self.safety_factor_m6)
            
            # Get joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Calculate weights
            W_P = self.g * payload_mass
            W_L6 = self.g * link_density * self.PI * r6**2 * L6
            
            # Calculate torques (following pseudocode exactly)
            T6_payload = W_P * (S6 - M6)
            T6_L6 = W_L6 * (S6 - M6 - L6/2)
            T6_total = T6_payload + T6_L6
            
            T6_sf = SF6 * T6_total
            T6_before = T6_total / R6 if R6 != 0 else 0
            T6_before_sf = SF6 * T6_before
            P6 = (T6_before * rpm6 * 1000 / 9550) if rpm6 != 0 else 0
            P6_sf = SF6 * P6
            
            self.motor_specs[6] = get_motor_specs(6, T6_sf, P6_sf)
            
            return {
                'total_torque': T6_total,
                'total_torque_sf': T6_sf,
                'torque_before_reduction': T6_before,
                'torque_before_reduction_sf': T6_before_sf,
                'power': P6,
                'power_sf': P6_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 6 calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor5_torque_and_power(self):
        """Calculate Motor 5 torque and power requirements"""
        try:
            # Get parameters
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            L6 = self.get_float_value(self.link6_length)
            r6 = self.get_float_value(self.link6_radius)
            L5 = self.get_float_value(self.link5_length)
            r5 = self.get_float_value(self.link5_radius)
            M5 = self.get_float_value(self.motor5_pivot)
            M6 = self.get_float_value(self.motor6_pivot)
            a6 = self.get_float_value(self.motor6_length)
            rpm5 = self.get_int_value(self.motor5_rpm)
            R5 = self.get_int_value(self.reduction_ratio_m5)
            SF5 = self.get_float_value(self.safety_factor_m5)
            
            # Get joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Calculate weights
            W_P = self.g * payload_mass
            W_L6 = self.g * link_density * self.PI * r6**2 * L6
            W_L5 = self.g * link_density * self.PI * r5**2 * L5
            m_motor6 = self.motor_specs.get(6, {'motor_weight': 0.0})['motor_weight']
            W_M6 = self.g * m_motor6
            
            # Calculate torques (following pseudocode exactly)
            T5_payload = W_P * (S6 - M5)
            T5_L6 = W_L6 * (S6 - M5 - L6/2)
            T5_M6 = W_M6 * ((M6 + a6/2) - M5)
            T5_L5 = W_L5 * (S5 - M5 - L5/2)
            T5_total = T5_payload + T5_L6 + T5_M6 + T5_L5
            
            T5_sf = SF5 * T5_total
            T5_before = T5_total / R5 if R5 != 0 else 0
            T5_before_sf = SF5 * T5_before
            P5 = (T5_before * rpm5 * 1000 / 9550) if rpm5 != 0 else 0
            P5_sf = SF5 * P5
            
            self.motor_specs[5] = get_motor_specs(5, T5_sf, P5_sf)
            
            return {
                'total_torque': T5_total,
                'total_torque_sf': T5_sf,
                'torque_before_reduction': T5_before,
                'torque_before_reduction_sf': T5_before_sf,
                'power': P5,
                'power_sf': P5_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 5 calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor4_torque_and_power(self):
        """Calculate Motor 4 torque and power requirements"""
        try:
            # Get parameters
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            L6 = self.get_float_value(self.link6_length)
            r6 = self.get_float_value(self.link6_radius)
            L5 = self.get_float_value(self.link5_length)
            r5 = self.get_float_value(self.link5_radius)
            L4 = self.get_float_value(self.link4_length)
            r4 = self.get_float_value(self.link4_radius)
            M4 = self.get_float_value(self.motor4_pivot)
            M5 = self.get_float_value(self.motor5_pivot)
            M6 = self.get_float_value(self.motor6_pivot)
            a5 = self.get_float_value(self.motor5_length)
            a6 = self.get_float_value(self.motor6_length)
            rpm4 = self.get_int_value(self.motor4_rpm)
            R4 = self.get_int_value(self.reduction_ratio_m4)
            SF4 = self.get_float_value(self.safety_factor_m4)
            
            # Get joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Calculate weights
            W_P = self.g * payload_mass
            W_L6 = self.g * link_density * self.PI * r6**2 * L6
            W_L5 = self.g * link_density * self.PI * r5**2 * L5
            W_L4 = self.g * link_density * self.PI * r4**2 * L4
            m_motor6 = self.motor_specs.get(6, {'motor_weight': 0.0})['motor_weight']
            m_motor5 = self.motor_specs.get(5, {'motor_weight': 0.0})['motor_weight']
            W_M6 = self.g * m_motor6
            W_M5 = self.g * m_motor5
            
            # Calculate torques (following pseudocode exactly)
            T4_payload = W_P * (S6 - M4)
            T4_L6 = W_L6 * (S6 - M4 - L6/2)
            T4_L5 = W_L5 * (S5 - M4 - L5/2)
            T4_M6 = W_M6 * ((M6 + a6/2) - M4)
            T4_M5 = W_M5 * ((M5 + a5/2) - M4)
            T4_L4 = W_L4 * (S4 - M4 - L4/2)
            T4_total = T4_payload + T4_L6 + T4_L5 + T4_M6 + T4_M5 + T4_L4
            
            T4_sf = SF4 * T4_total
            T4_before = T4_total / R4 if R4 != 0 else 0
            T4_before_sf = SF4 * T4_before
            P4 = (T4_before * rpm4 * 1000 / 9550) if rpm4 != 0 else 0
            P4_sf = SF4 * P4
            
            self.motor_specs[4] = get_motor_specs(4, T4_sf, P4_sf)
            
            return {
                'total_torque': T4_total,
                'total_torque_sf': T4_sf,
                'torque_before_reduction': T4_before,
                'torque_before_reduction_sf': T4_before_sf,
                'power': P4,
                'power_sf': P4_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 4 calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor3_torque_and_power(self):
        """Calculate Motor 3 torque and power requirements"""
        try:
            # Get parameters
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            L6 = self.get_float_value(self.link6_length)
            r6 = self.get_float_value(self.link6_radius)
            L5 = self.get_float_value(self.link5_length)
            r5 = self.get_float_value(self.link5_radius)
            L4 = self.get_float_value(self.link4_length)
            r4 = self.get_float_value(self.link4_radius)
            L3 = self.get_float_value(self.link3_length)
            r3 = self.get_float_value(self.link3_radius)
            M3 = self.get_float_value(self.motor3_pivot)
            M4 = self.get_float_value(self.motor4_pivot)
            M5 = self.get_float_value(self.motor5_pivot)
            M6 = self.get_float_value(self.motor6_pivot)
            a4 = self.get_float_value(self.motor4_length)
            a5 = self.get_float_value(self.motor5_length)
            a6 = self.get_float_value(self.motor6_length)
            rpm3 = self.get_int_value(self.motor3_rpm)
            R3 = self.get_int_value(self.reduction_ratio_m3)
            SF3 = self.get_float_value(self.safety_factor_m3)
            
            # Get joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Calculate weights
            W_P = self.g * payload_mass
            W_L6 = self.g * link_density * self.PI * r6**2 * L6
            W_L5 = self.g * link_density * self.PI * r5**2 * L5
            W_L4 = self.g * link_density * self.PI * r4**2 * L4
            W_L3 = self.g * link_density * self.PI * r3**2 * L3
            m_motor6 = self.motor_specs.get(6, {'motor_weight': 0.0})['motor_weight']
            m_motor5 = self.motor_specs.get(5, {'motor_weight': 0.0})['motor_weight']
            m_motor4 = self.motor_specs.get(4, {'motor_weight': 0.0})['motor_weight']
            W_M6 = self.g * m_motor6
            W_M5 = self.g * m_motor5
            W_M4 = self.g * m_motor4
            
            # Calculate torques (following pseudocode exactly)
            T3_payload = W_P * (S6 - M3)
            T3_L6 = W_L6 * (S6 - M3 - L6/2)
            T3_L5 = W_L5 * (S5 - M3 - L5/2)
            T3_L4 = W_L4 * (S4 - M3 - L4/2)
            T3_M6 = W_M6 * ((M6 + a6/2) - M3)
            T3_M5 = W_M5 * ((M5 + a5/2) - M3)
            T3_M4 = W_M4 * ((M4 + a4/2) - M3)
            T3_L3 = W_L3 * (S3 - M3 - L3/2)
            T3_total = T3_payload + T3_L6 + T3_L5 + T3_L4 + T3_M6 + T3_M5 + T3_M4 + T3_L3
            
            T3_sf = SF3 * T3_total
            T3_before = T3_total / R3 if R3 != 0 else 0
            T3_before_sf = SF3 * T3_before
            P3 = (T3_before * rpm3 * 1000 / 9550) if rpm3 != 0 else 0
            P3_sf = SF3 * P3
            
            self.motor_specs[3] = get_motor_specs(3, T3_sf, P3_sf)
            
            return {
                'total_torque': T3_total,
                'total_torque_sf': T3_sf,
                'torque_before_reduction': T3_before,
                'torque_before_reduction_sf': T3_before_sf,
                'power': P3,
                'power_sf': P3_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 3 calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor2_torque_and_power(self):
        """Calculate Motor 2 torque and power requirements"""
        try:
            # Get parameters
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            L6 = self.get_float_value(self.link6_length)
            r6 = self.get_float_value(self.link6_radius)
            L5 = self.get_float_value(self.link5_length)
            r5 = self.get_float_value(self.link5_radius)
            L4 = self.get_float_value(self.link4_length)
            r4 = self.get_float_value(self.link4_radius)
            L3 = self.get_float_value(self.link3_length)
            r3 = self.get_float_value(self.link3_radius)
            L2 = self.get_float_value(self.link2_length)
            r2 = self.get_float_value(self.link2_radius)
            M2 = self.get_float_value(self.motor2_pivot)
            M3 = self.get_float_value(self.motor3_pivot)
            M4 = self.get_float_value(self.motor4_pivot)
            M5 = self.get_float_value(self.motor5_pivot)
            M6 = self.get_float_value(self.motor6_pivot)
            a3 = self.get_float_value(self.motor3_length)
            a4 = self.get_float_value(self.motor4_length)
            a5 = self.get_float_value(self.motor5_length)
            a6 = self.get_float_value(self.motor6_length)
            rpm2 = self.get_int_value(self.motor2_rpm)
            R2 = self.get_int_value(self.reduction_ratio_m2)
            SF2 = self.get_float_value(self.safety_factor_m2)
            
            # Get joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Calculate weights
            W_P = self.g * payload_mass
            W_L6 = self.g * link_density * self.PI * r6**2 * L6
            W_L5 = self.g * link_density * self.PI * r5**2 * L5
            W_L4 = self.g * link_density * self.PI * r4**2 * L4
            W_L3 = self.g * link_density * self.PI * r3**2 * L3
            W_L2 = self.g * link_density * self.PI * r2**2 * L2
            m_motor6 = self.motor_specs.get(6, {'motor_weight': 0.0})['motor_weight']
            m_motor5 = self.motor_specs.get(5, {'motor_weight': 0.0})['motor_weight']
            m_motor4 = self.motor_specs.get(4, {'motor_weight': 0.0})['motor_weight']
            m_motor3 = self.motor_specs.get(3, {'motor_weight': 0.0})['motor_weight']
            W_M6 = self.g * m_motor6
            W_M5 = self.g * m_motor5
            W_M4 = self.g * m_motor4
            W_M3 = self.g * m_motor3
            
            # Calculate torques (following pseudocode exactly)
            T2_payload = W_P * (S6 - M2)
            T2_L6 = W_L6 * (S6 - M2 - L6/2)
            T2_L5 = W_L5 * (S5 - M2 - L5/2)
            T2_L4 = W_L4 * (S4 - M2 - L4/2)
            T2_L3 = W_L3 * (S3 - M2 - L3/2)
            T2_L2 = W_L2 * (S2 - M2 - L2/2)
            T2_M6 = W_M6 * ((M6 + a6/2) - M2)
            T2_M5 = W_M5 * ((M5 + a5/2) - M2)
            T2_M4 = W_M4 * ((M4 + a4/2) - M2)
            T2_M3 = W_M3 * ((M3 + a3/2) - M2)
            T2_total = T2_payload + T2_L6 + T2_L5 + T2_L4 + T2_L3 + T2_L2 + T2_M6 + T2_M5 + T2_M4 + T2_M3
            
            T2_sf = SF2 * T2_total
            T2_before = T2_total / R2 if R2 != 0 else 0
            T2_before_sf = SF2 * T2_before
            P2 = (T2_before * rpm2 * 1000 / 9550) if rpm2 != 0 else 0
            P2_sf = SF2 * P2
            
            self.motor_specs[2] = get_motor_specs(2, T2_sf, P2_sf)
            
            return {
                'total_torque': T2_total,
                'total_torque_sf': T2_sf,
                'torque_before_reduction': T2_before,
                'torque_before_reduction_sf': T2_before_sf,
                'power': P2,
                'power_sf': P2_sf
            }
            
        except Exception as e:
            print(f"Error in Motor 2 calculation: {e}")
            return self.get_zero_results()
    
    def calculate_motor1_torque_and_power(self):
        """Calculate Motor 1 torque and power requirements"""
        try:
            # Get parameters
            payload_mass = self.get_float_value(self.payload_mass)
            link_density = self.get_float_value(self.link_density)
            L6 = self.get_float_value(self.link6_length)
            r6 = self.get_float_value(self.link6_radius)
            L5 = self.get_float_value(self.link5_length)
            r5 = self.get_float_value(self.link5_radius)
            L4 = self.get_float_value(self.link4_length)
            r4 = self.get_float_value(self.link4_radius)
            L3 = self.get_float_value(self.link3_length)
            r3 = self.get_float_value(self.link3_radius)
            L2 = self.get_float_value(self.link2_length)
            r2 = self.get_float_value(self.link2_radius)
            L1 = self.get_float_value(self.link1_length)
            r1 = self.get_float_value(self.link1_radius)
            M1 = self.get_float_value(self.motor1_pivot)
            M2 = self.get_float_value(self.motor2_pivot)
            M3 = self.get_float_value(self.motor3_pivot)
            M4 = self.get_float_value(self.motor4_pivot)
            M5 = self.get_float_value(self.motor5_pivot)
            M6 = self.get_float_value(self.motor6_pivot)
            a2 = self.get_float_value(self.motor2_length)
            a3 = self.get_float_value(self.motor3_length)
            a4 = self.get_float_value(self.motor4_length)
            a5 = self.get_float_value(self.motor5_length)
            a6 = self.get_float_value(self.motor6_length)
            rpm1 = self.get_int_value(self.motor1_rpm)
            R1 = self.get_int_value(self.reduction_ratio_m1)
            SF1 = self.get_float_value(self.safety_factor_m1)
            
            # Get joint positions
            S1, S2, S3, S4, S5, S6 = self.get_joint_positions()
            
            # Calculate weights
            W_P = self.g * payload_mass
            W_L6 = self.g * link_density * self.PI * r6**2 * L6
            W_L5 = self.g * link_density * self.PI * r5**2 * L5
            W_L4 = self.g * link_density * self.PI * r4**2 * L4
            W_L3 = self.g * link_density * self.PI * r3**2 * L3
            W_L2 = self.g * link_density * self.PI * r2**2 * L2
            W_L1 = self.g * link_density * self.PI * r1**2 * L1
            m_motor6 = self.motor_specs.get(6, {'motor_weight': 0.0})['motor_weight']
            m_motor5 = self.motor_specs.get(5, {'motor_weight': 0.0})['motor_weight']
            m_motor4 = self.motor_specs.get(4, {'motor_weight': 0.0})['motor_weight']
            m_motor3 = self.motor_specs.get(3, {'motor_weight': 0.0})['motor_weight']
            m_motor2 = self.motor_specs.get(2, {'motor_weight': 0.0})['motor_weight']
    
    def get_zero_results(self):
        """Return zero results in case of calculation error"""
        return {
            'total_torque': 0.0,
            'total_torque_sf': 0.0,
            'torque_before_reduction': 0.0,
            'torque_before_reduction_sf': 0.0,
            'power': 0.0,
            'power_sf': 0.0
        }
    
    def update_results_display(self, motor_num, results):
        """Update the results display for a specific motor"""
        try:
            # Get motor specs using normal torque and power
            specs = get_motor_specs(motor_num, results['total_torque'], results['power'])
            values = [
                specs['motor'],
                specs['power_rating'],
                specs['flange_size'],
                specs['voltage_type'],
                specs['model_name'],
                specs['company_name'],
                specs['price'],
                specs['motor_weight'],
                results['total_torque'],
                results['total_torque_sf'],
                results['torque_before_reduction'],
                results['torque_before_reduction_sf'],
                results['power'],
                results['power_sf']
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
                values = self.all_results.get(motor_num, [f"Motor {motor_num}"] + ["N/A"] * 13)[0:8]
                self.new_tree.insert("", "end", values=tuple([str(v) for v in values]))
            
            # Update Motor Specifications with Safety Factor Table
            for item in self.sf_tree.get_children():
                self.sf_tree.delete(item)
            
            for motor_num in range(1, 7):
                # Get motor specs using torque and power with safety factor
                total_torque_sf = self.all_results.get(motor_num, [0] * 14)[9]
                power_sf = self.all_results.get(motor_num, [0] * 14)[13]
                specs = get_motor_specs(motor_num, total_torque_sf, power_sf)
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
                values = self.all_results.get(motor_num, [f"Motor {motor_num}"] + ["N/A"] * 13)[0:8]
                data.append([str(v) for v in values])
            
            # Add separator
            data.append([])
            
            # Add Motor Specifications with Safety Factor Table
            data.append(["Motor Specifications with Safety Factor"])
            data.append(sf_headers)
            for motor_num in range(1, 7):
                total_torque_sf = self.all_results.get(motor_num, [0] * 14)[9]
                power_sf = self.all_results.get(motor_num, [0] * 14)[13]
                specs = get_motor_specs(motor_num, total_torque_sf, power_sf)
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
            # Calculate in reverse order (6 to 1) to ensure motor weights are available
            self.motor_specs = {}  # Reset motor specs
            
            for motor_num in range(6, 0, -1):  # 6, 5, 4, 3, 2, 1
                results = self.calculate_motor_torque_and_power(motor_num)
                self.update_results_display(motor_num, results)
            
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
        
        # Link parameters
        app.link1_length.set("0.2")
        app.link1_radius.set("0.04")
        app.link2_length.set("0.3")
        app.link2_radius.set("0.035")
        app.link3_length.set("0.25")
        app.link3_radius.set("0.03")
        app.link4_length.set("0.25")
        app.link4_radius.set("0.025")
        app.link5_length.set("0.3")
        app.link5_radius.set("0.025")
        app.link6_length.set("0.2")
        app.link6_radius.set("0.02")
        
        # Motor pivot positions
        app.motor1_pivot.set("0.0")
        app.motor2_pivot.set("0.2")
        app.motor3_pivot.set("0.5")
        app.motor4_pivot.set("0.75")
        app.motor5_pivot.set("1.0")
        app.motor6_pivot.set("1.3")
        
        # Motor body lengths
        app.motor1_length.set("0.15")
        app.motor2_length.set("0.12")
        app.motor3_length.set("0.15")
        app.motor4_length.set("0.12")
        app.motor5_length.set("0.12")
        app.motor6_length.set("0.1")
        
        # Motor parameters
        for i in range(1, 7):
            getattr(app, f'motor{i}_rpm').set("3000")
            getattr(app, f'reduction_ratio_m{i}').set("50")
            getattr(app, f'safety_factor_m{i}').set("1.5")
        
    except Exception as e:
        messagebox.showerror("Error", f"Error resetting to defaults: {str(e)}")

def show_about_dialog(parent):
    """Show about dialog"""
    about_text = """6DOF Robotic Arm Torque and Power Calculator

This application calculates the torque and power requirements for each motor in a 6DOF robotic arm using static analysis with arbitrary motor pivot positions.

Features:
- Real-time calculations with new arbitrary pivot position formulas
- Comprehensive torque analysis for each joint
- Configurable motor pivot positions and body lengths
- Safety factor considerations
- Gear reduction calculations
- Power requirements in Watts
- Triple table view of results
- Export results to CSV
- Arm diagram visualization
- Automated motor selection

Mathematical Approach:
- Static worst-case analysis with arbitrary motor pivot positions
- Joint position calculations (S1, S2, S3, S4, S5, S6)
- Motor center of mass considerations
- Moment calculations about each motor pivot point
- Uniform cylindrical link modeling

New Formula Features:
- Arbitrary motor pivot positions (M1-M6)
- Motor body lengths for center of mass calculations
- More accurate torque calculations considering all upstream loads

Senior Developer: M Hassan Shahzad
Junior Developer: Basil Saeed Bari
Version: 2.0"""
    
    messagebox.showinfo("About", about_text)

if __name__ == "__main__":
    main()