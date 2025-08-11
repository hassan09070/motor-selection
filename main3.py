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
        self.root.geometry("1400x1000")
        
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
        
        # Link dimensions (L6, L5, L4, L3, L2, L1)
        self.L6 = tk.StringVar(value="0.2")
        self.L5 = tk.StringVar(value="0.3")
        self.L4 = tk.StringVar(value="0.25")
        self.L3 = tk.StringVar(value="0.25")
        self.L2 = tk.StringVar(value="0.3")
        self.L1 = tk.StringVar(value="0.0")  # Base link, can be 0
        
        # Link radii
        self.r6 = tk.StringVar(value="0.02")
        self.r5 = tk.StringVar(value="0.025")
        self.r4 = tk.StringVar(value="0.025")
        self.r3 = tk.StringVar(value="0.03")
        self.r2 = tk.StringVar(value="0.035")
        self.r1 = tk.StringVar(value="0.04")
        
        # Motor masses (will be updated from motor specs)
        self.m_motor6 = tk.StringVar(value="1.0")
        self.m_motor5 = tk.StringVar(value="1.2")
        self.m_motor4 = tk.StringVar(value="1.2")
        self.m_motor3 = tk.StringVar(value="1.5")
        self.m_motor2 = tk.StringVar(value="2.0")
        self.m_motor1 = tk.StringVar(value="2.5")
        
        # Motor body lengths
        self.a6 = tk.StringVar(value="0.1")
        self.a5 = tk.StringVar(value="0.12")
        self.a4 = tk.StringVar(value="0.12")
        self.a3 = tk.StringVar(value="0.15")
        self.a2 = tk.StringVar(value="0.18")
        self.a1 = tk.StringVar(value="0.2")
        
        # Motor pivot positions from base
        self.M6 = tk.StringVar(value="1.25")  # L1+L2+L3+L4+L5
        self.M5 = tk.StringVar(value="1.0")   # L1+L2+L3+L4
        self.M4 = tk.StringVar(value="0.75")  # L1+L2+L3
        self.M3 = tk.StringVar(value="0.5")   # L1+L2
        self.M2 = tk.StringVar(value="0.25")  # L1
        self.M1 = tk.StringVar(value="0.0")   # Base
        
        # Motor RPM values
        self.rpm6 = tk.StringVar(value="3000")
        self.rpm5 = tk.StringVar(value="3000")
        self.rpm4 = tk.StringVar(value="3000")
        self.rpm3 = tk.StringVar(value="3000")
        self.rpm2 = tk.StringVar(value="3000")
        self.rpm1 = tk.StringVar(value="3000")
        
        # Gear reduction ratios
        self.R6 = tk.StringVar(value="50")
        self.R5 = tk.StringVar(value="50")
        self.R4 = tk.StringVar(value="50")
        self.R3 = tk.StringVar(value="50")
        self.R2 = tk.StringVar(value="50")
        self.R1 = tk.StringVar(value="50")
        
        # Safety factors
        self.SF6 = tk.StringVar(value="1.5")
        self.SF5 = tk.StringVar(value="1.5")
        self.SF4 = tk.StringVar(value="1.5")
        self.SF3 = tk.StringVar(value="1.5")
        self.SF2 = tk.StringVar(value="1.5")
        self.SF1 = tk.StringVar(value="1.5")
    
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
        
        # Global Parameters
        global_frame = ttk.LabelFrame(scrollable_frame, text="Global Parameters", padding=10)
        global_frame.pack(fill="x", pady=5)
        
        ttk.Label(global_frame, text="Payload Mass (kg):").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(global_frame, textvariable=self.payload_mass, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(global_frame, text="Link Density (kg/m³):").grid(row=0, column=2, sticky="w", padx=5)
        ttk.Entry(global_frame, textvariable=self.link_density, width=15).grid(row=0, column=3, padx=5)
        
        # Link Dimensions
        link_frame = ttk.LabelFrame(scrollable_frame, text="Link Dimensions", padding=10)
        link_frame.pack(fill="x", pady=5)
        
        links = [
            ("L6 (m):", self.L6, "r6 (m):", self.r6),
            ("L5 (m):", self.L5, "r5 (m):", self.r5),
            ("L4 (m):", self.L4, "r4 (m):", self.r4),
            ("L3 (m):", self.L3, "r3 (m):", self.r3),
            ("L2 (m):", self.L2, "r2 (m):", self.r2),
            ("L1 (m):", self.L1, "r1 (m):", self.r1)
        ]
        
        for i, (l_label, l_var, r_label, r_var) in enumerate(links):
            row = i // 2
            col = (i % 2) * 4
            ttk.Label(link_frame, text=l_label).grid(row=row, column=col, sticky="w", padx=5, pady=2)
            ttk.Entry(link_frame, textvariable=l_var, width=12).grid(row=row, column=col+1, padx=5, pady=2)
            ttk.Label(link_frame, text=r_label).grid(row=row, column=col+2, sticky="w", padx=5, pady=2)
            ttk.Entry(link_frame, textvariable=r_var, width=12).grid(row=row, column=col+3, padx=5, pady=2)
        
        # Motor Parameters for each motor
        motor_vars = [
            (6, self.M6, self.a6, self.rpm6, self.R6, self.SF6),
            (5, self.M5, self.a5, self.rpm5, self.R5, self.SF5),
            (4, self.M4, self.a4, self.rpm4, self.R4, self.SF4),
            (3, self.M3, self.a3, self.rpm3, self.R3, self.SF3),
            (2, self.M2, self.a2, self.rpm2, self.R2, self.SF2),
            (1, self.M1, self.a1, self.rpm1, self.R1, self.SF1)
        ]
        
        for motor_num, M_var, a_var, rpm_var, R_var, SF_var in motor_vars:
            frame = ttk.LabelFrame(scrollable_frame, text=f"Motor {motor_num} Parameters", padding=10)
            frame.pack(fill="x", pady=5)
            
            params = [
                ("Pivot Position M (m):", M_var),
                ("Body Length a (m):", a_var),
                ("RPM:", rpm_var),
                ("Reduction Ratio R:", R_var),
                ("Safety Factor SF:", SF_var)
            ]
            
            for i, (label, var) in enumerate(params):
                row = i // 3
                col = (i % 3) * 2
                ttk.Label(frame, text=label).grid(row=row, column=col, sticky="w", padx=5, pady=2)
                ttk.Entry(frame, textvariable=var, width=15).grid(row=row, column=col+1, padx=5, pady=2)
    
    def create_results_tab(self, parent):
        """Create results display tab with scrollbar"""
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
        """Create table tab with scrollable tables"""
        # Create main frame with scrollbar
        canvas = tk.Canvas(parent)
        scrollbar_y = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollbar_x = ttk.Scrollbar(parent, orient="horizontal", command=canvas.xview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)
        
        # Pack scrollbars and canvas
        scrollbar_y.pack(side="right", fill="y")
        scrollbar_x.pack(side="bottom", fill="x")
        canvas.pack(side="left", fill="both", expand=True)
        
        # Torque and Power Results Table
        old_table_frame = ttk.LabelFrame(scrollable_frame, text="Torque and Power Results", padding=10)
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
        
        # Create frame for old table with scrollbars
        old_tree_frame = ttk.Frame(old_table_frame)
        old_tree_frame.pack(fill="both", expand=True)
        
        self.old_tree = ttk.Treeview(old_tree_frame, columns=old_columns, show="headings", height=8)
        old_tree_scroll_y = ttk.Scrollbar(old_tree_frame, orient="vertical", command=self.old_tree.yview)
        old_tree_scroll_x = ttk.Scrollbar(old_tree_frame, orient="horizontal", command=self.old_tree.xview)
        
        self.old_tree.configure(yscrollcommand=old_tree_scroll_y.set, xscrollcommand=old_tree_scroll_x.set)
        
        for col in old_columns:
            self.old_tree.heading(col, text=col)
            self.old_tree.column(col, width=180, anchor="center")
        
        # Pack old table components
        self.old_tree.grid(row=0, column=0, sticky="nsew")
        old_tree_scroll_y.grid(row=0, column=1, sticky="ns")
        old_tree_scroll_x.grid(row=1, column=0, sticky="ew")
        old_tree_frame.grid_rowconfigure(0, weight=1)
        old_tree_frame.grid_columnconfigure(0, weight=1)
        
        # Motor Specifications Table (Normal)
        new_table_frame = ttk.LabelFrame(scrollable_frame, text="Motor Specifications (Normal)", padding=10)
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
        
        # Create frame for new table with scrollbars
        new_tree_frame = ttk.Frame(new_table_frame)
        new_tree_frame.pack(fill="both", expand=True)
        
        self.new_tree = ttk.Treeview(new_tree_frame, columns=new_columns, show="headings", height=8)
        new_tree_scroll_y = ttk.Scrollbar(new_tree_frame, orient="vertical", command=self.new_tree.yview)
        new_tree_scroll_x = ttk.Scrollbar(new_tree_frame, orient="horizontal", command=self.new_tree.xview)
        
        self.new_tree.configure(yscrollcommand=new_tree_scroll_y.set, xscrollcommand=new_tree_scroll_x.set)
        
        for col in new_columns:
            self.new_tree.heading(col, text=col)
            self.new_tree.column(col, width=150, anchor="center")
        
        # Pack new table components
        self.new_tree.grid(row=0, column=0, sticky="nsew")
        new_tree_scroll_y.grid(row=0, column=1, sticky="ns")
        new_tree_scroll_x.grid(row=1, column=0, sticky="ew")
        new_tree_frame.grid_rowconfigure(0, weight=1)
        new_tree_frame.grid_columnconfigure(0, weight=1)
        
        # Motor Specifications with Safety Factor Table
        sf_table_frame = ttk.LabelFrame(scrollable_frame, text="Motor Specifications with Safety Factor", padding=10)
        sf_table_frame.pack(fill="x", pady=5)
        
        # Create frame for SF table with scrollbars
        sf_tree_frame = ttk.Frame(sf_table_frame)
        sf_tree_frame.pack(fill="both", expand=True)
        
        self.sf_tree = ttk.Treeview(sf_tree_frame, columns=new_columns, show="headings", height=8)
        sf_tree_scroll_y = ttk.Scrollbar(sf_tree_frame, orient="vertical", command=self.sf_tree.yview)
        sf_tree_scroll_x = ttk.Scrollbar(sf_tree_frame, orient="horizontal", command=self.sf_tree.xview)
        
        self.sf_tree.configure(yscrollcommand=sf_tree_scroll_y.set, xscrollcommand=sf_tree_scroll_x.set)
        
        for col in new_columns:
            self.sf_tree.heading(col, text=col)
            self.sf_tree.column(col, width=150, anchor="center")
        
        # Pack SF table components
        self.sf_tree.grid(row=0, column=0, sticky="nsew")
        sf_tree_scroll_y.grid(row=0, column=1, sticky="ns")
        sf_tree_scroll_x.grid(row=1, column=0, sticky="ew")
        sf_tree_frame.grid_rowconfigure(0, weight=1)
        sf_tree_frame.grid_columnconfigure(0, weight=1)
    
    def create_diagram_tab(self, parent):
        """Create diagram tab with arm plot"""
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill="both", expand=True)
        
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
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
            row = i // 2
            col = (i % 2) * 2
            ttk.Label(parent, text=f"{result}:").grid(row=row, column=col, sticky="w", padx=5, pady=2)
            label = ttk.Label(parent, text="N/A", background="white", relief="sunken", width=20)
            label.grid(row=row, column=col+1, sticky="w", padx=5, pady=2)
            setattr(self, f"result_m{motor_num}_{i}", label)
    
    def bind_events(self):
        """Bind events for real-time calculation"""
        variables = [
            self.payload_mass, self.link_density,
            self.L6, self.L5, self.L4, self.L3, self.L2, self.L1,
            self.r6, self.r5, self.r4, self.r3, self.r2, self.r1,
            self.M6, self.M5, self.M4, self.M3, self.M2, self.M1,
            self.a6, self.a5, self.a4, self.a3, self.a2, self.a1,
            self.rpm6, self.rpm5, self.rpm4, self.rpm3, self.rpm2, self.rpm1,
            self.R6, self.R5, self.R4, self.R3, self.R2, self.R1,
            self.SF6, self.SF5, self.SF4, self.SF3, self.SF2, self.SF1
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
            
            # Get link lengths
            L1 = self.get_float_value(self.L1)
            L2 = self.get_float_value(self.L2, 0.1)
            L3 = self.get_float_value(self.L3, 0.1)
            L4 = self.get_float_value(self.L4, 0.1)
            L5 = self.get_float_value(self.L5, 0.1)
            L6 = self.get_float_value(self.L6, 0.1)
            
            # Create arm configuration
            x = [0.0]
            y = [0.0]
            
            # Link 1 (if exists)
            if L1 > 0:
                x.append(x[-1])
                y.append(y[-1] + L1)
            
            # Link 2 (vertical)
            x.append(x[-1])
            y.append(y[-1] + L2)
            
            # Links 3-6 (horizontal)
            for L in [L3, L4, L5, L6]:
                x.append(x[-1] + L)
                y.append(y[-1])
            
            # Plot the arm
            self.ax.plot(x, y, marker="o", linestyle="-", color="blue", linewidth=3, markersize=8)
            
            # Add joint labels
            for i, (xi, yi) in enumerate(zip(x, y)):
                if i < len(x) - 1:  # Don't label the end effector
                    self.ax.text(xi, yi + 0.05, f"J{i+1}", fontsize=10, ha="center", weight="bold")
            
            # Add payload at end
            self.ax.plot(x[-1], y[-1], marker="s", color="red", markersize=12, label="Payload")
            
            # Set limits and labels
            total_x = sum([L3, L4, L5, L6])
            total_y = L1 + L2
            margin = 0.2
            self.ax.set_xlim(-margin, total_x + margin)
            self.ax.set_ylim(-margin, total_y + margin)
            self.ax.set_title("6DOF Robotic Arm Configuration", fontsize=12, weight="bold")
            self.ax.grid(True, linestyle="--", alpha=0.7)
            self.ax.set_xlabel("X (m)")
            self.ax.set_ylabel("Y (m)")
            self.ax.legend()
            self.ax.axis("equal")
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"Error updating diagram: {e}")
    
    def calculate_motor_torque_power(self, motor_num, with_sf=False):
        """Calculate torque and power for a specific motor using new formulas"""
        try:
            # Get all parameter values
            m_payload = self.get_float_value(self.payload_mass)
            density = self.get_float_value(self.link_density)
            
            # Link dimensions
            L1 = self.get_float_value(self.L1)
            L2 = self.get_float_value(self.L2)
            L3 = self.get_float_value(self.L3)
            L4 = self.get_float_value(self.L4)
            L5 = self.get_float_value(self.L5)
            L6 = self.get_float_value(self.L6)
            
            r1 = self.get_float_value(self.r1)
            r2 = self.get_float_value(self.r2)
            r3 = self.get_float_value(self.r3)
            r4 = self.get_float_value(self.r4)
            r5 = self.get_float_value(self.r5)
            r6 = self.get_float_value(self.r6)
            
            # Motor parameters
            motors_data = {
                1: (self.M1, self.a1, self.rpm1, self.R1, self.SF1),
                2: (self.M2, self.a2, self.rpm2, self.R2, self.SF2),
                3: (self.M3, self.a3, self.rpm3, self.R3, self.SF3),
                4: (self.M4, self.a4, self.rpm4, self.R4, self.SF4),
                5: (self.M5, self.a5, self.rpm5, self.R5, self.SF5),
                6: (self.M6, self.a6, self.rpm6, self.R6, self.SF6)
            }
            
            M_var, a_var, rpm_var, R_var, SF_var = motors_data[motor_num]
            M = self.get_float_value(M_var)
            a = self.get_float_value(a_var)
            rpm = self.get_int_value(rpm_var)
            R = self.get_int_value(R_var)
            SF = self.get_float_value(SF_var)
            
            # Calculate joint positions from base
            S1 = L1
            S2 = L1 + L2
            S3 = L1 + L2 + L3
            S4 = L1 + L2 + L3 + L4
            S5 = L1 + L2 + L3 + L4 + L5
            S6 = L1 + L2 + L3 + L4 + L5 + L6
            
            # Calculate link weights
            W_L1 = self.g * density * self.PI * r1**2 * L1 if L1 > 0 else 0
            W_L2 = self.g * density * self.PI * r2**2 * L2
            W_L3 = self.g * density * self.PI * r3**2 * L3
            W_L4 = self.g * density * self.PI * r4**2 * L4
            W_L5 = self.g * density * self.PI * r5**2 * L5
            W_L6 = self.g * density * self.PI * r6**2 * L6
            
            # Get motor weights from specs (use existing if available, otherwise use defaults)
            m_motor1 = self.motor_specs_normal.get(1, {'motor_weight': 2.5})['motor_weight'] if not with_sf else self.motor_specs_sf.get(1, {'motor_weight': 2.5})['motor_weight']
            m_motor2 = self.motor_specs_normal.get(2, {'motor_weight': 2.0})['motor_weight'] if not with_sf else self.motor_specs_sf.get(2, {'motor_weight': 2.0})['motor_weight']
            m_motor3 = self.motor_specs_normal.get(3, {'motor_weight': 1.5})['motor_weight'] if not with_sf else self.motor_specs_sf.get(3, {'motor_weight': 1.5})['motor_weight']
            m_motor4 = self.motor_specs_normal.get(4, {'motor_weight': 1.2})['motor_weight'] if not with_sf else self.motor_specs_sf.get(4, {'motor_weight': 1.2})['motor_weight']
            m_motor5 = self.motor_specs_normal.get(5, {'motor_weight': 1.2})['motor_weight'] if not with_sf else self.motor_specs_sf.get(5, {'motor_weight': 1.2})['motor_weight']
            m_motor6 = self.motor_specs_normal.get(6, {'motor_weight': 1.0})['motor_weight'] if not with_sf else self.motor_specs_sf.get(6, {'motor_weight': 1.0})['motor_weight']
            
            W_M1 = self.g * m_motor1
            W_M2 = self.g * m_motor2
            W_M3 = self.g * m_motor3
            W_M4 = self.g * m_motor4
            W_M5 = self.g * m_motor5
            W_M6 = self.g * m_motor6
            
            W_P = self.g * m_payload
            
            # Motor positions (from input parameters)
            M1_pos = self.get_float_value(self.M1)
            M2_pos = self.get_float_value(self.M2)
            M3_pos = self.get_float_value(self.M3)
            M4_pos = self.get_float_value(self.M4)
            M5_pos = self.get_float_value(self.M5)
            M6_pos = self.get_float_value(self.M6)
            
            a1_val = self.get_float_value(self.a1)
            a2_val = self.get_float_value(self.a2)
            a3_val = self.get_float_value(self.a3)
            a4_val = self.get_float_value(self.a4)
            a5_val = self.get_float_value(self.a5)
            a6_val = self.get_float_value(self.a6)
            
            # Calculate torque based on motor number using new formulas
            if motor_num == 6:
                T_total = W_P * (S6 - M6_pos) + W_L6 * (S6 - M6_pos - L6/2)
            elif motor_num == 5:
                T_total = (W_P * (S6 - M5_pos) + 
                          W_L6 * (S6 - M5_pos - L6/2) +
                          W_M6 * ((M6_pos + a6_val/2) - M5_pos) +
                          W_L5 * (S5 - M5_pos - L5/2))
            elif motor_num == 4:
                T_total = (W_P * (S6 - M4_pos) +
                          W_L6 * (S6 - M4_pos - L6/2) +
                          W_L5 * (S5 - M4_pos - L5/2) +
                          W_M6 * ((M6_pos + a6_val/2) - M4_pos) +
                          W_M5 * ((M5_pos + a5_val/2) - M4_pos) +
                          W_L4 * (S4 - M4_pos - L4/2))
            elif motor_num == 3:
                T_total = (W_P * (S6 - M3_pos) +
                          W_L6 * (S6 - M3_pos - L6/2) +
                          W_L5 * (S5 - M3_pos - L5/2) +
                          W_L4 * (S4 - M3_pos - L4/2) +
                          W_M6 * ((M6_pos + a6_val/2) - M3_pos) +
                          W_M5 * ((M5_pos + a5_val/2) - M3_pos) +
                          W_M4 * ((M4_pos + a4_val/2) - M3_pos) +
                          W_L3 * (S3 - M3_pos - L3/2))
            elif motor_num == 2:
                T_total = (W_P * (S6 - M2_pos) +
                          W_L6 * (S6 - M2_pos - L6/2) +
                          W_L5 * (S5 - M2_pos - L5/2) +
                          W_L4 * (S4 - M2_pos - L4/2) +
                          W_L3 * (S3 - M2_pos - L3/2) +
                          W_L2 * (S2 - M2_pos - L2/2) +
                          W_M6 * ((M6_pos + a6_val/2) - M2_pos) +
                          W_M5 * ((M5_pos + a5_val/2) - M2_pos) +
                          W_M4 * ((M4_pos + a4_val/2) - M2_pos) +
                          W_M3 * ((M3_pos + a3_val/2) - M2_pos))
            elif motor_num == 1:
                T_total = (W_P * (S6 - M1_pos) +
                          W_L6 * (S6 - M1_pos - L6/2) +
                          W_L5 * (S5 - M1_pos - L5/2) +
                          W_L4 * (S4 - M1_pos - L4/2) +
                          W_L3 * (S3 - M1_pos - L3/2) +
                          W_L2 * (S2 - M1_pos - L2/2) +
                          (W_L1 * (S1 - M1_pos - L1/2) if L1 > 0 else 0) +
                          W_M6 * ((M6_pos + a6_val/2) - M1_pos) +
                          W_M5 * ((M5_pos + a5_val/2) - M1_pos) +
                          W_M4 * ((M4_pos + a4_val/2) - M1_pos) +
                          W_M3 * ((M3_pos + a3_val/2) - M1_pos) +
                          W_M2 * ((M2_pos + a2_val/2) - M1_pos))
            
            # Apply safety factor if requested
            if with_sf:
                T_total_sf = SF * T_total
                T_before_sf = (T_total / R * SF) if R != 0 else 0
                P_sf = (T_total / R * rpm * 1000 / 9550 * SF) if (rpm != 0 and R != 0) else 0
                return {
                    'total_torque_sf': T_total_sf,
                    'torque_before_reduction_sf': T_before_sf,
                    'power_sf': P_sf
                }
            else:
                T_before = T_total / R if R != 0 else 0
                P = (T_before * rpm * 1000 / 9550) if rpm != 0 else 0
                return {
                    'total_torque': T_total,
                    'torque_before_reduction': T_before,
                    'power': P,
                    'safety_factor': SF
                }
                
        except Exception as e:
            print(f"Error calculating Motor {motor_num}: {e}")
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
            
            # Update Motor Specifications Table (Normal)
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
                    f"{specs['power_rating']:.3f}" if isinstance(specs['power_rating'], float) else specs['power_rating'],
                    f"{specs['flange_size']:.1f}" if isinstance(specs['flange_size'], float) else specs['flange_size'],
                    specs['voltage_type'],
                    specs['model_name'],
                    specs['company_name'],
                    f"{specs['price']:.2f}" if isinstance(specs['price'], float) else specs['price'],
                    f"{specs['motor_weight']:.3f}" if isinstance(specs['motor_weight'], float) else specs['motor_weight']
                ]
                self.new_tree.insert("", "end", values=tuple(values))
            
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
                    f"{specs['power_rating']:.3f}" if isinstance(specs['power_rating'], float) else specs['power_rating'],
                    f"{specs['flange_size']:.1f}" if isinstance(specs['flange_size'], float) else specs['flange_size'],
                    specs['voltage_type'],
                    specs['model_name'],
                    specs['company_name'],
                    f"{specs['price']:.2f}" if isinstance(specs['price'], float) else specs['price'],
                    f"{specs['motor_weight']:.3f}" if isinstance(specs['motor_weight'], float) else specs['motor_weight']
                ]
                self.sf_tree.insert("", "end", values=tuple(values))
                
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
            
            # Add Motor Specifications Table (Normal)
            data.append(["Motor Specifications (Normal)"])
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
            data.append(new_headers)
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
            
            with open("robot_arm_results.csv", "w", newline="", encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerows(data)
            
            messagebox.showinfo("Success", "Results exported to robot_arm_results.csv")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to export CSV: {str(e)}")
    
    def calculate_all(self):
        """Calculate torque and power for all motors and update diagram"""
        try:
            # Calculate for motors 6 to 1 (dependencies flow backwards)
            normal_results = {}
            sf_results = {}
            
            # First calculate all motors without SF to get motor weights
            for motor_num in range(6, 0, -1):
                # Calculate normal torque and power
                results_normal = self.calculate_motor_torque_power(motor_num, with_sf=False)
                normal_results[motor_num] = results_normal
                
                # Get motor specs based on normal calculations
                total_torque = results_normal['total_torque']
                power = results_normal['power']
                self.motor_specs_normal[motor_num] = get_motor_specs(motor_num, total_torque, power)
            
            # Then calculate all motors with SF using updated motor weights
            for motor_num in range(6, 0, -1):
                # Calculate SF torque and power
                results_sf = self.calculate_motor_torque_power(motor_num, with_sf=True)
                sf_results[motor_num] = results_sf
                
                # Get motor specs based on SF calculations
                total_torque_sf = results_sf['total_torque_sf']
                power_sf = results_sf['power_sf']
                self.motor_specs_sf[motor_num] = get_motor_specs(motor_num, total_torque_sf, power_sf)
            
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
        
        root.minsize(1200, 800)
        
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
        # Global parameters
        app.payload_mass.set("5.0")
        app.link_density.set("7850.0")
        
        # Link dimensions
        app.L6.set("0.2")
        app.L5.set("0.3")
        app.L4.set("0.25")
        app.L3.set("0.25")
        app.L2.set("0.3")
        app.L1.set("0.0")
        
        app.r6.set("0.02")
        app.r5.set("0.025")
        app.r4.set("0.025")
        app.r3.set("0.03")
        app.r2.set("0.035")
        app.r1.set("0.04")
        
        # Motor pivot positions (calculated from cumulative link lengths)
        app.M6.set("1.25")  # L1+L2+L3+L4+L5
        app.M5.set("1.0")   # L1+L2+L3+L4
        app.M4.set("0.75")  # L1+L2+L3
        app.M3.set("0.5")   # L1+L2
        app.M2.set("0.25")  # L1
        app.M1.set("0.0")
        
        # Motor body lengths
        app.a6.set("0.1")
        app.a5.set("0.12")
        app.a4.set("0.12")
        app.a3.set("0.15")
        app.a2.set("0.18")
        app.a1.set("0.2")
        
        # Motor RPM
        app.rpm6.set("3000")
        app.rpm5.set("3000")
        app.rpm4.set("3000")
        app.rpm3.set("3000")
        app.rpm2.set("3000")
        app.rpm1.set("3000")
        
        # Reduction ratios
        app.R6.set("50")
        app.R5.set("50")
        app.R4.set("50")
        app.R3.set("50")
        app.R2.set("50")
        app.R1.set("50")
        
        # Safety factors
        app.SF6.set("1.5")
        app.SF5.set("1.5")
        app.SF4.set("1.5")
        app.SF3.set("1.5")
        app.SF2.set("1.5")
        app.SF1.set("1.5")
        
        app.calculate_all()
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to reset to defaults: {str(e)}")

def show_about_dialog(root):
    """Show about dialog"""
    messagebox.showinfo(
        "About",
        "6DOF Robotic Arm Torque and Power Calculator\nVersion 2.0\nUpdated with new formulas and parameters\nDeveloped for robotic arm design."
    )

if __name__ == "__main__":
    main()