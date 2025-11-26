"""
Advanced Induction Motor Analysis Tool
Comprehensive Python + Tkinter GUI for electrical engineering calculations and dynamic simulation
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import math


class InductionMotorCalculator:
    """Core calculation engine for induction motor analysis"""

    def __init__(self):
        self.reset_parameters()

    def reset_parameters(self):
        """Reset all motor parameters to default values"""
        # Motor nameplate data
        self.power_hp = 20.0
        self.poles = 4
        self.frequency = 50.0
        self.voltage = 400.0
        self.rated_slip = 0.04
        self.efficiency = 0.90
        self.power_factor = 0.85

        # Loss parameters
        self.friction_windage_percent = 2.0
        self.stator_resistance = 0.5
        self.rotor_resistance = 0.3
        self.magnetizing_reactance = 50.0
        self.stator_leakage_reactance = 1.0
        self.rotor_leakage_reactance = 1.0

        # Mechanical parameters
        self.inertia = 0.5  # kg.m²
        self.load_torque = 0.0  # Nm
        self.friction_coeff = 0.01

    def calculate_steady_state(self):
        """Calculate steady-state performance at full load"""
        # Convert power to watts
        power_out_w = self.power_hp * 746.0

        # Synchronous speed
        ns_rpm = 120.0 * self.frequency / self.poles
        ns_rad_s = 2.0 * np.pi * ns_rpm / 60.0

        # Rotor speed
        nr_rpm = ns_rpm * (1.0 - self.rated_slip)
        nr_rad_s = 2.0 * np.pi * nr_rpm / 60.0

        # Friction and windage losses
        friction_windage_w = (self.friction_windage_percent / 100.0) * power_out_w

        # Mechanical power developed (shaft power + friction/windage)
        mech_power_dev = power_out_w + friction_windage_w

        # Rotor copper losses (I²R loss)
        # Using relationship: Pr : Pcr : Pm = 1 : s : (1-s)
        rotor_copper_loss = mech_power_dev * self.rated_slip / (1.0 - self.rated_slip)

        # Rotor input power
        rotor_input = mech_power_dev / (1.0 - self.rated_slip)

        # Output torque
        output_torque = power_out_w / nr_rad_s

        # Developed torque
        developed_torque = mech_power_dev / nr_rad_s

        # Air gap power (rotor input)
        air_gap_power = rotor_input

        # Estimate stator copper losses (assuming efficiency)
        total_input = power_out_w / self.efficiency
        stator_copper_loss = total_input - rotor_input - friction_windage_w
        if stator_copper_loss < 0:
            stator_copper_loss = 0.1 * rotor_input  # Estimate

        # Total losses
        total_losses = rotor_copper_loss + stator_copper_loss + friction_windage_w

        results = {
            'power_output_w': power_out_w,
            'power_output_hp': self.power_hp,
            'synchronous_speed_rpm': ns_rpm,
            'rotor_speed_rpm': nr_rpm,
            'slip_percent': self.rated_slip * 100.0,
            'rotor_copper_loss_w': rotor_copper_loss,
            'rotor_input_w': rotor_input,
            'output_torque_nm': output_torque,
            'developed_torque_nm': developed_torque,
            'friction_windage_w': friction_windage_w,
            'mechanical_power_dev_w': mech_power_dev,
            'stator_copper_loss_w': stator_copper_loss,
            'total_losses_w': total_losses,
            'air_gap_power_w': air_gap_power,
            'efficiency_percent': (power_out_w / total_input) * 100.0 if total_input > 0 else 0,
        }

        return results

    def calculate_torque_speed_curve(self, speed_range=None):
        """Calculate torque-speed characteristics using equivalent circuit"""
        if speed_range is None:
            ns_rpm = 120.0 * self.frequency / self.poles
            speed_range = np.linspace(0, ns_rpm * 1.1, 200)

        ns_rpm = 120.0 * self.frequency / self.poles

        torques = []
        slips = []

        for speed in speed_range:
            slip = (ns_rpm - speed) / ns_rpm if ns_rpm > 0 else 0
            slip = max(min(slip, 2.0), -0.5)  # Limit slip range
            slips.append(slip)

            if abs(slip) < 1e-6:
                torques.append(0)
                continue

            # Simplified equivalent circuit
            r2_s = self.rotor_resistance / slip if slip != 0 else 1e6

            # Thevenin equivalent
            z_mag = 1j * self.magnetizing_reactance
            z_stator = self.stator_resistance + 1j * self.stator_leakage_reactance

            # Voltage behind stator impedance
            v_th = self.voltage / np.sqrt(3) * abs(z_mag / (z_stator + z_mag))
            z_th = abs((z_stator * z_mag) / (z_stator + z_mag))

            # Total impedance
            z_total = np.sqrt((z_th + r2_s)**2 + (self.rotor_leakage_reactance)**2)

            # Current
            i2 = v_th / z_total if z_total > 0 else 0

            # Torque
            ns_rad_s = 2.0 * np.pi * ns_rpm / 60.0
            torque = (3 * i2**2 * r2_s) / ns_rad_s if ns_rad_s > 0 else 0
            torques.append(torque)

        return speed_range, np.array(torques), np.array(slips)

    def motor_dynamics(self, t, y, torque_load_func):
        """
        Differential equations for motor dynamics
        State vector y = [omega, theta]
        omega: angular velocity (rad/s)
        theta: rotor position (rad)
        """
        omega = y[0]

        # Synchronous speed
        ns_rpm = 120.0 * self.frequency / self.poles
        ns_rad_s = 2.0 * np.pi * ns_rpm / 60.0

        # Current speed in rpm
        current_rpm = omega * 60.0 / (2.0 * np.pi)

        # Slip
        slip = (ns_rpm - current_rpm) / ns_rpm if ns_rpm > 0 else 0

        # Electromagnetic torque (simplified model)
        if abs(slip) < 1e-6:
            T_em = 0
        else:
            r2_s = self.rotor_resistance / slip
            z_mag = self.magnetizing_reactance
            z_stator = np.sqrt(self.stator_resistance**2 + self.stator_leakage_reactance**2)

            v_th = self.voltage / np.sqrt(3) * z_mag / (z_stator + z_mag)
            z_th = (z_stator * z_mag) / (z_stator + z_mag)

            z_total = np.sqrt((z_th + r2_s)**2 + self.rotor_leakage_reactance**2)
            i2 = v_th / z_total if z_total > 0 else 0

            T_em = (3 * i2**2 * r2_s) / ns_rad_s if ns_rad_s > 0 else 0

        # Load torque
        T_load = torque_load_func(t, omega)

        # Friction torque
        T_friction = self.friction_coeff * omega

        # Equation of motion: J * dω/dt = T_em - T_load - T_friction
        domega_dt = (T_em - T_load - T_friction) / self.inertia

        # Angular position
        dtheta_dt = omega

        return [domega_dt, dtheta_dt]


class InductionMotorGUI:
    """Advanced Tkinter GUI for Induction Motor Analysis"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Induction Motor Analysis Tool")
        self.root.geometry("1400x900")

        # Initialize calculator
        self.calculator = InductionMotorCalculator()

        # Simulation state
        self.simulation_running = False
        self.simulation_data = None

        # Configure root window for resizing
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # Create main container
        self.create_main_layout()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

        # Initial calculation
        self.calculate_and_display()

    def create_main_layout(self):
        """Create the main application layout"""
        # Main container with notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_steady_state_tab()
        self.create_dynamic_simulation_tab()
        self.create_torque_speed_tab()
        self.create_about_tab()

    def create_steady_state_tab(self):
        """Create steady-state analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Steady-State Analysis")

        # Configure grid
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=2)

        # Left panel - Input parameters
        left_frame = ttk.LabelFrame(tab, text="Motor Parameters", padding=10)
        left_frame.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=5, pady=5)

        self.create_parameter_inputs(left_frame)

        # Right top panel - Results
        right_top_frame = ttk.LabelFrame(tab, text="Calculation Results", padding=10)
        right_top_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.create_results_display(right_top_frame)

        # Right bottom panel - Power flow diagram
        right_bottom_frame = ttk.LabelFrame(tab, text="Power Flow Visualization", padding=10)
        right_bottom_frame.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)
        right_bottom_frame.rowconfigure(0, weight=1)
        right_bottom_frame.columnconfigure(0, weight=1)

        # Create matplotlib figure for power flow
        self.power_flow_fig = Figure(figsize=(8, 4), dpi=100)
        self.power_flow_canvas = FigureCanvasTkAgg(self.power_flow_fig, right_bottom_frame)
        self.power_flow_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_parameter_inputs(self, parent):
        """Create input fields for motor parameters"""
        row = 0

        # Nameplate data section
        ttk.Label(parent, text="Nameplate Data", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, pady=(0, 10))
        row += 1

        # Power
        ttk.Label(parent, text="Power (HP):").grid(row=row, column=0, sticky='w', pady=2)
        self.power_var = tk.DoubleVar(value=self.calculator.power_hp)
        power_entry = ttk.Entry(parent, textvariable=self.power_var, width=15)
        power_entry.grid(row=row, column=1, pady=2)
        row += 1

        # Poles
        ttk.Label(parent, text="Poles:").grid(row=row, column=0, sticky='w', pady=2)
        self.poles_var = tk.IntVar(value=self.calculator.poles)
        poles_combo = ttk.Combobox(parent, textvariable=self.poles_var,
                                   values=[2, 4, 6, 8, 10, 12], width=13, state='readonly')
        poles_combo.grid(row=row, column=1, pady=2)
        row += 1

        # Frequency
        ttk.Label(parent, text="Frequency (Hz):").grid(row=row, column=0, sticky='w', pady=2)
        self.freq_var = tk.DoubleVar(value=self.calculator.frequency)
        freq_entry = ttk.Entry(parent, textvariable=self.freq_var, width=15)
        freq_entry.grid(row=row, column=1, pady=2)
        row += 1

        # Voltage
        ttk.Label(parent, text="Voltage (V):").grid(row=row, column=0, sticky='w', pady=2)
        self.voltage_var = tk.DoubleVar(value=self.calculator.voltage)
        voltage_entry = ttk.Entry(parent, textvariable=self.voltage_var, width=15)
        voltage_entry.grid(row=row, column=1, pady=2)
        row += 1

        # Rated slip
        ttk.Label(parent, text="Full-Load Slip (%):").grid(row=row, column=0, sticky='w', pady=2)
        self.slip_var = tk.DoubleVar(value=self.calculator.rated_slip * 100)
        slip_scale = ttk.Scale(parent, from_=0, to=10, variable=self.slip_var,
                              orient='horizontal', command=self.on_slider_change)
        slip_scale.grid(row=row, column=1, pady=2, sticky='ew')
        self.slip_label = ttk.Label(parent, text=f"{self.slip_var.get():.2f}%")
        self.slip_label.grid(row=row, column=2, pady=2)
        row += 1

        # Separator
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1

        # Loss parameters
        ttk.Label(parent, text="Loss Parameters", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, pady=(0, 10))
        row += 1

        # Friction and windage
        ttk.Label(parent, text="Friction & Windage (%):").grid(row=row, column=0, sticky='w', pady=2)
        self.fw_var = tk.DoubleVar(value=self.calculator.friction_windage_percent)
        fw_scale = ttk.Scale(parent, from_=0, to=10, variable=self.fw_var,
                            orient='horizontal', command=self.on_slider_change)
        fw_scale.grid(row=row, column=1, pady=2, sticky='ew')
        self.fw_label = ttk.Label(parent, text=f"{self.fw_var.get():.2f}%")
        self.fw_label.grid(row=row, column=2, pady=2)
        row += 1

        # Separator
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1

        # Circuit parameters
        ttk.Label(parent, text="Equivalent Circuit Parameters",
                 font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=2, pady=(0, 10))
        row += 1

        # Stator resistance
        ttk.Label(parent, text="Stator Resistance (Ω):").grid(row=row, column=0, sticky='w', pady=2)
        self.rs_var = tk.DoubleVar(value=self.calculator.stator_resistance)
        rs_entry = ttk.Entry(parent, textvariable=self.rs_var, width=15)
        rs_entry.grid(row=row, column=1, pady=2)
        row += 1

        # Rotor resistance
        ttk.Label(parent, text="Rotor Resistance (Ω):").grid(row=row, column=0, sticky='w', pady=2)
        self.rr_var = tk.DoubleVar(value=self.calculator.rotor_resistance)
        rr_entry = ttk.Entry(parent, textvariable=self.rr_var, width=15)
        rr_entry.grid(row=row, column=1, pady=2)
        row += 1

        # Magnetizing reactance
        ttk.Label(parent, text="Mag. Reactance (Ω):").grid(row=row, column=0, sticky='w', pady=2)
        self.xm_var = tk.DoubleVar(value=self.calculator.magnetizing_reactance)
        xm_entry = ttk.Entry(parent, textvariable=self.xm_var, width=15)
        xm_entry.grid(row=row, column=1, pady=2)
        row += 1

        # Buttons
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1

        button_frame = ttk.Frame(parent)
        button_frame.grid(row=row, column=0, columnspan=3, pady=10)

        ttk.Button(button_frame, text="Calculate",
                  command=self.calculate_and_display).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Reset",
                  command=self.reset_parameters).pack(side='left', padx=5)

    def create_results_display(self, parent):
        """Create results display area"""
        # Create text widget with scrollbar
        text_frame = ttk.Frame(parent)
        text_frame.pack(fill='both', expand=True)

        scrollbar = ttk.Scrollbar(text_frame)
        scrollbar.pack(side='right', fill='y')

        self.results_text = tk.Text(text_frame, height=25, width=60,
                                   yscrollcommand=scrollbar.set,
                                   font=('Courier', 10))
        self.results_text.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=self.results_text.yview)

    def create_dynamic_simulation_tab(self):
        """Create dynamic simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Configure grid
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        # Simulation parameters
        params_frame = ttk.Frame(control_frame)
        params_frame.pack(fill='x', pady=5)

        ttk.Label(params_frame, text="Simulation Time (s):").pack(side='left', padx=5)
        self.sim_time_var = tk.DoubleVar(value=5.0)
        ttk.Entry(params_frame, textvariable=self.sim_time_var, width=10).pack(side='left', padx=5)

        ttk.Label(params_frame, text="Load Torque (Nm):").pack(side='left', padx=5)
        self.load_torque_var = tk.DoubleVar(value=50.0)
        ttk.Entry(params_frame, textvariable=self.load_torque_var, width=10).pack(side='left', padx=5)

        ttk.Label(params_frame, text="Inertia (kg·m²):").pack(side='left', padx=5)
        self.inertia_var = tk.DoubleVar(value=0.5)
        ttk.Entry(params_frame, textvariable=self.inertia_var, width=10).pack(side='left', padx=5)

        # ODE Solver selection
        solver_frame = ttk.Frame(control_frame)
        solver_frame.pack(fill='x', pady=5)

        ttk.Label(solver_frame, text="ODE Solver:").pack(side='left', padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(solver_frame, text="RK45 (Adaptive)",
                       variable=self.solver_var, value='RK45').pack(side='left', padx=5)
        ttk.Radiobutton(solver_frame, text="Euler (Fixed Step)",
                       variable=self.solver_var, value='Euler').pack(side='left', padx=5)

        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill='x', pady=5)

        self.start_btn = ttk.Button(button_frame, text="Start Simulation",
                                    command=self.start_simulation)
        self.start_btn.pack(side='left', padx=5)

        self.stop_btn = ttk.Button(button_frame, text="Stop",
                                   command=self.stop_simulation, state='disabled')
        self.stop_btn.pack(side='left', padx=5)

        ttk.Button(button_frame, text="Reset",
                  command=self.reset_simulation).pack(side='left', padx=5)

        # Status label
        self.status_label = ttk.Label(control_frame, text="Ready",
                                     foreground='green', font=('Arial', 10, 'bold'))
        self.status_label.pack(pady=5)

        # Plot area
        plot_frame = ttk.LabelFrame(tab, text="Simulation Results", padding=10)
        plot_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)

        # Create matplotlib figure
        self.sim_fig = Figure(figsize=(12, 8), dpi=100)
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, plot_frame)
        self.sim_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_torque_speed_tab(self):
        """Create torque-speed characteristics tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Torque-Speed Characteristics")

        # Configure grid
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        ttk.Button(control_frame, text="Generate Curve",
                  command=self.generate_torque_speed_curve).pack(side='left', padx=5)
        ttk.Button(control_frame, text="Export Data",
                  command=self.export_torque_speed_data).pack(side='left', padx=5)

        # Plot area
        plot_frame = ttk.LabelFrame(tab, text="Torque-Speed Curve", padding=10)
        plot_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)

        # Create matplotlib figure
        self.ts_fig = Figure(figsize=(10, 6), dpi=100)
        self.ts_canvas = FigureCanvasTkAgg(self.ts_fig, plot_frame)
        self.ts_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Generate initial curve
        self.generate_torque_speed_curve()

    def create_about_tab(self):
        """Create about tab with instructions"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="About")

        about_text = """
        ADVANCED INDUCTION MOTOR ANALYSIS TOOL
        =====================================

        Version 1.0

        Features:
        ---------
        1. Steady-State Analysis
           - Calculate rotor I²R losses, rotor input power, output torque
           - Power flow visualization
           - Comprehensive performance metrics

        2. Dynamic Simulation
           - Real-time motor starting simulation
           - Multiple ODE solvers (RK45 adaptive, Euler fixed-step)
           - Adjustable load conditions and inertia
           - Transient response analysis

        3. Torque-Speed Characteristics
           - Complete motor characteristic curves
           - Operating point identification
           - Performance envelope analysis

        Solution to Example Problem:
        ---------------------------
        Motor: 20 HP, 4-pole, 50 Hz, 3-phase
        Friction & Windage: 2% of output
        Full-load slip: 4%

        Results:
        (a) Rotor I²R Loss: 633.27 W
        (b) Rotor Input: 15,852.5 W
        (c) Output Torque: 98.94 Nm

        Instructions:
        ------------
        - Use sliders to adjust parameters in real-time
        - Click "Calculate" to update steady-state results
        - Run dynamic simulations to observe motor starting behavior
        - All plots auto-scale with window resizing

        Developed for Electrical Engineering Education and Analysis
        """

        text_widget = tk.Text(tab, wrap='word', font=('Courier', 10))
        text_widget.pack(fill='both', expand=True, padx=10, pady=10)
        text_widget.insert('1.0', about_text)
        text_widget.config(state='disabled')

    def on_slider_change(self, event=None):
        """Update labels when sliders change"""
        self.slip_label.config(text=f"{self.slip_var.get():.2f}%")
        self.fw_label.config(text=f"{self.fw_var.get():.2f}%")

    def update_calculator_params(self):
        """Update calculator with current GUI values"""
        try:
            self.calculator.power_hp = self.power_var.get()
            self.calculator.poles = self.poles_var.get()
            self.calculator.frequency = self.freq_var.get()
            self.calculator.voltage = self.voltage_var.get()
            self.calculator.rated_slip = self.slip_var.get() / 100.0
            self.calculator.friction_windage_percent = self.fw_var.get()
            self.calculator.stator_resistance = self.rs_var.get()
            self.calculator.rotor_resistance = self.rr_var.get()
            self.calculator.magnetizing_reactance = self.xm_var.get()
            return True
        except tk.TclError:
            messagebox.showerror("Input Error", "Please enter valid numeric values")
            return False

    def calculate_and_display(self):
        """Perform calculation and display results"""
        if not self.update_calculator_params():
            return

        results = self.calculator.calculate_steady_state()

        # Display results
        self.results_text.config(state='normal')
        self.results_text.delete('1.0', 'end')

        output = f"""
{'='*60}
INDUCTION MOTOR STEADY-STATE ANALYSIS
{'='*60}

MOTOR SPECIFICATIONS:
{'─'*60}
Power Rating         : {results['power_output_hp']:.2f} HP ({results['power_output_w']:.2f} W)
Poles                : {self.calculator.poles}
Frequency            : {self.calculator.frequency:.2f} Hz
Synchronous Speed    : {results['synchronous_speed_rpm']:.2f} RPM
Full-Load Slip       : {results['slip_percent']:.2f} %
Rotor Speed          : {results['rotor_speed_rpm']:.2f} RPM

FULL-LOAD PERFORMANCE:
{'─'*60}
(a) Rotor I²R Loss   : {results['rotor_copper_loss_w']:.2f} W
(b) Rotor Input      : {results['rotor_input_w']:.2f} W
(c) Output Torque    : {results['output_torque_nm']:.2f} Nm

DETAILED ANALYSIS:
{'─'*60}
Air Gap Power        : {results['air_gap_power_w']:.2f} W
Mechanical Power Dev.: {results['mechanical_power_dev_w']:.2f} W
Developed Torque     : {results['developed_torque_nm']:.2f} Nm
Shaft Output Power   : {results['power_output_w']:.2f} W
Shaft Output Torque  : {results['output_torque_nm']:.2f} Nm

LOSSES:
{'─'*60}
Stator Copper Loss   : {results['stator_copper_loss_w']:.2f} W
Rotor Copper Loss    : {results['rotor_copper_loss_w']:.2f} W
Friction & Windage   : {results['friction_windage_w']:.2f} W
Total Losses         : {results['total_losses_w']:.2f} W

EFFICIENCY:
{'─'*60}
Efficiency           : {results['efficiency_percent']:.2f} %

{'='*60}
"""

        self.results_text.insert('1.0', output)
        self.results_text.config(state='disabled')

        # Update power flow diagram
        self.plot_power_flow(results)

    def plot_power_flow(self, results):
        """Plot power flow diagram"""
        self.power_flow_fig.clear()
        ax = self.power_flow_fig.add_subplot(111)

        # Data for power flow
        stages = ['Input\nPower', 'Air Gap\nPower', 'Mechanical\nPower', 'Output\nPower']
        powers = [
            results['air_gap_power_w'] + results['stator_copper_loss_w'],
            results['air_gap_power_w'],
            results['mechanical_power_dev_w'],
            results['power_output_w']
        ]

        losses = [
            results['stator_copper_loss_w'],
            results['rotor_copper_loss_w'],
            results['friction_windage_w']
        ]

        loss_labels = [
            f"Stator Loss\n{results['stator_copper_loss_w']:.0f} W",
            f"Rotor Loss\n{results['rotor_copper_loss_w']:.0f} W",
            f"Mech. Loss\n{results['friction_windage_w']:.0f} W"
        ]

        # Plot bars
        x_pos = np.arange(len(stages))
        bars = ax.bar(x_pos, powers, color=['#3498db', '#2ecc71', '#f39c12', '#e74c3c'],
                     alpha=0.7, edgecolor='black', linewidth=2)

        # Add value labels on bars
        for i, (bar, power) in enumerate(zip(bars, powers)):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{power:.0f} W',
                   ha='center', va='bottom', fontweight='bold', fontsize=9)

        # Add loss arrows
        for i in range(len(losses)):
            ax.annotate('', xy=(i+0.5, powers[i]/2), xytext=(i+0.5, -powers[i]*0.15),
                       arrowprops=dict(arrowstyle='->', lw=2, color='red'))
            ax.text(i+0.5, -powers[i]*0.2, loss_labels[i],
                   ha='center', va='top', fontsize=8, color='red', fontweight='bold')

        ax.set_xticks(x_pos)
        ax.set_xticklabels(stages, fontsize=10, fontweight='bold')
        ax.set_ylabel('Power (W)', fontsize=11, fontweight='bold')
        ax.set_title('Power Flow Diagram', fontsize=12, fontweight='bold', pad=20)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_ylim(bottom=-max(powers)*0.3)

        self.power_flow_fig.tight_layout()
        self.power_flow_canvas.draw()

    def start_simulation(self):
        """Start dynamic simulation"""
        if not self.update_calculator_params():
            return

        self.simulation_running = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.status_label.config(text="Simulating...", foreground='orange')

        # Update calculator parameters
        self.calculator.inertia = self.inertia_var.get()

        # Define load torque function
        load_torque_value = self.load_torque_var.get()
        def torque_load_func(t, omega):
            # Step load at t=0.5s
            if t < 0.5:
                return 0
            else:
                return load_torque_value

        # Time span
        t_span = (0, self.sim_time_var.get())
        t_eval = np.linspace(0, self.sim_time_var.get(), 1000)

        # Initial conditions [omega, theta]
        y0 = [0, 0]

        # Solve ODE
        if self.solver_var.get() == 'RK45':
            sol = solve_ivp(
                lambda t, y: self.calculator.motor_dynamics(t, y, torque_load_func),
                t_span, y0, method='RK45', t_eval=t_eval, max_step=0.01
            )
        else:  # Euler method
            sol = self.euler_solve(
                lambda t, y: self.calculator.motor_dynamics(t, y, torque_load_func),
                t_span, y0, t_eval
            )

        self.simulation_data = {
            't': sol.t if hasattr(sol, 't') else sol['t'],
            'omega': sol.y[0] if hasattr(sol, 'y') else sol['y'][0],
            'theta': sol.y[1] if hasattr(sol, 'y') else sol['y'][1],
            'load_torque_func': torque_load_func
        }

        # Plot results
        self.plot_simulation_results()

        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        self.status_label.config(text="Simulation Complete", foreground='green')

    def euler_solve(self, func, t_span, y0, t_eval):
        """Simple Euler method ODE solver"""
        t = t_eval
        y = np.zeros((len(y0), len(t)))
        y[:, 0] = y0

        for i in range(len(t) - 1):
            dt = t[i+1] - t[i]
            dydt = func(t[i], y[:, i])
            y[:, i+1] = y[:, i] + np.array(dydt) * dt

        return {'t': t, 'y': y}

    def plot_simulation_results(self):
        """Plot simulation results"""
        if self.simulation_data is None:
            return

        self.sim_fig.clear()

        t = self.simulation_data['t']
        omega = self.simulation_data['omega']
        speed_rpm = omega * 60 / (2 * np.pi)

        # Calculate electromagnetic torque and load torque
        ns_rpm = 120.0 * self.calculator.frequency / self.calculator.poles
        torque_em = []
        torque_load = []

        for i, (ti, omegai) in enumerate(zip(t, omega)):
            current_rpm = omegai * 60.0 / (2.0 * np.pi)
            slip = (ns_rpm - current_rpm) / ns_rpm if ns_rpm > 0 else 0

            if abs(slip) < 1e-6:
                torque_em.append(0)
            else:
                r2_s = self.calculator.rotor_resistance / slip
                v_th = self.calculator.voltage / np.sqrt(3)
                z_total = np.sqrt((r2_s)**2 + self.calculator.rotor_leakage_reactance**2)
                i2 = v_th / z_total if z_total > 0 else 0
                ns_rad_s = 2.0 * np.pi * ns_rpm / 60.0
                T_em = (3 * i2**2 * r2_s) / ns_rad_s if ns_rad_s > 0 else 0
                torque_em.append(T_em)

            torque_load.append(self.simulation_data['load_torque_func'](ti, omegai))

        # Create subplots
        ax1 = self.sim_fig.add_subplot(311)
        ax2 = self.sim_fig.add_subplot(312)
        ax3 = self.sim_fig.add_subplot(313)

        # Speed vs time
        ax1.plot(t, speed_rpm, 'b-', linewidth=2, label='Rotor Speed')
        ax1.axhline(y=ns_rpm, color='r', linestyle='--', label='Synchronous Speed')
        ax1.set_ylabel('Speed (RPM)', fontweight='bold')
        ax1.set_title('Motor Starting Characteristics', fontweight='bold', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')

        # Torque vs time
        ax2.plot(t, torque_em, 'g-', linewidth=2, label='Electromagnetic Torque')
        ax2.plot(t, torque_load, 'r--', linewidth=2, label='Load Torque')
        ax2.set_ylabel('Torque (Nm)', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best')

        # Slip vs time
        slip_percent = [(ns_rpm - s) / ns_rpm * 100 if ns_rpm > 0 else 0 for s in speed_rpm]
        ax3.plot(t, slip_percent, 'm-', linewidth=2)
        ax3.set_xlabel('Time (s)', fontweight='bold')
        ax3.set_ylabel('Slip (%)', fontweight='bold')
        ax3.grid(True, alpha=0.3)

        self.sim_fig.tight_layout()
        self.sim_canvas.draw()

    def stop_simulation(self):
        """Stop running simulation"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        self.status_label.config(text="Stopped", foreground='red')

    def reset_simulation(self):
        """Reset simulation"""
        self.simulation_data = None
        self.sim_fig.clear()
        self.sim_canvas.draw()
        self.status_label.config(text="Ready", foreground='green')

    def generate_torque_speed_curve(self):
        """Generate and plot torque-speed characteristics"""
        if not self.update_calculator_params():
            return

        speed, torque, slip = self.calculator.calculate_torque_speed_curve()

        self.ts_fig.clear()

        # Create two subplots
        ax1 = self.ts_fig.add_subplot(211)
        ax2 = self.ts_fig.add_subplot(212)

        # Torque vs Speed
        ax1.plot(speed, torque, 'b-', linewidth=2)
        ax1.set_xlabel('Speed (RPM)', fontweight='bold')
        ax1.set_ylabel('Torque (Nm)', fontweight='bold')
        ax1.set_title('Torque-Speed Characteristics', fontweight='bold', fontsize=12)
        ax1.grid(True, alpha=0.3)

        # Mark rated point
        results = self.calculator.calculate_steady_state()
        ax1.plot(results['rotor_speed_rpm'], results['developed_torque_nm'],
                'ro', markersize=10, label='Rated Operating Point')
        ax1.legend(loc='best')

        # Add regions
        ns_rpm = 120.0 * self.calculator.frequency / self.calculator.poles
        ax1.axvline(x=ns_rpm, color='r', linestyle='--', alpha=0.5, label='Synchronous Speed')
        ax1.fill_between([0, ns_rpm], 0, max(torque)*1.1, alpha=0.1, color='green',
                         label='Motoring Region')

        # Torque vs Slip
        slip_percent = slip * 100
        ax2.plot(slip_percent, torque, 'g-', linewidth=2)
        ax2.set_xlabel('Slip (%)', fontweight='bold')
        ax2.set_ylabel('Torque (Nm)', fontweight='bold')
        ax2.set_title('Torque-Slip Characteristics', fontweight='bold', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.axvline(x=self.calculator.rated_slip*100, color='r', linestyle='--',
                   alpha=0.5, label='Rated Slip')
        ax2.legend(loc='best')

        self.ts_fig.tight_layout()
        self.ts_canvas.draw()

    def export_torque_speed_data(self):
        """Export torque-speed data to file"""
        speed, torque, slip = self.calculator.calculate_torque_speed_curve()

        try:
            with open('torque_speed_data.csv', 'w') as f:
                f.write('Speed (RPM),Torque (Nm),Slip (%)\n')
                for s, t, sl in zip(speed, torque, slip):
                    f.write(f'{s:.2f},{t:.2f},{sl*100:.2f}\n')
            messagebox.showinfo("Export Successful",
                              "Data exported to torque_speed_data.csv")
        except Exception as e:
            messagebox.showerror("Export Failed", str(e))

    def reset_parameters(self):
        """Reset all parameters to default values"""
        self.calculator.reset_parameters()

        self.power_var.set(self.calculator.power_hp)
        self.poles_var.set(self.calculator.poles)
        self.freq_var.set(self.calculator.frequency)
        self.voltage_var.set(self.calculator.voltage)
        self.slip_var.set(self.calculator.rated_slip * 100)
        self.fw_var.set(self.calculator.friction_windage_percent)
        self.rs_var.set(self.calculator.stator_resistance)
        self.rr_var.set(self.calculator.rotor_resistance)
        self.xm_var.set(self.calculator.magnetizing_reactance)

        self.calculate_and_display()

    def on_window_resize(self, event=None):
        """Handle window resize events for autoscaling"""
        # The matplotlib tight_layout() handles most of the autoscaling
        # This method can be extended for additional resize handling
        pass


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = InductionMotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
