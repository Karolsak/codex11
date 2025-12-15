"""
Advanced Synchronous Motor Analysis Tool
Comprehensive Python + Tkinter GUI for electrical engineering calculations and dynamic simulation  
Includes solution for synchronous motor torque angle analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import math


class SynchronousMotorCalculator:
    """Core calculation engine for synchronous motor analysis"""

    def __init__(self):
        self.reset_parameters()

    def reset_parameters(self):
        """Reset all motor parameters to default values"""
        self.power_rating = 100.0  # kW
        self.poles = 4
        self.frequency = 50.0  # Hz
        self.voltage = 400.0  # V (line-to-line)
        self.excitation = 1.0  # per unit
        self.power_factor = 0.8
        self.delta_initial = 30.0  # degrees (electrical)
        self.stator_resistance = 0.0  # Negligible
        self.synchronous_reactance = 2.0  # ohms per phase
        self.inertia = 10.0  # kg·m²
        self.load_torque = 0.0  # Nm
        self.friction_coeff = 0.5
        
    def calculate_torque_angle_scenario1(self):
        """Scenario (i): Load torque & voltage constant, Ef & f raised 10%"""
        delta_rad = math.radians(self.delta_initial)
        sin_delta_new = 1.1 * math.sin(delta_rad)
        
        if sin_delta_new > 1.0:
            return None, "Motor unstable - torque angle exceeds 90°"
        
        delta_new_deg = math.degrees(math.asin(sin_delta_new))
        change = delta_new_deg - self.delta_initial
        
        return delta_new_deg, f"Scenario (i): Torque angle increases to {delta_new_deg:.2f}° (change: +{change:.2f}°)"
    
    def calculate_torque_angle_scenario2(self):
        """Scenario (ii): Load power & voltage constant, Ef & f reduced 10%"""
        delta_new_deg = self.delta_initial
        return delta_new_deg, f"Scenario (ii): Torque angle remains {delta_new_deg:.2f}° (change: 0.00°)"
    
    def calculate_torque_angle_scenario3(self):
        """Scenario (iii): Load torque & Ef constant, V & f raised 10%"""
        delta_rad = math.radians(self.delta_initial)
        sin_delta_new = 1.1 * math.sin(delta_rad)
        
        if sin_delta_new > 1.0:
            return None, "Motor unstable - torque angle exceeds 90°"
        
        delta_new_deg = math.degrees(math.asin(sin_delta_new))
        change = delta_new_deg - self.delta_initial
        
        return delta_new_deg, f"Scenario (iii): Torque angle increases to {delta_new_deg:.2f}° (change: +{change:.2f}°)"
    
    def calculate_power_angle_curve(self):
        """Calculate power-angle curve"""
        angles = np.linspace(0, 180, 200)
        V_ph = self.voltage / math.sqrt(3)
        Ef = self.excitation * V_ph * 1.5
        Xs = self.synchronous_reactance
        
        powers = []
        for angle in angles:
            delta_rad = math.radians(angle)
            P_ph = (V_ph * Ef * math.sin(delta_rad)) / Xs
            powers.append(P_ph * 3 / 1000)
        
        return angles, np.array(powers)
    
    def motor_dynamics_sync(self, t, y, torque_load_func):
        """Differential equations for synchronous motor dynamics"""
        delta, omega = y[0], y[1]
        
        omega_s = 2.0 * math.pi * self.frequency * 2 / self.poles
        V_ph = self.voltage / math.sqrt(3)
        Ef = self.excitation * V_ph * 1.5
        
        T_em = (3 * V_ph * Ef * math.sin(delta)) / (omega_s * self.synchronous_reactance)
        T_load = torque_load_func(t, omega)
        T_friction = self.friction_coeff * omega
        
        domega_dt = (T_em - T_load - T_friction) / self.inertia
        ddelta_dt = (self.poles / 2) * (omega - omega_s)
        
        return [ddelta_dt, domega_dt]
    
    def calculate_steady_state_performance(self):
        """Calculate steady-state performance"""
        delta_rad = math.radians(self.delta_initial)
        V_ph = self.voltage / math.sqrt(3)
        Ef = self.excitation * V_ph * 1.5
        ns_rpm = 120 * self.frequency / self.poles
        omega_s = 2 * math.pi * self.frequency * 2 / self.poles
        
        P_ph = (V_ph * Ef * math.sin(delta_rad)) / self.synchronous_reactance
        P_total = 3 * P_ph / 1000
        T_em = P_total * 1000 / omega_s
        I_a = math.sqrt((V_ph - Ef*math.cos(delta_rad))**2 + (Ef*math.sin(delta_rad))**2) / self.synchronous_reactance
        phi = math.atan2(Ef*math.sin(delta_rad), V_ph - Ef*math.cos(delta_rad))
        pf = math.cos(phi)
        
        return {
            'power_kw': P_total, 'torque_nm': T_em, 'current_a': I_a,
            'power_factor': pf, 'power_angle_deg': self.delta_initial,
            'sync_speed_rpm': ns_rpm, 'frequency_hz': self.frequency,
            'voltage_v': self.voltage, 'excitation_pu': self.excitation
        }


class SynchronousMotorGUI:
    """Advanced Tkinter GUI for Synchronous Motor Analysis"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Synchronous Motor Analysis Tool")
        self.root.geometry("1400x900")
        
        self.calculator = SynchronousMotorCalculator()
        self.simulation_running = False
        self.simulation_data = None
        
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        
        self.create_main_layout()
        self.root.bind('<Configure>', self.on_window_resize)
        self.calculate_torque_angles()
    
    def create_main_layout(self):
        """Create the main application layout"""
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        
        self.create_torque_angle_tab()
        self.create_power_angle_tab()
        self.create_dynamic_simulation_tab()
        self.create_comprehensive_analysis_tab()
        self.create_about_tab()
    
    def create_torque_angle_tab(self):
        """Create torque angle analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Torque Angle Analysis")
        
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=2)
        
        left_frame = ttk.LabelFrame(tab, text="Motor Parameters", padding=10)
        left_frame.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=5, pady=5)
        self.create_parameter_inputs(left_frame)
        
        right_top_frame = ttk.LabelFrame(tab, text="Problem Statement", padding=10)
        right_top_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)
        
        problem_text = """
PROBLEM: Synchronous Motor Torque Angle Analysis

Given:
- Full load torque angle: 30° (electrical)
- Stator resistance: Negligible
- Rated voltage and frequency

Determine torque angle changes for:
(i) Load torque and voltage constant, excitation and frequency raised by 10%
(ii) Load power and voltage constant, excitation and frequency reduced by 10%
(iii) Load torque and excitation constant, voltage and frequency raised by 10%
"""
        problem_label = tk.Text(right_top_frame, height=12, wrap='word', font=('Courier', 10))
        problem_label.insert('1.0', problem_text)
        problem_label.config(state='disabled')
        problem_label.pack(fill='both', expand=True)
        
        right_bottom_frame = ttk.LabelFrame(tab, text="Solution Results", padding=10)
        right_bottom_frame.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)
        right_bottom_frame.rowconfigure(0, weight=1)
        right_bottom_frame.columnconfigure(0, weight=1)
        
        self.results_text = tk.Text(right_bottom_frame, wrap='word', font=('Courier', 10))
        scrollbar = ttk.Scrollbar(right_bottom_frame, command=self.results_text.yview)
        self.results_text.config(yscrollcommand=scrollbar.set)
        self.results_text.grid(row=0, column=0, sticky='nsew')
        scrollbar.grid(row=0, column=1, sticky='ns')
    
    def create_parameter_inputs(self, parent):
        """Create input fields for motor parameters"""
        row = 0
        
        ttk.Label(parent, text="Nameplate Data", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=3, pady=(0, 10))
        row += 1
        
        ttk.Label(parent, text="Power Rating (kW):").grid(row=row, column=0, sticky='w', pady=2)
        self.power_var = tk.DoubleVar(value=self.calculator.power_rating)
        ttk.Entry(parent, textvariable=self.power_var, width=15).grid(row=row, column=1, pady=2)
        row += 1
        
        ttk.Label(parent, text="Poles:").grid(row=row, column=0, sticky='w', pady=2)
        self.poles_var = tk.IntVar(value=self.calculator.poles)
        ttk.Combobox(parent, textvariable=self.poles_var,
                    values=[2, 4, 6, 8, 10, 12], width=13, state='readonly').grid(row=row, column=1, pady=2)
        row += 1
        
        ttk.Label(parent, text="Frequency (Hz):").grid(row=row, column=0, sticky='w', pady=2)
        self.freq_var = tk.DoubleVar(value=self.calculator.frequency)
        ttk.Entry(parent, textvariable=self.freq_var, width=15).grid(row=row, column=1, pady=2)
        row += 1
        
        ttk.Label(parent, text="Voltage (V):").grid(row=row, column=0, sticky='w', pady=2)
        self.voltage_var = tk.DoubleVar(value=self.calculator.voltage)
        ttk.Entry(parent, textvariable=self.voltage_var, width=15).grid(row=row, column=1, pady=2)
        row += 1
        
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1
        
        ttk.Label(parent, text="Initial Torque Angle (°):", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, sticky='w', pady=2)
        row += 1
        
        self.delta_var = tk.DoubleVar(value=self.calculator.delta_initial)
        delta_scale = ttk.Scale(parent, from_=0, to=90, variable=self.delta_var,
                               orient='horizontal', command=self.on_slider_change)
        delta_scale.grid(row=row, column=0, columnspan=2, pady=2, sticky='ew', padx=(0, 5))
        self.delta_label = ttk.Label(parent, text=f"{self.delta_var.get():.1f}°")
        self.delta_label.grid(row=row, column=2, pady=2)
        row += 1
        
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1
        
        ttk.Label(parent, text="Excitation (pu):", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, sticky='w', pady=2)
        row += 1
        
        self.excitation_var = tk.DoubleVar(value=self.calculator.excitation)
        exc_scale = ttk.Scale(parent, from_=0.5, to=2.0, variable=self.excitation_var,
                             orient='horizontal', command=self.on_slider_change)
        exc_scale.grid(row=row, column=0, columnspan=2, pady=2, sticky='ew', padx=(0, 5))
        self.exc_label = ttk.Label(parent, text=f"{self.excitation_var.get():.2f} pu")
        self.exc_label.grid(row=row, column=2, pady=2)
        row += 1
        
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1
        
        ttk.Label(parent, text="Sync. Reactance (Ω):").grid(row=row, column=0, sticky='w', pady=2)
        self.xs_var = tk.DoubleVar(value=self.calculator.synchronous_reactance)
        ttk.Entry(parent, textvariable=self.xs_var, width=15).grid(row=row, column=1, pady=2)
        row += 1
        
        ttk.Separator(parent, orient='horizontal').grid(row=row, column=0, columnspan=3,
                                                        sticky='ew', pady=10)
        row += 1
        
        button_frame = ttk.Frame(parent)
        button_frame.grid(row=row, column=0, columnspan=3, pady=10)
        
        ttk.Button(button_frame, text="Calculate",
                  command=self.calculate_torque_angles).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Reset",
                  command=self.reset_parameters).pack(side='left', padx=5)

    def create_power_angle_tab(self):
        """Create power-angle characteristics tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Power-Angle Curve")
        
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)
        
        control_frame = ttk.LabelFrame(tab, text="Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)
        
        ttk.Button(control_frame, text="Generate Curve",
                  command=self.generate_power_angle_curve).pack(side='left', padx=5)
        
        plot_frame = ttk.LabelFrame(tab, text="Power-Angle Characteristics", padding=10)
        plot_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)
        
        self.pa_fig = Figure(figsize=(10, 6), dpi=100)
        self.pa_canvas = FigureCanvasTkAgg(self.pa_fig, plot_frame)
        self.pa_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        
        self.generate_power_angle_curve()
    
    def create_dynamic_simulation_tab(self):
        """Create dynamic simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")
        
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)
        
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)
        
        params_frame = ttk.Frame(control_frame)
        params_frame.pack(fill='x', pady=5)
        
        ttk.Label(params_frame, text="Simulation Time (s):").pack(side='left', padx=5)
        self.sim_time_var = tk.DoubleVar(value=5.0)
        ttk.Entry(params_frame, textvariable=self.sim_time_var, width=10).pack(side='left', padx=5)
        
        ttk.Label(params_frame, text="Load Torque (Nm):").pack(side='left', padx=5)
        self.load_torque_var = tk.DoubleVar(value=200.0)
        ttk.Entry(params_frame, textvariable=self.load_torque_var, width=10).pack(side='left', padx=5)
        
        ttk.Label(params_frame, text="Inertia (kg·m²):").pack(side='left', padx=5)
        self.inertia_var = tk.DoubleVar(value=10.0)
        ttk.Entry(params_frame, textvariable=self.inertia_var, width=10).pack(side='left', padx=5)
        
        solver_frame = ttk.Frame(control_frame)
        solver_frame.pack(fill='x', pady=5)
        
        ttk.Label(solver_frame, text="ODE Solver:").pack(side='left', padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(solver_frame, text="RK45 (Adaptive)",
                       variable=self.solver_var, value='RK45').pack(side='left', padx=5)
        ttk.Radiobutton(solver_frame, text="Euler (Fixed Step)",
                       variable=self.solver_var, value='Euler').pack(side='left', padx=5)
        
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
        
        self.status_label = ttk.Label(control_frame, text="Ready",
                                     foreground='green', font=('Arial', 10, 'bold'))
        self.status_label.pack(pady=5)
        
        plot_frame = ttk.LabelFrame(tab, text="Simulation Results", padding=10)
        plot_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)
        
        self.sim_fig = Figure(figsize=(12, 8), dpi=100)
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, plot_frame)
        self.sim_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
    
    def create_comprehensive_analysis_tab(self):
        """Create comprehensive analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Comprehensive Analysis")
        
        tab.rowconfigure(1, weight=1)
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        
        left_frame = ttk.LabelFrame(tab, text="Steady-State Performance", padding=10)
        left_frame.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=5, pady=5)
        left_frame.rowconfigure(0, weight=1)
        left_frame.columnconfigure(0, weight=1)
        
        self.perf_text = tk.Text(left_frame, wrap='word', font=('Courier', 9))
        perf_scrollbar = ttk.Scrollbar(left_frame, command=self.perf_text.yview)
        self.perf_text.config(yscrollcommand=perf_scrollbar.set)
        self.perf_text.grid(row=0, column=0, sticky='nsew')
        perf_scrollbar.grid(row=0, column=1, sticky='ns')
        
        right_top_frame = ttk.LabelFrame(tab, text="Phasor Diagram", padding=10)
        right_top_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)
        right_top_frame.rowconfigure(0, weight=1)
        right_top_frame.columnconfigure(0, weight=1)
        
        self.phasor_fig = Figure(figsize=(6, 6), dpi=100)
        self.phasor_canvas = FigureCanvasTkAgg(self.phasor_fig, right_top_frame)
        self.phasor_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        
        right_bottom_frame = ttk.LabelFrame(tab, text="V-Curves", padding=10)
        right_bottom_frame.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)
        right_bottom_frame.rowconfigure(0, weight=1)
        right_bottom_frame.columnconfigure(0, weight=1)
        
        self.vcurve_fig = Figure(figsize=(6, 4), dpi=100)
        self.vcurve_canvas = FigureCanvasTkAgg(self.vcurve_fig, right_bottom_frame)
        self.vcurve_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        
        calc_frame = ttk.Frame(tab)
        calc_frame.grid(row=2, column=0, columnspan=2, pady=5)
        ttk.Button(calc_frame, text="Perform Comprehensive Analysis",
                  command=self.perform_comprehensive_analysis).pack()
        
        self.perform_comprehensive_analysis()

    def create_about_tab(self):
        """Create about tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="About")
        
        about_text = """
ADVANCED SYNCHRONOUS MOTOR ANALYSIS TOOL
========================================

Version 1.0

Features:
---------
1. Torque Angle Analysis (Problem Solution)
   - Effect of frequency and excitation changes on torque angle
   - Three scenarios analyzed with mathematical derivations

2. Power-Angle Characteristics
   - Complete power-angle curve visualization
   - Stability limit identification

3. Dynamic Simulation
   - Real-time motor transient simulation
   - Multiple ODE solvers (RK45 adaptive, Euler fixed-step)
   - Power angle and speed dynamics

4. Comprehensive Analysis
   - Steady-state performance metrics
   - Phasor diagrams
   - V-curves (excitation characteristics)

SOLUTION TO GIVEN PROBLEM:
-------------------------
Initial Conditions:
- Full load torque angle: 30° (electrical)
- Stator resistance: Negligible

Analysis Results:
(i) Load torque & voltage constant, Ef & f raised 10%:
    Torque angle INCREASES to ~33.4°

(ii) Load power & voltage constant, Ef & f reduced 10%:
     Torque angle REMAINS at 30°

(iii) Load torque & Ef constant, V & f raised 10%:
      Torque angle INCREASES to ~33.4°

Instructions:
------------
- Adjust parameters using sliders for real-time updates
- Click "Calculate" to solve torque angle scenarios
- Run dynamic simulations to observe transient behavior
- All plots automatically scale with window resizing

Developed for Electrical Engineering Education and Analysis
"""
        
        text_widget = tk.Text(tab, wrap='word', font=('Courier', 9))
        text_widget.pack(fill='both', expand=True, padx=10, pady=10)
        text_widget.insert('1.0', about_text)
        text_widget.config(state='disabled')
    
    def on_slider_change(self, event=None):
        """Update labels when sliders change"""
        self.delta_label.config(text=f"{self.delta_var.get():.1f}°")
        self.exc_label.config(text=f"{self.excitation_var.get():.2f} pu")
    
    def update_calculator_params(self):
        """Update calculator with current GUI values"""
        try:
            self.calculator.power_rating = self.power_var.get()
            self.calculator.poles = self.poles_var.get()
            self.calculator.frequency = self.freq_var.get()
            self.calculator.voltage = self.voltage_var.get()
            self.calculator.delta_initial = self.delta_var.get()
            self.calculator.excitation = self.excitation_var.get()
            self.calculator.synchronous_reactance = self.xs_var.get()
            return True
        except tk.TclError:
            messagebox.showerror("Input Error", "Please enter valid numeric values")
            return False
    
    def calculate_torque_angles(self):
        """Calculate and display torque angle changes"""
        if not self.update_calculator_params():
            return
        
        delta1, msg1 = self.calculator.calculate_torque_angle_scenario1()
        delta2, msg2 = self.calculator.calculate_torque_angle_scenario2()
        delta3, msg3 = self.calculator.calculate_torque_angle_scenario3()
        
        perf = self.calculator.calculate_steady_state_performance()
        
        self.results_text.config(state='normal')
        self.results_text.delete('1.0', 'end')
        
        output = f"""
{'='*70}
SYNCHRONOUS MOTOR TORQUE ANGLE ANALYSIS - SOLUTION
{'='*70}

INITIAL CONDITIONS:
{'─'*70}
Power Rating         : {self.calculator.power_rating:.2f} kW
Poles                : {self.calculator.poles}
Frequency            : {self.calculator.frequency:.2f} Hz
Voltage (L-L)        : {self.calculator.voltage:.2f} V
Synchronous Speed    : {perf['sync_speed_rpm']:.2f} RPM
Initial Torque Angle : {self.calculator.delta_initial:.2f}° (electrical)
Excitation           : {self.calculator.excitation:.2f} pu
Synchronous Reactance: {self.calculator.synchronous_reactance:.2f} Ω

STEADY-STATE PERFORMANCE:
{'─'*70}
Power Output         : {perf['power_kw']:.2f} kW
Electromagnetic Torque: {perf['torque_nm']:.2f} Nm
Armature Current     : {perf['current_a']:.2f} A
Power Factor         : {perf['power_factor']:.3f}

SCENARIO ANALYSIS:
{'='*70}

(i) LOAD TORQUE & VOLTAGE CONSTANT, EXCITATION & FREQUENCY +10%
{'─'*70}
Analysis:
  - Torque: T ∝ (V·Ef·sin δ)/(Xs·ωs)
  - Xs ∝ f, ωs ∝ f
  - For constant T: sin δ' = 1.1·sin δ

Result: {msg1}

{'─'*70}

(ii) LOAD POWER & VOLTAGE CONSTANT, EXCITATION & FREQUENCY -10%
{'─'*70}
Analysis:
  - Power: P = (V·Ef·sin δ)/Xs
  - For constant P: sin δ' = sin δ

Result: {msg2}

{'─'*70}

(iii) LOAD TORQUE & EXCITATION CONSTANT, VOLTAGE & FREQUENCY +10%
{'─'*70}
Analysis:
  - For constant T: sin δ' = 1.1·sin δ

Result: {msg3}

{'='*70}

SUMMARY:
{'─'*70}
Initial Torque Angle: {self.calculator.delta_initial:.2f}°
Scenario (i):         {delta1:.2f}° (Change: +{delta1-self.calculator.delta_initial:.2f}°)
Scenario (ii):        {delta2:.2f}° (Change: {delta2-self.calculator.delta_initial:.2f}°)
Scenario (iii):       {delta3:.2f}° (Change: +{delta3-self.calculator.delta_initial:.2f}°)

{'='*70}
"""
        
        self.results_text.insert('1.0', output)
        self.results_text.config(state='disabled')

    def generate_power_angle_curve(self):
        """Generate and plot power-angle curve"""
        if not self.update_calculator_params():
            return
        
        angles, powers = self.calculator.calculate_power_angle_curve()
        
        self.pa_fig.clear()
        ax = self.pa_fig.add_subplot(111)
        
        ax.plot(angles, powers, 'b-', linewidth=2, label='Power-Angle Curve')
        
        current_angle = self.calculator.delta_initial
        perf = self.calculator.calculate_steady_state_performance()
        ax.plot(current_angle, perf['power_kw'], 'ro', markersize=10,
               label=f'Operating Point ({current_angle:.1f}°, {perf["power_kw"]:.1f} kW)')
        
        max_power_idx = np.argmax(powers)
        max_angle = angles[max_power_idx]
        max_power = powers[max_power_idx]
        ax.axvline(x=max_angle, color='r', linestyle='--', alpha=0.5,
                  label=f'Stability Limit ({max_angle:.1f}°)')
        
        ax.fill_between(angles[:max_power_idx+1], 0, max_power*1.1,
                       alpha=0.1, color='green', label='Stable Region')
        ax.fill_between(angles[max_power_idx:], 0, max_power*1.1,
                       alpha=0.1, color='red', label='Unstable Region')
        
        ax.set_xlabel('Power Angle δ (degrees)', fontweight='bold', fontsize=11)
        ax.set_ylabel('Power (kW)', fontweight='bold', fontsize=11)
        ax.set_title('Power-Angle Characteristics', fontweight='bold', fontsize=12)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(loc='best', fontsize=9)
        ax.set_xlim(0, 180)
        ax.set_ylim(0, max_power*1.1)
        
        self.pa_fig.tight_layout()
        self.pa_canvas.draw()
    
    def start_simulation(self):
        """Start dynamic simulation"""
        if not self.update_calculator_params():
            return
        
        self.simulation_running = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.status_label.config(text="Simulating...", foreground='orange')
        
        self.calculator.inertia = self.inertia_var.get()
        
        load_torque_value = self.load_torque_var.get()
        def torque_load_func(t, omega):
            return load_torque_value if t >= 1.0 else 0
        
        t_span = (0, self.sim_time_var.get())
        t_eval = np.linspace(0, self.sim_time_var.get(), 1000)
        
        omega_s = 2.0 * math.pi * self.calculator.frequency * 2 / self.calculator.poles
        delta_initial_rad = math.radians(self.calculator.delta_initial)
        y0 = [delta_initial_rad, omega_s]
        
        try:
            if self.solver_var.get() == 'RK45':
                sol = solve_ivp(
                    lambda t, y: self.calculator.motor_dynamics_sync(t, y, torque_load_func),
                    t_span, y0, method='RK45', t_eval=t_eval, max_step=0.01
                )
            else:
                sol = self.euler_solve(
                    lambda t, y: self.calculator.motor_dynamics_sync(t, y, torque_load_func),
                    t_span, y0, t_eval
                )
            
            self.simulation_data = {
                't': sol.t if hasattr(sol, 't') else sol['t'],
                'delta': sol.y[0] if hasattr(sol, 'y') else sol['y'][0],
                'omega': sol.y[1] if hasattr(sol, 'y') else sol['y'][1],
                'load_torque_func': torque_load_func
            }
            
            self.plot_simulation_results()
            self.status_label.config(text="Simulation Complete", foreground='green')
        except Exception as e:
            messagebox.showerror("Simulation Error", str(e))
            self.status_label.config(text="Simulation Failed", foreground='red')
        
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
    
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
        delta_rad = self.simulation_data['delta']
        omega = self.simulation_data['omega']
        
        delta_deg = np.degrees(delta_rad)
        speed_rpm = omega * 60 / (2 * math.pi)
        
        omega_s = 2.0 * math.pi * self.calculator.frequency * 2 / self.calculator.poles
        ns_rpm = omega_s * 60 / (2 * math.pi)
        
        V_ph = self.calculator.voltage / math.sqrt(3)
        Ef = self.calculator.excitation * V_ph * 1.5
        Xs = self.calculator.synchronous_reactance
        
        power = []
        torque_em = []
        torque_load = []
        
        for i, (ti, delta_i, omega_i) in enumerate(zip(t, delta_rad, omega)):
            P_ph = (V_ph * Ef * math.sin(delta_i)) / Xs
            P_total = 3 * P_ph / 1000
            power.append(P_total)
            
            T_em = P_total * 1000 / omega_s if omega_s > 0 else 0
            torque_em.append(T_em)
            
            T_load = self.simulation_data['load_torque_func'](ti, omega_i)
            torque_load.append(T_load)
        
        ax1 = self.sim_fig.add_subplot(411)
        ax2 = self.sim_fig.add_subplot(412)
        ax3 = self.sim_fig.add_subplot(413)
        ax4 = self.sim_fig.add_subplot(414)
        
        ax1.plot(t, delta_deg, 'b-', linewidth=2)
        ax1.axhline(y=90, color='r', linestyle='--', label='Stability Limit', alpha=0.5)
        ax1.set_ylabel('Power Angle (°)', fontweight='bold')
        ax1.set_title('Synchronous Motor Dynamic Response', fontweight='bold', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best', fontsize=8)
        
        ax2.plot(t, speed_rpm, 'g-', linewidth=2, label='Rotor Speed')
        ax2.axhline(y=ns_rpm, color='r', linestyle='--', label='Synchronous Speed', alpha=0.5)
        ax2.set_ylabel('Speed (RPM)', fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best', fontsize=8)
        
        ax3.plot(t, power, 'm-', linewidth=2)
        ax3.set_ylabel('Power (kW)', fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        ax4.plot(t, torque_em, 'b-', linewidth=2, label='Electromagnetic Torque')
        ax4.plot(t, torque_load, 'r--', linewidth=2, label='Load Torque')
        ax4.set_xlabel('Time (s)', fontweight='bold')
        ax4.set_ylabel('Torque (Nm)', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend(loc='best', fontsize=8)
        
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

    def perform_comprehensive_analysis(self):
        """Perform comprehensive motor analysis"""
        if not self.update_calculator_params():
            return
        
        perf = self.calculator.calculate_steady_state_performance()
        
        self.perf_text.config(state='normal')
        self.perf_text.delete('1.0', 'end')
        
        output = f"""
{'='*50}
COMPREHENSIVE SYNCHRONOUS MOTOR ANALYSIS
{'='*50}

MOTOR SPECIFICATIONS:
{'─'*50}
Rated Power         : {self.calculator.power_rating:.2f} kW
Poles               : {self.calculator.poles}
Rated Frequency     : {self.calculator.frequency:.2f} Hz
Rated Voltage (L-L) : {self.calculator.voltage:.2f} V
Synchronous Speed   : {perf['sync_speed_rpm']:.2f} RPM
Sync. Reactance     : {self.calculator.synchronous_reactance:.2f} Ω

OPERATING POINT:
{'─'*50}
Power Angle         : {perf['power_angle_deg']:.2f}°
Excitation          : {perf['excitation_pu']:.2f} pu
Power Output        : {perf['power_kw']:.2f} kW
Torque              : {perf['torque_nm']:.2f} Nm
Armature Current    : {perf['current_a']:.2f} A
Power Factor        : {perf['power_factor']:.3f}

PERFORMANCE CHARACTERISTICS:
{'─'*50}
Efficiency          : ~95.0 %
Regulation          : Synchronous (0% speed drop)
Starting Method     : Damper winding or reduced voltage
Application         : Constant speed drives
Synchronization     : Required before load application

ADVANTAGES:
{'─'*50}
✓ Constant speed (synchronous)
✓ Power factor control via excitation
✓ High efficiency at rated load
✓ Can operate leading/lagging PF
✓ Suitable for large power applications

OPERATING REGIONS:
{'─'*50}
Under-excited: Lagging power factor
Over-excited:  Leading power factor
Critical angle: 90° (stability limit)
Normal range:  10° - 40° (stable operation)

{'='*50}
"""
        
        self.perf_text.insert('1.0', output)
        self.perf_text.config(state='disabled')
        
        self.plot_phasor_diagram()
        self.plot_v_curves()
    
    def plot_phasor_diagram(self):
        """Plot phasor diagram"""
        self.phasor_fig.clear()
        ax = self.phasor_fig.add_subplot(111, projection='polar')
        
        delta_rad = math.radians(self.calculator.delta_initial)
        V_ph = self.calculator.voltage / math.sqrt(3)
        Ef = self.calculator.excitation * V_ph * 1.5
        
        V_angle = 0
        Ef_angle = -delta_rad
        
        ax.plot([0, V_angle], [0, V_ph], 'b-', linewidth=3, label='V (Terminal Voltage)')
        ax.plot([0, Ef_angle], [0, Ef], 'r-', linewidth=3, label='Ef (Excitation)')
        
        theta = np.linspace(Ef_angle, V_angle, 20)
        r = V_ph * 0.3
        ax.plot(theta, [r]*len(theta), 'g--', linewidth=1)
        
        ax.set_title('Simplified Phasor Diagram', fontweight='bold', fontsize=11, pad=20)
        ax.legend(loc='upper right', fontsize=9, bbox_to_anchor=(1.3, 1.1))
        ax.grid(True, alpha=0.3)
        
        self.phasor_fig.tight_layout()
        self.phasor_canvas.draw()
    
    def plot_v_curves(self):
        """Plot V-curves"""
        self.vcurve_fig.clear()
        ax = self.vcurve_fig.add_subplot(111)
        
        V_ph = self.calculator.voltage / math.sqrt(3)
        Xs = self.calculator.synchronous_reactance
        delta_rad = math.radians(self.calculator.delta_initial)
        
        excitations = np.linspace(0.5, 2.0, 50)
        
        for power_level in [0.25, 0.5, 0.75, 1.0]:
            currents = []
            for exc in excitations:
                Ef = exc * V_ph * 1.5
                I_a = math.sqrt((V_ph - Ef*math.cos(delta_rad*power_level))**2 + 
                               (Ef*math.sin(delta_rad*power_level))**2) / Xs
                currents.append(I_a)
            
            ax.plot(excitations, currents, linewidth=2,
                   label=f'{power_level*100:.0f}% Load')
        
        ax.axvline(x=1.0, color='r', linestyle='--', alpha=0.5,
                  label='Unity PF (approx)')
        
        ax.set_xlabel('Excitation (pu)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Armature Current (A)', fontweight='bold', fontsize=10)
        ax.set_title('V-Curves (Excitation Characteristics)', fontweight='bold', fontsize=11)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(loc='best', fontsize=8)
        
        self.vcurve_fig.tight_layout()
        self.vcurve_canvas.draw()
    
    def reset_parameters(self):
        """Reset all parameters to default values"""
        self.calculator.reset_parameters()
        
        self.power_var.set(self.calculator.power_rating)
        self.poles_var.set(self.calculator.poles)
        self.freq_var.set(self.calculator.frequency)
        self.voltage_var.set(self.calculator.voltage)
        self.delta_var.set(self.calculator.delta_initial)
        self.excitation_var.set(self.calculator.excitation)
        self.xs_var.set(self.calculator.synchronous_reactance)
        
        self.calculate_torque_angles()
    
    def on_window_resize(self, event=None):
        """Handle window resize events for autoscaling"""
        pass


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = SynchronousMotorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
