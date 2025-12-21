"""
Advanced DC Shunt Motor Laboratory
----------------------------------
Interactive Python + Tkinter application for modeling, analysis, and
real-time simulation of a DC shunt motor. The tool provides:

1. Main menu with quick access to documentation and reset controls.
2. Input parameter widgets with both entry fields and sliders.
3. Dynamic simulation with selectable integrators (Euler and RK45).
4. Real-time visualization of speed and armature current.
5. Advanced analysis tabs covering fault-current estimation and
   protection coordination concepts.

The computational core includes steady-state calculations such as the
additional series resistance required to achieve a target speed (e.g.,
halving the speed under constant load torque) and dynamic state-space
integration for speed response.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import math
import threading
import time
from typing import Callable, List

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp


class DCShuntMotorModel:
    """Mathematical model and utilities for a DC shunt motor."""

    def __init__(self) -> None:
        self.voltage = 230.0
        self.armature_resistance = 0.5
        self.additional_resistance = 0.0
        self.back_emf_constant = 1.2  # V/(rad/s)
        self.torque_constant = 1.2  # Nm/A
        self.inertia = 0.25  # kg.m^2
        self.friction_coeff = 0.02  # Nms/rad
        self.load_torque = 10.0  # Nm
        self.initial_speed_rad_s = 0.0
        self.armature_current = 20.0

    def electrical_speed(self, omega: float) -> float:
        """Return the back EMF for a given speed."""
        return self.back_emf_constant * omega

    def steady_state_current(self, omega: float) -> float:
        """Compute armature current at a given speed."""
        emf = self.electrical_speed(omega)
        total_r = self.armature_resistance + self.additional_resistance
        return (self.voltage - emf) / total_r if total_r > 0 else 0.0

    def electromagnetic_torque(self, omega: float) -> float:
        """Torque produced by the motor at a given speed."""
        ia = self.steady_state_current(omega)
        return self.torque_constant * ia

    def required_series_resistance_for_speed_ratio(
        self, target_ratio: float, armature_current: float
    ) -> float:
        """
        Calculate additional series resistance to reach a speed ratio.

        Assumes constant flux and constant armature current (constant load
        torque). Speed is proportional to back EMF. For the initial state,
        E1 = V - Ia*Ra. For the target speed ratio k = ω2/ω1, the required
        back EMF is E2 = k * E1. Additional resistance is obtained from
        V - Ia*(Ra + R_add) = E2.
        """
        if not 0 < target_ratio:
            raise ValueError("Target speed ratio must be positive.")

        e1 = self.voltage - armature_current * self.armature_resistance
        e2 = target_ratio * e1
        r_add = (self.voltage - e2) / armature_current - self.armature_resistance
        return max(r_add, 0.0)

    def dynamics(self, t: float, state: np.ndarray, load_torque_func: Callable[[float, float], float]) -> List[float]:
        """Differential equations for angular speed and position."""
        omega, theta = state
        emf = self.electrical_speed(omega)
        total_r = self.armature_resistance + self.additional_resistance
        ia = (self.voltage - emf) / total_r if total_r > 0 else 0.0
        torque_em = self.torque_constant * ia

        torque_load = load_torque_func(t, omega)
        torque_friction = self.friction_coeff * omega
        domega_dt = (torque_em - torque_load - torque_friction) / self.inertia
        dtheta_dt = omega
        return [domega_dt, dtheta_dt]


class SimulationEngine:
    """Manage dynamic simulation with selectable solvers and buffering."""

    def __init__(self, model: DCShuntMotorModel) -> None:
        self.model = model
        self.reset()

    def reset(self) -> None:
        self.time_history: List[float] = [0.0]
        self.omega_history: List[float] = [self.model.initial_speed_rad_s]
        self.current_history: List[float] = [self.model.armature_current]
        self.running = False
        self.solver = "RK45"
        self.dt = 0.02
        self.max_time = 5.0
        self.load_torque_func: Callable[[float, float], float] = (
            lambda _t, _omega: self.model.load_torque
        )

    def step(self) -> bool:
        if len(self.time_history) >= 1 and self.time_history[-1] >= self.max_time:
            return False

        t0 = self.time_history[-1]
        omega0 = self.omega_history[-1]
        theta0 = 0.0
        state0 = np.array([omega0, theta0], dtype=float)

        if self.solver == "Euler":
            derivatives = self.model.dynamics(t0, state0, self.load_torque_func)
            new_state = state0 + np.array(derivatives) * self.dt
        else:
            sol = solve_ivp(
                lambda t, y: self.model.dynamics(t, y, self.load_torque_func),
                (t0, t0 + self.dt),
                state0,
                method="RK45",
                max_step=self.dt,
            )
            new_state = sol.y[:, -1]

        omega_new = max(new_state[0], 0.0)
        t_new = t0 + self.dt

        self.time_history.append(t_new)
        self.omega_history.append(omega_new)

        emf = self.model.electrical_speed(omega_new)
        total_r = self.model.armature_resistance + self.model.additional_resistance
        ia = (self.model.voltage - emf) / total_r if total_r > 0 else 0.0
        self.current_history.append(ia)
        return True


class DCShuntMotorLabApp(tk.Tk):
    """Tkinter GUI application for the DC shunt motor laboratory."""

    def __init__(self) -> None:
        super().__init__()
        self.title("DC Shunt Motor Virtual Lab")
        self.geometry("1100x750")
        self.model = DCShuntMotorModel()
        self.sim_engine = SimulationEngine(self.model)
        self._simulation_thread: threading.Thread | None = None

        self._build_menu()
        self._build_layout()
        self._configure_responsive()
        self._update_resistance_display()

    def _build_menu(self) -> None:
        menubar = tk.Menu(self)
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Reset", command=self.reset_all)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.destroy)
        menubar.add_cascade(label="Main", menu=file_menu)

        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(
            label="About",
            command=lambda: messagebox.showinfo(
                "About", "DC Shunt Motor Lab – GPT-5.1-Codex-Max driven assistant"
            ),
        )
        menubar.add_cascade(label="Help", menu=help_menu)
        self.config(menu=menubar)

    def _build_layout(self) -> None:
        notebook = ttk.Notebook(self)
        notebook.pack(fill=tk.BOTH, expand=True)

        self.tab_inputs = ttk.Frame(notebook)
        self.tab_simulation = ttk.Frame(notebook)
        self.tab_faults = ttk.Frame(notebook)
        self.tab_protection = ttk.Frame(notebook)

        notebook.add(self.tab_inputs, text="Inputs & Steady-State")
        notebook.add(self.tab_simulation, text="Dynamics & Visualization")
        notebook.add(self.tab_faults, text="Fault Current Modeling")
        notebook.add(self.tab_protection, text="Protection Coordination")

        self._build_inputs_tab()
        self._build_simulation_tab()
        self._build_fault_tab()
        self._build_protection_tab()

    def _configure_responsive(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        for tab in [self.tab_inputs, self.tab_simulation, self.tab_faults, self.tab_protection]:
            tab.columnconfigure(0, weight=1)
            tab.rowconfigure(0, weight=1)
        self.bind("<Configure>", lambda _event: self._resize_plots())

    def _build_inputs_tab(self) -> None:
        container = ttk.Frame(self.tab_inputs, padding=10)
        container.grid(sticky="nsew")
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        self.params = {
            "voltage": tk.DoubleVar(value=self.model.voltage),
            "armature_resistance": tk.DoubleVar(value=self.model.armature_resistance),
            "additional_resistance": tk.DoubleVar(value=self.model.additional_resistance),
            "load_torque": tk.DoubleVar(value=self.model.load_torque),
            "inertia": tk.DoubleVar(value=self.model.inertia),
            "friction": tk.DoubleVar(value=self.model.friction_coeff),
            "torque_constant": tk.DoubleVar(value=self.model.torque_constant),
            "emf_constant": tk.DoubleVar(value=self.model.back_emf_constant),
            "speed_ratio": tk.DoubleVar(value=0.5),
            "target_current": tk.DoubleVar(value=self.model.armature_current),
        }

        sliders = [
            ("Supply Voltage (V)", "voltage", 100, 400, 1),
            ("Armature Resistance (Ω)", "armature_resistance", 0.1, 2.0, 0.01),
            ("Additional Series R (Ω)", "additional_resistance", 0.0, 10.0, 0.1),
            ("Load Torque (Nm)", "load_torque", 0, 60, 1),
            ("Inertia (kg·m²)", "inertia", 0.05, 1.5, 0.01),
            ("Friction Coefficient", "friction", 0.0, 0.2, 0.005),
            ("Torque Constant (Nm/A)", "torque_constant", 0.5, 2.5, 0.05),
            ("Back EMF Constant (V/(rad/s))", "emf_constant", 0.5, 2.5, 0.05),
            ("Target Speed Ratio", "speed_ratio", 0.2, 1.2, 0.02),
            ("Target Armature Current (A)", "target_current", 5, 50, 1),
        ]

        for idx, (label, key, mn, mx, res) in enumerate(sliders):
            row = idx // 2
            col = idx % 2
            frame = ttk.LabelFrame(container, text=label, padding=8)
            frame.grid(row=row, column=col, sticky="nsew", padx=6, pady=6)
            container.rowconfigure(row, weight=1)
            container.columnconfigure(col, weight=1)

            scale = ttk.Scale(
                frame,
                orient=tk.HORIZONTAL,
                from_=mn,
                to=mx,
                variable=self.params[key],
                command=lambda _val, k=key: self._update_param(k),
            )
            scale.pack(fill=tk.X, padx=4, pady=4)
            entry = ttk.Entry(frame, textvariable=self.params[key])
            entry.pack(fill=tk.X, padx=4, pady=2)
            entry.bind("<FocusOut>", lambda _e, k=key: self._update_param(k))

        button_row = ttk.Frame(container)
        button_row.grid(row=5, column=0, columnspan=2, pady=10, sticky="ew")
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)
        button_row.columnconfigure(2, weight=1)

        ttk.Button(button_row, text="Compute Half-Speed Resistance", command=self._compute_half_speed).grid(
            row=0, column=0, padx=4, sticky="ew"
        )
        ttk.Button(button_row, text="Apply Parameters", command=self._apply_parameters).grid(
            row=0, column=1, padx=4, sticky="ew"
        )
        ttk.Button(button_row, text="Reset", command=self.reset_all).grid(
            row=0, column=2, padx=4, sticky="ew"
        )

        self.result_label = ttk.Label(container, text="", foreground="#004080", anchor="center")
        self.result_label.grid(row=6, column=0, columnspan=2, sticky="ew", pady=6)

    def _build_simulation_tab(self) -> None:
        container = ttk.Frame(self.tab_simulation, padding=10)
        container.grid(sticky="nsew")
        container.columnconfigure(0, weight=2)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)

        plot_frame = ttk.LabelFrame(container, text="Real-Time Simulation", padding=8)
        plot_frame.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)

        self.figure = Figure(figsize=(7, 5), dpi=100)
        self.ax_speed = self.figure.add_subplot(211)
        self.ax_current = self.figure.add_subplot(212)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.LabelFrame(container, text="Controls & Solver", padding=8)
        control_frame.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        control_frame.columnconfigure(0, weight=1)

        ttk.Label(control_frame, text="Time Step (s)").pack(fill=tk.X, pady=2)
        self.entry_dt = ttk.Entry(control_frame)
        self.entry_dt.insert(0, str(self.sim_engine.dt))
        self.entry_dt.pack(fill=tk.X, pady=2)

        ttk.Label(control_frame, text="Max Time (s)").pack(fill=tk.X, pady=2)
        self.entry_maxt = ttk.Entry(control_frame)
        self.entry_maxt.insert(0, str(self.sim_engine.max_time))
        self.entry_maxt.pack(fill=tk.X, pady=2)

        ttk.Label(control_frame, text="Solver").pack(fill=tk.X, pady=2)
        self.solver_choice = tk.StringVar(value=self.sim_engine.solver)
        solver_box = ttk.Combobox(control_frame, textvariable=self.solver_choice, state="readonly")
        solver_box["values"] = ("RK45", "Euler")
        solver_box.pack(fill=tk.X, pady=2)

        self.btn_start = ttk.Button(control_frame, text="Start", command=self.start_simulation)
        self.btn_start.pack(fill=tk.X, pady=4)
        ttk.Button(control_frame, text="Stop", command=self.stop_simulation).pack(fill=tk.X, pady=4)
        ttk.Button(control_frame, text="Reset", command=self.reset_all).pack(fill=tk.X, pady=4)

        ttk.Separator(control_frame).pack(fill=tk.X, pady=6)
        ttk.Label(control_frame, text="Live Measurements").pack(fill=tk.X, pady=2)
        self.label_live_speed = ttk.Label(control_frame, text="Speed: 0 rpm")
        self.label_live_speed.pack(fill=tk.X, pady=2)
        self.label_live_current = ttk.Label(control_frame, text="Armature Current: 0 A")
        self.label_live_current.pack(fill=tk.X, pady=2)

    def _build_fault_tab(self) -> None:
        container = ttk.Frame(self.tab_faults, padding=10)
        container.grid(sticky="nsew")
        container.columnconfigure(0, weight=1)

        description = (
            "Estimate fault currents using the Thevenin equivalent at the motor terminals. "
            "Adjust source impedance to visualize fault duty."
        )
        ttk.Label(container, text=description, wraplength=800, justify=tk.LEFT).grid(
            row=0, column=0, sticky="w"
        )

        param_frame = ttk.LabelFrame(container, text="Fault Parameters", padding=8)
        param_frame.grid(row=1, column=0, sticky="ew", pady=6)
        param_frame.columnconfigure(1, weight=1)

        ttk.Label(param_frame, text="Source Impedance (Ω)").grid(row=0, column=0, sticky="w")
        self.var_source_z = tk.DoubleVar(value=0.3)
        ttk.Entry(param_frame, textvariable=self.var_source_z).grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(param_frame, text="Fault Resistance (Ω)").grid(row=1, column=0, sticky="w")
        self.var_fault_r = tk.DoubleVar(value=0.05)
        ttk.Entry(param_frame, textvariable=self.var_fault_r).grid(row=1, column=1, sticky="ew", padx=4)

        ttk.Label(param_frame, text="Prefault Voltage (V)").grid(row=2, column=0, sticky="w")
        self.var_prefault_v = tk.DoubleVar(value=self.model.voltage)
        ttk.Entry(param_frame, textvariable=self.var_prefault_v).grid(row=2, column=1, sticky="ew", padx=4)

        ttk.Button(param_frame, text="Compute Fault Current", command=self._compute_fault_current).grid(
            row=3, column=0, columnspan=2, pady=6, sticky="ew"
        )

        self.label_fault_result = ttk.Label(param_frame, text="", foreground="#006400")
        self.label_fault_result.grid(row=4, column=0, columnspan=2, sticky="w", pady=4)

    def _build_protection_tab(self) -> None:
        container = ttk.Frame(self.tab_protection, padding=10)
        container.grid(sticky="nsew")
        container.columnconfigure(0, weight=1)

        ttk.Label(
            container,
            text=(
                "Protection coordination explorer – compare device pickup and clearing times "
                "against motor withstand curves."
            ),
            wraplength=800,
            justify=tk.LEFT,
        ).grid(row=0, column=0, sticky="w")

        frame = ttk.LabelFrame(container, text="Protection Settings", padding=8)
        frame.grid(row=1, column=0, sticky="ew", pady=6)
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Breaker Pickup (A)").grid(row=0, column=0, sticky="w")
        self.var_pickup = tk.DoubleVar(value=80.0)
        ttk.Entry(frame, textvariable=self.var_pickup).grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(frame, text="Breaker Time Dial (s)").grid(row=1, column=0, sticky="w")
        self.var_time_dial = tk.DoubleVar(value=0.3)
        ttk.Entry(frame, textvariable=self.var_time_dial).grid(row=1, column=1, sticky="ew", padx=4)

        ttk.Label(frame, text="Fuse Melt I²t (A²s)").grid(row=2, column=0, sticky="w")
        self.var_fuse_i2t = tk.DoubleVar(value=5000.0)
        ttk.Entry(frame, textvariable=self.var_fuse_i2t).grid(row=2, column=1, sticky="ew", padx=4)

        ttk.Button(frame, text="Evaluate Coordination", command=self._evaluate_protection).grid(
            row=3, column=0, columnspan=2, pady=6, sticky="ew"
        )

        self.label_protection_result = ttk.Label(frame, text="", foreground="#8B0000")
        self.label_protection_result.grid(row=4, column=0, columnspan=2, sticky="w", pady=4)

    # Core logic helpers
    def _update_param(self, key: str) -> None:
        try:
            value = float(self.params[key].get())
        except tk.TclError:
            return
        attr = key if key != "friction" else "friction_coeff"
        if hasattr(self.model, attr):
            setattr(self.model, attr, value)
        if key == "emf_constant":
            self.model.back_emf_constant = value
        if key == "torque_constant":
            self.model.torque_constant = value
        if key == "speed_ratio":
            self._update_resistance_display()
        if key == "target_current":
            self._update_resistance_display()

    def _apply_parameters(self) -> None:
        for key in self.params:
            self._update_param(key)
        self.model.additional_resistance = float(self.params["additional_resistance"].get())
        self.sim_engine.dt = float(self.entry_dt.get() or self.sim_engine.dt)
        self.sim_engine.max_time = float(self.entry_maxt.get() or self.sim_engine.max_time)
        self.sim_engine.solver = self.solver_choice.get()
        self.sim_engine.reset()
        self._update_visualization()
        self._update_resistance_display()

    def _compute_half_speed(self) -> None:
        try:
            target_ratio = float(self.params["speed_ratio"].get())
            ia = float(self.params["target_current"].get())
            r_add = self.model.required_series_resistance_for_speed_ratio(target_ratio, ia)
            message = (
                f"To achieve {target_ratio:.2f}× speed with Ia={ia:.1f} A, "
                f"add {r_add:.2f} Ω in series."
            )
            self.params["additional_resistance"].set(round(r_add, 3))
            self.result_label.configure(text=message)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Calculation error", str(exc))

    def _update_resistance_display(self) -> None:
        try:
            target_ratio = float(self.params["speed_ratio"].get())
            ia = float(self.params["target_current"].get())
            r_add = self.model.required_series_resistance_for_speed_ratio(target_ratio, ia)
            baseline_e = self.model.voltage - ia * self.model.armature_resistance
            half_speed_text = (
                f"Back EMF baseline: {baseline_e:.1f} V | Target Radd: {r_add:.2f} Ω"
            )
            self.result_label.configure(text=half_speed_text)
        except Exception:
            self.result_label.configure(text="")

    def start_simulation(self) -> None:
        if self.sim_engine.running:
            return
        self._apply_parameters()
        self.sim_engine.running = True
        self.btn_start.state(["disabled"])
        self._simulation_thread = threading.Thread(target=self._run_simulation_loop, daemon=True)
        self._simulation_thread.start()

    def _run_simulation_loop(self) -> None:
        while self.sim_engine.running:
            step_ok = self.sim_engine.step()
            self.after(0, self._update_visualization)
            if not step_ok:
                break
            time.sleep(self.sim_engine.dt)
        self.sim_engine.running = False
        self.after(0, lambda: self.btn_start.state(["!disabled"]))

    def stop_simulation(self) -> None:
        self.sim_engine.running = False

    def reset_all(self) -> None:
        self.stop_simulation()
        self.model = DCShuntMotorModel()
        self.sim_engine = SimulationEngine(self.model)

        defaults = {
            "voltage": self.model.voltage,
            "armature_resistance": self.model.armature_resistance,
            "additional_resistance": self.model.additional_resistance,
            "load_torque": self.model.load_torque,
            "inertia": self.model.inertia,
            "friction": self.model.friction_coeff,
            "torque_constant": self.model.torque_constant,
            "emf_constant": self.model.back_emf_constant,
            "speed_ratio": 0.5,
            "target_current": self.model.armature_current,
        }
        for key, value in defaults.items():
            if key in self.params:
                self.params[key].set(value)

        self.entry_dt.delete(0, tk.END)
        self.entry_dt.insert(0, str(self.sim_engine.dt))
        self.entry_maxt.delete(0, tk.END)
        self.entry_maxt.insert(0, str(self.sim_engine.max_time))
        self.solver_choice.set(self.sim_engine.solver)

        self._update_visualization()
        self._update_resistance_display()

    def _update_visualization(self) -> None:
        self.ax_speed.clear()
        self.ax_current.clear()

        time_axis = self.sim_engine.time_history
        omega_axis = self.sim_engine.omega_history
        current_axis = self.sim_engine.current_history

        rpm = [w * 60 / (2 * math.pi) for w in omega_axis]
        self.ax_speed.plot(time_axis, rpm, label="Speed (rpm)", color="#1f77b4")
        self.ax_speed.set_ylabel("Speed (rpm)")
        self.ax_speed.grid(True)
        self.ax_speed.legend(loc="upper right")

        self.ax_current.plot(time_axis, current_axis, label="Armature Current (A)", color="#d62728")
        self.ax_current.set_ylabel("Current (A)")
        self.ax_current.set_xlabel("Time (s)")
        self.ax_current.grid(True)
        self.ax_current.legend(loc="upper right")

        if time_axis:
            self.ax_speed.set_xlim(0, max(time_axis))
            self.ax_current.set_xlim(0, max(time_axis))

        self.canvas.draw()

        if omega_axis:
            rpm_latest = rpm[-1]
            ia_latest = current_axis[-1]
            self.label_live_speed.configure(text=f"Speed: {rpm_latest:.1f} rpm")
            self.label_live_current.configure(text=f"Armature Current: {ia_latest:.2f} A")

    def _resize_plots(self) -> None:
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)
        self.canvas.draw_idle()

    def _compute_fault_current(self) -> None:
        v_prefault = float(self.var_prefault_v.get())
        z_source = float(self.var_source_z.get())
        r_fault = float(self.var_fault_r.get())
        r_motor = self.model.armature_resistance + self.model.additional_resistance
        total_impedance = z_source + r_fault + r_motor
        if total_impedance <= 0:
            messagebox.showerror("Invalid impedance", "Total impedance must be positive")
            return
        fault_current = v_prefault / total_impedance
        self.label_fault_result.configure(
            text=f"Estimated fault current: {fault_current:.1f} A (including motor armature)"
        )

    def _evaluate_protection(self) -> None:
        pickup = float(self.var_pickup.get())
        time_dial = float(self.var_time_dial.get())
        fuse_i2t = float(self.var_fuse_i2t.get())

        running_current = float(self.params["target_current"].get())
        fault_current = self.var_prefault_v.get() / (
            self.var_source_z.get() + self.var_fault_r.get() + self.model.armature_resistance
        )

        breaker_time = time_dial * (fault_current / pickup) ** -1 if fault_current > pickup else float("inf")
        fuse_time = fuse_i2t / (fault_current**2) if fault_current > 0 else float("inf")

        assessment = [
            f"Running current margin: {running_current / pickup * 100:.1f}% of pickup",
            f"Breaker clearing time: {breaker_time:.3f} s",
            f"Fuse melt time: {fuse_time:.3f} s",
        ]

        if fuse_time < breaker_time:
            assessment.append("Fuse likely clears first – good selectivity for upstream breaker.")
        else:
            assessment.append("Breaker may trip before fuse – adjust pickup/time dial for coordination.")

        self.label_protection_result.configure(text="\n".join(assessment))


def main() -> None:
    app = DCShuntMotorLabApp()
    app.mainloop()


if __name__ == "__main__":
    main()
