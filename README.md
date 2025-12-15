# Advanced Electrical Engineering Motor Analysis Tools

This repository contains comprehensive Python + Tkinter GUI applications for electrical engineering motor analysis and simulation.

## Applications

### 1. Synchronous Motor Analysis Tool (`synchronous_motor_analysis.py`)

A comprehensive application for analyzing synchronous motor performance and solving torque angle problems.

#### Features

- **Torque Angle Analysis**: Solves the classic synchronous motor problem analyzing how torque angles change under varying conditions
- **Power-Angle Characteristics**: Visualizes complete power-angle curves with stability limits
- **Dynamic Simulation**: Real-time motor transient simulation with multiple ODE solvers (RK45, Euler)
- **Comprehensive Analysis**: Includes phasor diagrams, V-curves, and detailed performance metrics
- **Auto-scaling GUI**: Automatically adjusts to window resizing

#### Problem Solved

Given a synchronous motor at full load with torque angle of 30° (electrical) and negligible stator resistance, the application calculates torque angle changes for three scenarios:

1. **Scenario (i)**: Load torque and terminal voltage constant, excitation and frequency raised by 10%
   - Result: Torque angle **increases to ~33.4°**

2. **Scenario (ii)**: Load power and terminal voltage constant, excitation and frequency reduced by 10%
   - Result: Torque angle **remains at 30°**

3. **Scenario (iii)**: Load torque and excitation constant, terminal voltage and frequency raised by 10%
   - Result: Torque angle **increases to ~33.4°**

#### Tabs

1. **Torque Angle Analysis**: Main problem solution with detailed mathematical derivations
2. **Power-Angle Curve**: Interactive power-angle characteristics with operating point visualization
3. **Dynamic Simulation**: Real-time transient simulation with adjustable parameters
4. **Comprehensive Analysis**: Phasor diagrams, V-curves, and complete performance data
5. **About**: Application information and theoretical background

### 2. Induction Motor Calculator (`induction_motor_calculator.py`)

Advanced analysis tool for induction motor steady-state and dynamic performance.

#### Features

- Steady-state performance calculations
- Torque-speed characteristics
- Dynamic starting simulation
- Power flow visualization
- Multiple ODE solvers for transient analysis

## Installation

### Requirements

- Python 3.7+
- NumPy
- Matplotlib
- SciPy
- Tkinter (usually comes with Python)

### Setup

```bash
# Clone the repository
git clone https://github.com/Karolsak/codex11.git
cd codex11

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Running the Synchronous Motor Analysis Tool

```bash
python synchronous_motor_analysis.py
```

The application will open with a comprehensive GUI featuring:
- Adjustable motor parameters via sliders and input fields
- Real-time calculation updates
- Interactive plots and visualizations
- Multiple analysis tabs

### Running the Induction Motor Calculator

```bash
python induction_motor_calculator.py
```

## Application Screenshots

The applications feature:
- Professional multi-tab interface
- Real-time parameter adjustment with sliders
- High-quality matplotlib visualizations
- Responsive auto-scaling layouts

## Key Components

### Synchronous Motor Calculator

```python
class SynchronousMotorCalculator:
    - calculate_torque_angle_scenario1()  # Ef & f raised 10%
    - calculate_torque_angle_scenario2()  # Ef & f reduced 10%
    - calculate_torque_angle_scenario3()  # V & f raised 10%
    - calculate_power_angle_curve()
    - motor_dynamics_sync()  # ODE system
    - calculate_steady_state_performance()
```

### GUI Features

- **Automatic resizing**: All plots and UI elements adapt to window size changes
- **Input validation**: Prevents invalid parameter entries
- **Real-time updates**: Sliders provide immediate visual feedback
- **Multiple visualization modes**: Charts, curves, diagrams, and numerical results

## Technical Details

### ODE Solvers

The application implements two numerical integration methods:

1. **RK45 (Runge-Kutta 4/5)**: Adaptive step-size method for accurate results
2. **Euler Method**: Fixed-step method for educational comparison

### Mathematical Model

Synchronous motor equations:
- Power: `P = (V·Ef·sin δ)/Xs`
- Torque: `T = P/ωs = (V·Ef·sin δ)/(Xs·ωs)`
- Dynamics: `J·dω/dt = T_em - T_load - T_friction`
- Power angle: `dδ/dt = (P/2)·(ω - ωs)`

Where:
- `V`: Terminal voltage (per phase)
- `Ef`: Excitation voltage
- `δ`: Power/torque angle (electrical)
- `Xs`: Synchronous reactance
- `ωs`: Synchronous angular velocity

## Educational Value

These applications are designed for:
- Electrical engineering students
- Power systems analysis courses
- Motor control education
- Professional engineering analysis

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is open source and available for educational and professional use.

## Author

Developed for electrical engineering education and analysis.

## References

- Synchronous Motor Theory
- Induction Motor Equivalent Circuit Analysis
- Numerical Methods for Differential Equations
- Power Systems Analysis

---

**Note**: This application requires a graphical display environment. For headless systems, consider using virtual display solutions like Xvfb.
