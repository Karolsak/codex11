# Quick Start Guide

## Running the Synchronous Motor Analysis Tool

### Step 1: Install Dependencies

```bash
pip install -r requirements.txt
```

This installs:
- NumPy (numerical computing)
- Matplotlib (plotting)
- SciPy (scientific computing, ODE solvers)

### Step 2: Run the Application

```bash
python synchronous_motor_analysis.py
```

### Step 3: Using the Application

#### Tab 1: Torque Angle Analysis (Main Problem)

1. **Set Initial Parameters** (left panel):
   - Power Rating: 100 kW (default)
   - Poles: 4 (default)
   - Frequency: 50 Hz
   - Voltage: 400 V
   - Initial Torque Angle: 30° (use slider)
   - Excitation: 1.0 pu (use slider)
   - Synchronous Reactance: 2.0 Ω

2. **Click "Calculate"** to solve all three scenarios

3. **View Results** (right panel):
   - Scenario (i): Ef & f +10% → Torque angle increases to ~33.4°
   - Scenario (ii): Ef & f -10% → Torque angle remains at 30°
   - Scenario (iii): V & f +10% → Torque angle increases to ~33.4°

#### Tab 2: Power-Angle Curve

1. The curve automatically generates when you open the tab
2. Click "Generate Curve" to refresh after parameter changes
3. **Observe**:
   - Blue curve: Complete P-δ characteristics
   - Red dot: Current operating point
   - Red dashed line: Stability limit (90°)
   - Green shaded: Stable operating region
   - Red shaded: Unstable region

#### Tab 3: Dynamic Simulation

1. **Set Simulation Parameters**:
   - Simulation Time: 5.0 s (default)
   - Load Torque: 200 Nm
   - Inertia: 10 kg·m²
   
2. **Choose ODE Solver**:
   - RK45 (Adaptive): For accurate results (recommended)
   - Euler (Fixed Step): For educational comparison

3. **Click "Start Simulation"**

4. **View Results** (4 plots):
   - Power angle vs time
   - Rotor speed vs time
   - Power output vs time
   - Torque comparison (electromagnetic vs load)

5. **Controls**:
   - Stop: Interrupt simulation
   - Reset: Clear results

#### Tab 4: Comprehensive Analysis

1. **Click "Perform Comprehensive Analysis"**

2. **View Results**:
   - Left: Steady-state performance metrics
   - Top Right: Phasor diagram
   - Bottom Right: V-curves (excitation characteristics)

#### Tab 5: About

- Contains theoretical background
- Usage instructions
- Problem solution summary

### Tips for Best Results

1. **Adjust Parameters Interactively**:
   - Use sliders for real-time updates
   - Torque angle slider: 0-90°
   - Excitation slider: 0.5-2.0 pu

2. **Window Resizing**:
   - All plots automatically scale with window size
   - Drag window corners or edges to resize

3. **Parameter Ranges**:
   - Keep torque angle < 90° for stable operation
   - Typical excitation: 0.8-1.2 pu for normal operation
   - Over-excitation (>1.0): Leading power factor
   - Under-excitation (<1.0): Lagging power factor

### Understanding the Results

#### Scenario (i): Ef & f raised 10%
- **Physics**: Reactance increases with frequency (Xs ∝ f)
- **Effect**: Higher excitation partially compensated by higher reactance
- **Result**: Net increase in torque angle

#### Scenario (ii): Ef & f reduced 10%
- **Physics**: Power equation P = (V·Ef·sin δ)/Xs
- **Effect**: Ef and Xs both scale by 0.9, canceling out
- **Result**: Torque angle unchanged

#### Scenario (iii): V & f raised 10%
- **Physics**: Similar to scenario (i), voltage increase doesn't fully compensate
- **Effect**: Increased reactance dominates
- **Result**: Torque angle increases

### Common Issues

#### Issue: "No module named 'tkinter'"
**Solution**: Install tkinter
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# macOS (should be included with Python)
brew install python-tk

# Windows (should be included with Python)
# Reinstall Python with tkinter option checked
```

#### Issue: "No module named 'numpy'" (or matplotlib/scipy)
**Solution**: Install requirements
```bash
pip install -r requirements.txt
```

#### Issue: Application window is blank/frozen
**Solution**: Ensure you have a display environment
- For local machines: Should work by default
- For remote servers: Use X11 forwarding or VNC
- For headless servers: Use virtual display (Xvfb)

### Example Workflow

1. **Problem Solving**:
   ```
   Open App → Tab 1 → Set δ = 30° → Click Calculate → Read Results
   ```

2. **Exploring Power Characteristics**:
   ```
   Tab 2 → Adjust Excitation Slider → Generate Curve → Observe Changes
   ```

3. **Simulating Motor Starting**:
   ```
   Tab 3 → Set Parameters → Select RK45 → Start Simulation → Analyze Plots
   ```

4. **Complete Analysis**:
   ```
   Tab 4 → Perform Analysis → Study Phasors & V-Curves
   ```

### Educational Value

- **For Students**: Learn synchronous motor theory through visualization
- **For Engineers**: Analyze motor performance under various conditions
- **For Educators**: Demonstrate concepts with interactive tool

### Next Steps

- Experiment with different motor parameters
- Compare RK45 vs Euler solver accuracy
- Analyze stability limits by varying torque angle
- Study effect of excitation on power factor (V-curves)

---

**Need Help?** Refer to README.md for detailed documentation.
