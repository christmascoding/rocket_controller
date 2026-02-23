# Visualization Enhancement Design Document

## Overview
This document outlines the planned enhancements to the rocket controller visualization system, focusing on three main areas:
1. **Left Panel**: Close-up rocket attitude view
2. **Right Panel**: MPC controller internals visualization
3. **Performance Optimization**: Strategies to reduce lag

---

## 1. New Layout Structure

### Current Layout (3x2 grid)
```
+-------------------+-------------+
|                   |  Thrust     |
|   3D Trajectory   |  Plot       |
|   (Main View)     +-------------+
|                   |  Gimbal     |
|                   |  Pitch      |
|                   +-------------+
|                   |  Gimbal Yaw |
+-------------------+-------------+
```

### Proposed Layout (3x3 grid)
```
+-------------+-------------------+-------------+
|             |                   |  Position   |
|  Rocket     |   3D Trajectory   |  Error      |
|  Close-up   |   (Main View)     +-------------+
|  (Fixed 3D) |                   |  MPC Ref    |
|             |                   |  Horizon    |
|             |                   +-------------+
|             |                   |  MPC Cost   |
+-------------+-------------------+-------------+
```

**Grid Ratios**: `width_ratios=[1.0, 2.0, 1.2]`, `height_ratios=[1.0, 1.0, 1.0]`

---

## 2. Left Panel: Rocket Close-Up View

### Purpose
Provide a detailed, zoomed-in view of the rocket's attitude, gimbal deflection, and thrust vector in isolation from the trajectory.

### Specifications

#### 2.1 View Configuration
- **Type**: 3D axis (Axes3D)
- **Perspective**: Fixed camera angle (not user-rotatable)
  - Elevation: 15° (looking slightly down)
  - Azimuth: 45° (diagonal front-right view)
  - Use `ax.view_init(elev=15, azim=45)`
  - Disable mouse interaction: `ax.mouse_init(rotate_btn=None, zoom_btn=None)`
- **Scale**: Always centered on rocket origin, fixed axis limits
  - Limits: x=[-10, 10], y=[-10, 10], z=[0, 15]
  - Shows rocket body (0-7m) with margin

#### 2.2 Visual Elements

**Rocket Body** (same as main view):
- Cylinder: radius=0.8m, height=6m
- Cone: base radius=0.8m, height=1m
- Color: Light gray (#CCCCCC)
- Alpha: 0.8 for transparency

**Thrust Vector Stick**:
- **Line from nozzle** (z=0) in gimbal deflection direction
- Length: Proportional to thrust magnitude
  - Scale: `length = 8.0 * (thrust_n / max_thrust_n)`
  - Max length: 8m (at full thrust)
  - Min length: 0m (zero thrust)
- Color: **Orange-red gradient** based on thrust intensity
  - High thrust (>80%): `#FF4500` (OrangeRed)
  - Medium thrust (40-80%): `#FFA500` (Orange)
  - Low thrust (<40%): `#FFD700` (Gold)
- Line width: 5.0 (thick for visibility)
- Add small cone at end for directionality (arrowhead-style)

**Gimbal Deflection Indicator**:
- **Transparent cone** showing gimbal deflection range
- Base at nozzle (z=0)
- Apex pointing in gimbal direction
- Height: 3m
- Base radius: `3.0 * tan(gimbal_angle)` (shows actual deflection)
- Color: Cyan (#00FFFF)
- Alpha: 0.2 (very transparent)

**Body Attitude Reference**:
- **Vertical line** (local Z-axis before rotation)
- From z=0 to z=8
- Color: Green (#00FF00)
- Line style: Dashed
- Width: 1.5
- Label: "Body Z-axis"

**Coordinate Frame**:
- Small XYZ triad at origin (0,0,0)
- X: Red, Y: Green, Z: Blue
- Length: 2m each
- Width: 1.0

#### 2.3 Text Annotations

Display key values as text overlay:
```
Pitch:  XX.X°
Yaw:    XX.X°
Gimbal: (XX.X°, XX.X°)
Thrust: XXXX N (XX%)
```

Position: Upper-left corner of subplot
Font: Monospace, size 9
Background: Semi-transparent white box

#### 2.4 Title
```
"Rocket Attitude & Thrust"
```

---

## 3. Right Panel: MPC Internals Visualization

### Purpose
Show what the MPC controller "sees" and how it makes decisions - educational for students learning control theory.

### Three Subplots (stacked vertically)

---

### 3.1 Top: Position Error Over Time

**Content**:
- X-axis: Time (seconds)
- Y-axis: Position error magnitude (meters)
- Line: Rolling window of last 200 time steps (~4 seconds)
- Color: Red (#FF0000)
- Fill under curve: Light red with alpha=0.3

**Features**:
- Horizontal reference line at y=0 (dashed gray)
- Grid: Minor grid enabled
- Title: "Position Error from Closest Path Point"
- Y-label: "Error [m]"

**Additional Line** (optional):
- Desired error threshold (e.g., 5m) as dashed green line
- Shows acceptable tracking accuracy

---

### 3.2 Middle: MPC Reference Horizon (2D Projection)

**Content**:
- Shows the MPC's "look-ahead" - what it's planning
- **XY projection** (top-down view) or **XZ projection** (side view) - make configurable
- Plot elements:
  1. **Current position**: Large red dot (●)
  2. **Reference horizon**: Green line with markers (○)
     - Plots `pos_refs[:, 0:N+1]` from MPC
     - Shows next 15 waypoints the MPC is tracking
  3. **Closest path point**: Purple dot (●)
  4. **Trajectory segment**: Light gray line showing path ahead

**Projection Selection**:
- Default: XZ (side view) - most informative for vertical trajectories
- Could add dropdown or toggle button later

**Axes**:
- Equal aspect ratio
- Auto-scaling to horizon bounds
- X-label: "X Position [m]" or "Z Position [m]"
- Y-label: "Y Position [m]" or "X Position [m]"
- Title: "MPC Prediction Horizon (XZ View)"

**Legend**:
- Current, Reference Path, Closest Point

---

### 3.3 Bottom: MPC Cost Components

**Content**:
- Stacked area chart or individual lines showing cost breakdown
- X-axis: Time (seconds)
- Y-axis: Cost value (arbitrary units)

**Lines/Areas**:
1. **Position Cost**: Blue - `weight_pos * ||pos_error||²`
2. **Velocity Cost**: Green - `weight_vel * ||vel_error||²`
3. **Acceleration Cost**: Orange - `weight_accel * ||accel||²`
4. **Jerk Cost**: Red - `weight_jerk * ||Δaccel||²`
5. **Total Cost**: Black (thick line on top)

**Implementation Note**:
- Need to store these cost components during MPC solve
- Modify `MPCController.solve()` to return cost breakdown
- Add to simulator history

**Features**:
- Rolling window: Last 200 time steps
- Legend: Right side, small font
- Grid: Enabled
- Title: "MPC Cost Function Components"
- Y-label: "Cost [-]"

---

## 4. Additional MPC Data to Track

To implement the right panel visualizations, we need to store additional data during simulation:

### 4.1 Modify `MPCController.solve()` Return Value

**Current**: Returns `np.ndarray` (acceleration command)

**New**: Returns `dict` containing:
```python
{
    "accel_cmd": np.ndarray,          # The acceleration command
    "predicted_pos": np.ndarray,       # X.value (3 x N+1) - predicted positions
    "predicted_vel": np.ndarray,       # V.value (3 x N+1) - predicted velocities
    "predicted_accel": np.ndarray,     # A.value (3 x N) - predicted accelerations
    "ref_pos": np.ndarray,             # x_ref (3 x N+1) - reference positions
    "ref_vel": np.ndarray,             # v_ref (3 x N+1) - reference velocities
    "cost_total": float,               # Total optimization cost
    "cost_pos": float,                 # Position error cost
    "cost_vel": float,                 # Velocity error cost
    "cost_accel": float,               # Acceleration cost
    "cost_jerk": float,                # Jerk cost
    "solver_status": str,              # "optimal", "infeasible", etc.
}
```

### 4.2 Extend Simulator History

Add new fields to track MPC internals:
```python
history = {
    # ... existing fields ...
    "mpc_pred_pos": [],       # Store MPC predicted positions (3 x N+1)
    "mpc_ref_pos": [],        # Store MPC reference positions (3 x N+1)
    "mpc_cost_pos": [],       # Position cost component
    "mpc_cost_vel": [],       # Velocity cost component
    "mpc_cost_accel": [],     # Acceleration cost component
    "mpc_cost_jerk": [],      # Jerk cost component
    "mpc_cost_total": [],     # Total cost
    "desired_accel": [],      # Desired acceleration from MPC
}
```

---

## 5. Performance Optimization Strategies

### Problem Analysis
The current visualization is laggy because:
1. **Too many redraws**: Every frame updates all plot elements
2. **3D rendering overhead**: Matplotlib 3D is computationally expensive
3. **Data volume**: Long trajectories create many points to render
4. **Inefficient updates**: Not using `set_data()` / `set_3d_properties()` efficiently
5. **No downsampling**: Rendering every single data point

### Optimization Strategies

---

#### 5.1 **Reduce Update Frequency** (Easy Win)
- **Current**: Updates every frame (50 fps if dt=0.02)
- **Proposed**: Update visualization every N simulation steps
  - Default: Update every 2 steps (25 fps)
  - Configurable: Add `viz_update_interval` to config
  - Implementation:
    ```python
    def animate(i):
        actual_idx = i * viz_update_interval
        if actual_idx >= len(time):
            return artists
        # ... update with history[actual_idx] ...
    ```

**Impact**: 2x speedup immediately

---

#### 5.2 **Downsample Trail Data** (Moderate)
- **Current**: Renders up to 500 trail points
- **Proposed**: Adaptive downsampling based on velocity
  - Keep more points during turns/maneuvers
  - Skip points during straight flight
  - Algorithm: Douglas-Peucker line simplification
  - Alternative: Fixed stride (every Nth point)
  
**Implementation**:
```python
def downsample_trail(positions, max_points=200):
    if len(positions) <= max_points:
        return positions
    stride = len(positions) // max_points
    return positions[::stride]
```

**Impact**: 2-3x rendering speedup for trails

---

#### 5.3 **Blitting** (Advanced)
- **Technique**: Only redraw changed artists, not entire figure
- Matplotlib's `FuncAnimation` supports `blit=True`
- **Requirements**:
  - Separate static elements (trajectory, ground) from dynamic (rocket, trail)
  - Return only changed artists from update function
  
**Implementation**:
```python
anim = FuncAnimation(
    fig, 
    animate, 
    frames=num_frames,
    interval=interval_ms,
    blit=True,  # ← Enable blitting
    repeat=False
)
```

**Impact**: 3-5x speedup, but requires code restructuring

---

#### 5.4 **Reduce Mesh Complexity** (Moderate)
- **Current**: Cylinder/cone have 12 circumferential points
- **Proposed**: Reduce to 8 points (still looks smooth)
  - Especially for close-up view where camera is fixed
  
**Change**:
```python
def _make_cylinder(radius, z0, z1, n=8):  # Was n=12
```

**Impact**: ~30% faster 3D mesh rendering

---

#### 5.5 **Pre-compute Static Elements** (Easy)
- **Current**: Ground grid and trajectory re-rendered every frame
- **Proposed**: Plot once, never update
  - Ground plane: Plot once in init
  - Full trajectory: Plot once in init
  - Mark as static (don't return from update function if using blit)

**Impact**: Small but measurable improvement

---

#### 5.6 **Use Figure Canvas Events** (Advanced)
- Disable unnecessary event handlers
- Reduce responsive resizing overhead
  
**Implementation**:
```python
fig.canvas.toolbar_visible = False
fig.canvas.header_visible = False
fig.canvas.footer_visible = False
```

**Impact**: Minor, but reduces overhead

---

#### 5.7 **Parallel Pre-computation** (Advanced)
- Pre-compute all rocket orientations before animation starts
- Store rotation matrices, mesh vertices
- Trade memory for speed

**Implementation**:
```python
# Before animation, compute all frames
precomputed_meshes = []
for i in range(len(time)):
    mesh = compute_rocket_mesh(attitude[i], gimbal[i], ...)
    precomputed_meshes.append(mesh)

# During animation, just set data
def animate(i):
    rocket_mesh.set_data(precomputed_meshes[i])
```

**Impact**: 10x faster animation, but uses more RAM

---

#### 5.8 **Progressive Loading** (UX Improvement)
- Show animation while simulation is still running
- Don't wait for full simulation to complete
  
**Implementation**:
- Run simulation in separate thread
- Update `history` dict in real-time
- Animation reads from shared history
- Requires thread-safe data structure

**Impact**: Better perceived performance

---

### Recommended Implementation Order

**Phase 1** (Quick Wins - Do First):
1. ✅ Reduce update frequency (every 2 steps)
2. ✅ Downsample trail data
3. ✅ Pre-compute static elements (ground, full trajectory)
4. ✅ Reduce mesh complexity (n=8)

**Phase 2** (Medium Effort):
5. Implement blitting for dynamic elements
6. Add configurable quality settings (low/medium/high)

**Phase 3** (Advanced - If Still Needed):
7. Pre-compute all meshes
8. Progressive loading with threading

---

## 6. Configuration Changes

Add to `VisualizationConfig`:

```python
@dataclass
class VisualizationConfig:
    trail_length: int = 500
    axis_margin: float = 5.0
    
    # NEW: Performance settings
    update_interval: int = 2          # Update every N sim steps
    trail_downsample_stride: int = 2  # Keep every Nth trail point
    mesh_quality: int = 8             # Circumferential points (8/12/16)
    use_blitting: bool = True         # Enable blitting optimization
    
    # NEW: Layout settings
    show_rocket_closeup: bool = True
    show_mpc_internals: bool = True
    mpc_horizon_projection: str = "xz"  # "xy" or "xz"
```

---

## 7. Implementation Checklist

### Step 1: Extend MPC Controller
- [ ] Modify `MPCController.solve()` to return dict with cost breakdown
- [ ] Add cost component calculations
- [ ] Store solver status

### Step 2: Extend Simulator
- [ ] Update `RocketSimulator.step()` to store MPC data
- [ ] Add new history fields
- [ ] Handle MPC dict return value

### Step 3: Create Rocket Close-Up View
- [ ] Add new subplot in left panel
- [ ] Implement fixed camera angle
- [ ] Render rocket body
- [ ] Add thrust vector stick with color gradient
- [ ] Add gimbal deflection cone
- [ ] Add body Z-axis reference
- [ ] Add coordinate frame triad
- [ ] Add text annotations

### Step 4: Create MPC Internals Views
- [ ] Add position error plot (top right)
- [ ] Add MPC horizon projection plot (middle right)
- [ ] Add cost components plot (bottom right)
- [ ] Implement proper data windowing (last 200 steps)

### Step 5: Performance Optimizations
- [ ] Implement update interval
- [ ] Add trail downsampling
- [ ] Pre-compute static elements
- [ ] Reduce mesh complexity
- [ ] Add configuration options
- [ ] Test and measure performance gains

### Step 6: Testing & Refinement
- [ ] Test with all three trajectory types
- [ ] Verify MPC data accuracy
- [ ] Check layout on different screen sizes
- [ ] Ensure labels/legends are readable
- [ ] Add speed control buttons for new layout

---

## 8. Expected Results

### Visual Improvements
- **Better understanding**: Students can see MPC prediction horizon and cost components
- **Detailed attitude view**: Clear visualization of gimbal deflection and body orientation
- **Educational value**: Professor can explain control theory concepts using live data

### Performance Improvements
- **Target**: 5-10x faster animation
- **From**: ~5 fps with lag → **To**: ~25-30 fps smooth
- **Memory**: Slight increase (acceptable for visualization tool)

### User Experience
- Professional-looking 3-panel layout
- Interactive speed controls still functional
- Clear, labeled visualizations
- Suitable for presentations and demonstrations

---

## 9. Future Enhancements (Beyond This Phase)

1. **Interactive MPC Tuning**:
   - Sliders to adjust weight_pos, weight_vel, etc. in real-time
   - Re-run simulation with new parameters
   
2. **Constraint Visualization**:
   - Show acceleration constraints as shaded regions
   - Highlight when constraints are active/saturated
   
3. **Multi-Trajectory Comparison**:
   - Overlay multiple simulation runs
   - Compare different controller gains
   
4. **Export Capabilities**:
   - Save animation as MP4 video
   - Export plots as PNG/SVG
   - Save data as CSV for analysis
   
5. **3D Path Tube**:
   - Show acceptable error tolerance as tube around trajectory
   - Color-code when rocket is inside/outside tolerance

---

## Notes for Implementation

- Keep all changes modular and configurable
- Add docstrings to new functions
- Update `software_overview.md` after implementation
- Test performance on different machines
- Consider making some features optional (toggle buttons)

This design maintains backward compatibility while adding significant educational and visual value to the project.
