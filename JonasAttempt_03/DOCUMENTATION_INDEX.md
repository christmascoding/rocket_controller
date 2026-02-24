# Documentation Index & Navigation Guide

Welcome to the 6-DOF Rocket Landing Controller documentation. This index will help you find exactly what you need.

---

## Quick Navigation

### 🚀 New to the Project?

Start here: [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md)
- Complete system overview
- Architecture explanation  
- How all phases work together
- Physics model fundamentals

### 💻 Want to Run It?

Follow: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → Section "Quick Start"
```bash
cd JonasAttempt_03
python run_demo.py
```

### 🔧 Need to Modify Parameters?

See: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → Section "Modifying Parameters"

Then cross-reference: [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) for detailed explanations

### 📚 Deep Dive: Phase 1c Innovation

Read: [PHASE_1C_TECHNICAL_ANALYSIS.md](PHASE_1C_TECHNICAL_ANALYSIS.md)
- Why vector-based control was needed
- Mathematical derivation
- Comparison with traditional angle-based control
- Convergence analysis

### 🐛 Debugging Issues?

Consult: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → Section "Debugging Checklist"

---

## Document Structure

### [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) - Complete Reference (2500+ lines)

**Purpose**: Comprehensive technical documentation of entire system

**Contains**:
1. **System Overview** - Key features, technical specs
2. **Architecture Overview** - File structure, control flow
3. **File Structure & Modules** - Description of each Python file with line counts and key functions
4. **Physics Model** - Coordinate frames, DCM, gimbal mechanics, aerodynamics
5. **Control Phases** - Detailed explanation of all 5 phases
6. **Detailed Controller Implementations** - Code snippets with explanations
7. **Configuration Parameters** - All tunable values with effects
8. **Visualization System** - How 3D animation works
9. **Running the Simulation** - Execution instructions
10. **Troubleshooting** - Common issues and fixes

**Read This If**:
- You want complete understanding of system
- You need to implement new features
- You're modifying control laws
- You need to understand physics calculations

---

### [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Developer Cheat Sheet (1000+ lines)

**Purpose**: Fast lookup guide for common tasks

**Contains**:
1. **Quick Start** - How to run simulation in 3 lines
2. **Key Files & Their Roles** - Table of all Python files with purposes
3. **Core Simulation Loop** - Pseudocode for main execution
4. **Phase Transition Logic** - State machine diagram
5. **Modifying Parameters** - Most common tuning knobs
6. **Debugging Checklist** - Troubleshooting flowchart
7. **Common Control Law Implementations** - Templates for new controllers
8. **State Vector Breakdown** - What each element means
9. **Physics Quick Reference** - Equations and calculations
10. **Testing & Validation** - How to verify correctness
11. **Performance Tuning** - Dial in landing precision
12. **Known Limitations** - What's not implemented
13. **Key References in Code** - Where to find specific features

**Read This If**:
- You want fast answers
- You're modifying parameters
- You're debugging specific issues
- You want example code templates
- You're tuning controller gains

---

### [PHASE_1C_TECHNICAL_ANALYSIS.md](PHASE_1C_TECHNICAL_ANALYSIS.md) - Deep Dive (1500+ lines)

**Purpose**: Detailed analysis of the innovative vector-based gimbal control

**Contains**:
1. **Problem Statement** - Why angle-based control failed
2. **Root Cause Analysis** - ±180° discontinuity issue
3. **Solution: Vector-Based Control** - Mathematical derivation
   - Step 1: Represent as unit vectors
   - Step 2: Compute error angle (no wrapping)
   - Step 3: Compute rotation axis
   - Step 4: Compute desired angular velocity
   - Step 5: PD control on angular velocity
4. **Implementation Details** - Full code with comments
5. **Why This Works** - Detailed analysis without discontinuities
6. **Gimbal Authority Analysis** - Ensure commands are achievable
7. **Convergence Behavior** - Typical flip trajectory
8. **Failure Modes & Recovery** - Singular cases, timeouts
9. **Comparison: Old vs. New** - Side-by-side analysis
10. **Extension: Generalized Attitude Control** - Use for any orientation
11. **Numerical Stability Notes** - Floating-point considerations
12. **Real-World Validation Metrics** - Test criteria

**Read This If**:
- You want to understand Phase 1c deeply
- You're implementing similar control laws
- You want mathematical rigor
- You're explaining the system to others
- You're considering similar approaches for other problems

---

### [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) - Parameters Bible (800+ lines)

**Purpose**: Single source of truth for all configuration values

**Contains**:
1. **Physical Constants** - Gravity, air density, timestep
2. **Rocket Geometry & Mass Properties** - Dimensions, mass, thrust
3. **Moments of Inertia** - Detailed calculation with explanation
4. **Aerodynamics** - Lift/drag coefficients, CP offsets
5. **Control Limits** - Gimbal, throttle, torque limits
6. **Phase 1a/1b Parameters** - Guidance, control gains, thresholds
7. **Phase 1c Parameters** - Gimbal control specific
8. **Phase 2 Parameters** - Aerodynamic control
9. **Phase 3 Parameters** - LQR, throttle, engagement logic
10. **Trajectory Planning** - Reference values
11. **Launch Phase Limits** - Safety constraints
12. **Summary Table** - One-page reference of all parameters

**Read This If**:
- You need to change a parameter
- You want to understand what a value does
- You're tuning for different rocket properties
- You're building a parameter optimization loop
- You want default values for a similar system

---

## How Parameters Flow Through System

```
config.py (stores all values)
    ↓
cascaded.py (controllers read from config)
    ↓ [compute control commands]
    ↓
simulator.py (applies controls, updates state)
    ↓ [state_derivative called per timestep]
    ↓
dynamics.py (uses config for inertia, thrust limits)
    ↓ [integrate state forward]
    ↓
plotter.py (visualizes results)
```

To tune behavior:
1. Identify which phase needs adjustment
2. Find relevant parameters in [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md)
3. Understand effect in [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md)
4. Learn typical values in [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
5. Check for side effects in [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Performance Tuning"

---

## Common Workflows

### Workflow 1: Land Closer to 0.1m Target

1. Check current results in `console output`
2. If landing at Z > 0.3m:
   - Go to [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Landing Precision"
   - Increase safety factor in `phase3_controller()`
   - Re-run simulation
3. If landing velocity too high:
   - Increase Phase 3 LQR gains in [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md)
   - See impact explanation in [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → Phase 3 section

### Workflow 2: Understand How Phase 1c Works

1. Read [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Control Phases" → "Phase 1c"
2. For mathematical depth, read [PHASE_1C_TECHNICAL_ANALYSIS.md](PHASE_1C_TECHNICAL_ANALYSIS.md)
3. Find specific lines in code:
   - [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Key References in Code"
   - Points to `cascaded.py` lines ~175-280

### Workflow 3: Implement New Controller for Phase 1a

1. Find template in [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Common Control Law Implementations"
2. Check phase 1a architecture in [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Phase 1a"
3. Reference existing phase1a code in `cascaded.py`
4. Check parameter names in [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md)
5. Test and tune following [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Testing & Validation"

### Workflow 4: Debug Phase 1c Not Executing

1. Follow [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Debugging Checklist" → "Phase 1c Not Executing?"
2. Check `simulator.py` code around apogee detection
3. Verify phase transition conditions in [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md)
4. Enable debug prints and check console output

### Workflow 5: Optimize for Different Rocket

1. Modify rocket properties:
   - Edit `config.py` dimensions, mass, thrust
   - Recalculate inertias (use [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) formulas)
2. Recompute trajectory:
   - Use trajectory planning algorithm
   - Update TRAJ_X_REF, TRAJ_Z_REF
3. Re-tune controller gains:
   - See [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Performance Tuning"
   - Use [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) as baseline
4. Test with [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Running the Simulation"

---

## File Cross-References

### Parameters by Phase

**Phase 1a/1b** (Ascent & Coast):
- Parameters: [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "Phase 1a/1b"
- Algorithm: [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Control Phases" → "Phase 1a/1b"
- Code: `cascaded.py` functions `pure_pursuit()`, `outer_loop()`, `middle_loop()`, `inner_loop()`
- Quick tips: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Phase 1a/1b Control"

**Phase 1c** (Powered Flip):
- Parameters: [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "Phase 1c"
- Algorithm: [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Control Phases" → "Phase 1c"
- Deep analysis: [PHASE_1C_TECHNICAL_ANALYSIS.md](PHASE_1C_TECHNICAL_ANALYSIS.md) (entire document)
- Code: `cascaded.py` function `phase1c_controller()` (lines ~175-280)
- Quick ref: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Phase 1c Gimbal Authority"

**Phase 2** (Descent):
- Parameters: [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "Phase 2"
- Algorithm: [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Control Phases" → "Phase 2"
- Code: `cascaded.py` function `phase2_flip_controller()` (lines ~150-175)

**Phase 3** (Hover-Slam):
- Parameters: [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "Phase 3"
- Algorithm: [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Control Phases" → "Phase 3"
- Code: `cascaded.py` functions `phase3_controller()`, `lqr_gain()` (lines ~280-410)
- Quick tips: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Phase 3 Descent Time"

---

## Physics Reference

| Concept | Document | Section |
|---------|----------|---------|
| Coordinate frames | [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) | "Physics Model" |
| Direction Cosine Matrix (DCM) | [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) | "Key Physics Relations" |
| Gimbal-to-torque conversion | [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) | "Key Physics Relations" |
| Aerodynamic forces | [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) | "Physics Model" |
| State derivative equation | [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) | "dynamics.py" |
| Vector-based angle error | [PHASE_1C_TECHNICAL_ANALYSIS.md](PHASE_1C_TECHNICAL_ANALYSIS.md) | "Mathematical Foundation" |
| LQR formulation | [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) | "Phase 3" → "Level 2: LQR" |
| Gimbal angle limits | [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) | "Control Limits" → "Gimbal Angle Limits" |

---

## Key Insights for Each Document

### SYSTEM_DOCUMENTATION.md
- **Best for**: Complete understanding, implementation of new features, detailed explanation
- **Length**: ~2500 lines, comprehensive
- **Reading time**: 1-2 hours full, 20 min per section
- **Style**: Detailed, technically rigorous, with code examples

### QUICK_REFERENCE.md
- **Best for**: Fast lookup, common tasks, debugging
- **Length**: ~1000 lines, scannable
- **Reading time**: Minute-to-minute lookup, 5 min per section
- **Style**: Tables, checklists, quick examples

### PHASE_1C_TECHNICAL_ANALYSIS.md
- **Best for**: Understanding gimbal control innovation, mathematical rigor
- **Length**: ~1500 lines, specialized
- **Reading time**: 1 hour focused study
- **Style**: Mathematical, problem-solution approach

### PARAMETER_REFERENCE.md
- **Best for**: Understanding what each parameter does, tuning
- **Length**: ~800 lines, reference style
- **Reading time**: Lookup-based, 2 min per parameter
- **Style**: Concise definitions, units, effects

---

## Examples by Topic

### "How do I change landing altitude?"
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Landing Precision"
- [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "PHASE3_IGNITION_SAFETY_MARGIN"
- [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Phase 3" → "Level 1: Vertical Thrust Control"

### "What does Kp_pos do?"
- [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "Outer Loop (Position Error → Acceleration)"
- [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Phase 1a/1b" → "Control Architecture"
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "State Vector Breakdown"

### "Why is Phase 1c using vector-based control?"
- [PHASE_1C_TECHNICAL_ANALYSIS.md](PHASE_1C_TECHNICAL_ANALYSIS.md) → "Problem Statement" & "Root Cause Analysis"
- [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Phase 1c"
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "Key Files & Their Roles" → `cascaded.py`

### "How do I implement LQR?"
- [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Phase 3" → "Level 2: LQR Attitude Stabilization"
- [PARAMETER_REFERENCE.md](PARAMETER_REFERENCE.md) → "LQR State Feedback"
- Code: `cascaded.py` → `lqr_gain()` function

### "What happens at landing?"
- [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) → "Phase 3" → "Landing Detection"
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) → "State Vector Breakdown"
- Code: `simulator.py` → landing detection logic

---

## File Naming Convention

All documentation files use descriptive names:
- `SYSTEM_DOCUMENTATION.md` - Comprehensive reference
- `QUICK_REFERENCE.md` - Fast lookup guide
- `PHASE_1C_TECHNICAL_ANALYSIS.md` - Deep dive on Phase 1c
- `PARAMETER_REFERENCE.md` - Configuration parameters
- `README.md` (in parent dirs) - Project overview (not part of this documentation set)

---

## Updates & Maintenance

**Last Updated**: Session 6 (Gimbal Visualization Fix & Complete Documentation)

**Documentation Version**: 1.0 Complete

**Status**: All major features documented, ready for external use

**Future Additions** (if implemented):
- Wind disturbance modeling
- Propellant mass dynamics
- Advanced LQR with integral action
- MPC alternative to LQR
- Sensor simulation & filtering
- Structural load analysis

---

## How to Use This Index

1. **Find your question** in the list above
2. **Click the link** to the relevant document
3. **Use Ctrl+F** to search within document for specific terms
4. **Cross-reference** using the tables provided
5. **Refer to code** using line numbers from Quick Reference tables

---

**Welcome to the Complete 6-DOF Rocket Landing Controller Documentation!**

