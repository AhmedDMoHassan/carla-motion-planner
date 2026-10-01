# Motion Planning for a Self-Driving Car in CARLA

**Final project, *Motion Planning for Self-Driving Cars*, University of Toronto Self-Driving Cars Specialization (Coursera).**

A complete local motion-planning stack that drives a car through a CARLA scenario with a parked car, a lead vehicle and a stop sign. It tracks the lane centre, swerves around static obstacles, follows a slower car, and comes to a full stop at the stop sign. It runs closed loop, end to end, at a fixed 30 Hz.

> 🎥 **Demo video:** *(add link)*
> ✅ **Result:** passed both graded checks: **0 collisions**, and waypoint deviation plus stop-sign stop within the required window.

---

## What the planner does

| Scenario event | Planner response |
|---|---|
| Parked car blocking the lane | Generates 7 laterally offset candidate paths, rejects the ones that collide, and picks a smooth path around the car and back to the centreline |
| Slower lead vehicle | Switches to a following velocity profile that keeps a time gap behind it |
| Stop sign | Decelerates to a complete stop before the stop line, waits, then continues and turns |

## Architecture

```
          global waypoints (route)
                     │
        ┌────────────▼────────────┐
        │  Behavioural planner    │  state machine: FOLLOW_LANE → DECELERATE_TO_STOP
        │  (decides WHAT to do)   │                 → STAY_STOPPED → FOLLOW_LANE
        └────────────┬────────────┘  + lead-vehicle following decision
                     │ goal state (x, y, v)
        ┌────────────▼────────────┐
        │  Local planner          │  lateral goal set → cubic-spiral paths
        │  (decides the GEOMETRY) │  → circle-based collision check → path selection
        └────────────┬────────────┘
                     │ best path σ(s)
        ┌────────────▼────────────┐
        │  Velocity planner       │  nominal / follow-lead / decelerate-to-stop
        │  (adds TIME / SPEED)    │  constant-acceleration profiles
        └────────────┬────────────┘
                     │ trajectory (x, y, v)
        ┌────────────▼────────────┐
        │  Controller             │  longitudinal PID + Stanley-style lateral control
        └────────────┬────────────┘
                     ▼
                CARLA vehicle
```

The behaviour and local planners run at **15 Hz**; the controller runs at the **30 Hz** simulation step.

## Implementation highlights

**Behavioural planning:** `behavioural_planner.py`
- Closest-waypoint search, and a goal point chosen by accumulated arc length with a speed-dependent lookahead (8 m + 2 s × v).
- A three-state finite-state machine for stop-sign handling. The stop is detected by intersecting the planned waypoint segments with the stop line.

**Path generation:** `local_planner.py`, `path_optimizer.py`
- Goal state transformed into the vehicle frame (translate, then rotate by −ψ). Seven goals offset perpendicular to the goal heading.
- Cubic spiral paths κ(s) = a + bs + cs² + ds³, found as a two-point boundary-value problem with **L-BFGS-B**. The curvature knots are bounded to ±0.5 m⁻¹ and the arc length is bounded below by the straight-line distance.
- Heading θ(s) is integrated analytically; x and y come from cumulative trapezoidal integration of cos θ and sin θ.

**Collision checking and path selection:** `collision_checker.py`
- The vehicle footprint is approximated by three circles placed along its heading at every path point.
- Score = distance to the centreline goal + a decaying penalty for ending near a colliding path. This pushes the selection away from the obstacle and keeps a safety margin.

**Velocity profile:** `velocity_planner.py`
- Constant-acceleration kinematics (v_f² = v_i² + 2ad) for the ramps: decelerate-to-stop with a stop-line buffer, lead-vehicle following, and nominal cruising.

## Tuning note: sim-to-model mismatch

My first graded run stopped **0.4 m before** the required stop window. The cause was not the logic. The controller never commands the brake, and CARLA's vehicle slows down harder while coasting than the planner's assumed 1.5 m/s². Reducing `STOP_LINE_BUFFER` from 3.5 m to 1.5 m moved the planned stop point to the middle of the window, and the run passed. This was a good reminder that a planner is only as good as its assumptions about the vehicle below it.

## Running it

**Requirements:** the course's modified **CARLA 0.8.4** binary, and **Python 3.6 (64-bit)** with:
```
numpy==1.19.5  scipy==1.5.4  matplotlib==2.2.5  Pillow==8.4.0
protobuf==3.19.6  pygame==2.1.2  future==0.18.3
```
`matplotlib` must stay on 2.2.x, because the course's `live_plotter.py` imports `matplotlib.backends.tkagg`, which was removed in later versions.

```bash
# Terminal 1: CARLA server (fixed 30 Hz step)
CarlaUE4.exe /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30

# Terminal 2: the planner client
cd PythonClient/Course4FinalProject
python module_7.py
```
Outputs land in `controller_output/`: `trajectory.txt`, `collision_count.txt` and the plots.

## Repository contents

- `results/`: trajectory, speed, throttle, brake and steer plots, and the demo video
- `docs/`: architecture notes
<!-- - `Course4FinalProject/`: planner source (see the note on academic integrity below) -->

> **Academic integrity:** this project is part of a graded Coursera course. To respect the course's honor code, the solution source for the graded TODOs is not published here. Happy to discuss the approach.

## Related

I'm also publishing a **GNC guidance-layer pipeline guide**: the study map I built while working through this course. It walks one car mission and one drone mission through every layer (mission → behaviour → local planning → velocity profile → control) and maps each layer to Python, MATLAB/Simulink and industry practice. *(link coming soon)*

---

**Ahmed Mohamed Ahmed Hassan**, Aerospace / GNC Engineer · [LinkedIn](https://www.linkedin.com/in/ahmedhassan2002)
Course: [Self-Driving Cars Specialization, University of Toronto](https://www.coursera.org/specializations/self-driving-cars)
