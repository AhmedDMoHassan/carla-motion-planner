**GNC** · Guidance layerSelf-driving car ⟷ QuadrotorAfter U. Toronto · Motion Planning for Self-Driving Cars

# The Guidance Stack — one mission, every layer

Global route, behaviour, local planning, velocity profile, reactive avoidance and the handoff to control, traced through one running example for a car and a drone. Each layer lists its inputs, outputs, update rate, figure, and one video.

By **Ahmed Mohamed Ahmed Hassan**, Aerospace Engineer · Version 1.0 · Last reviewed 19 September 2026 · [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/): free to use and share with credit

Example

Depth

The stack

1. [GOStart here](#s-start)
2. [A–ZGlossary](#s-gloss)
3. [00What guidance is](#s-frame)
4. [MAPMind map](#s-map)
5. [EXThe mission](#s-mission)
6. [∑At a glance](#s-glance)
7. [ZOOMFive zooms · car](#s-zoom)
8. [✈×2Quad vs fixed-wing](#s-qfw)
9. [L0World model](#s-l0)
10. [L1Global / mission](#s-l1)
11. [L2Behaviour](#s-l2)
12. [L3Local planning](#s-l3)
13. [L4Velocity profile](#s-l4)
14. [L5Handoff to control](#s-l5)
15. [×Cross-cutting](#s-cross)
16. [⟳Replanning loop](#s-loop)
17. [?Which layer broke?](#s-debug)
18. [PYBuild it in Python](#s-python)
19. [▶Study path](#s-study)
20. [SLCoursera → Simulink](#s-simulink)
21. [INDTools in industry](#s-industry)
22. [©Author & licence](#s-credits)

GO

## Start here

[#](#s-start)

This page explains the guidance layer of a GNC system as one connected story: from the mission route down to the commands the controller receives. It follows one car mission and one drone mission through every layer, and maps everything to Python, MATLAB and Simulink. Pick the route that matches your background:

#### New to guidance

1. [What guidance is](#s-frame)
2. [The running example](#s-mission)
3. [Five zooms · car](#s-zoom)
4. [L1](#s-l1) → [L5](#s-l5), one layer at a time
5. [Study path](#s-study), beginner column

#### Aerospace / GNC engineer

1. [Aerospace ⟷ self-driving vocabulary](#s-frame)
2. [The stack at a glance](#s-glance)
3. [Quadcopter vs fixed-wing](#s-qfw)
4. [L4](#s-l4) (V–n, g-g) and [L5](#s-l5) (L1, PN)
5. [Replanning loop](#s-loop)

#### MBD / Simulink user

1. [The stack at a glance](#s-glance)
2. [Coursera → Simulink mapping](#s-simulink)
3. [Python / C++ verdicts](#s-simulink) per module
4. [How industry splits the tools](#s-industry)
5. [Build it in Python](#s-python) for the reference implementation

Using the page

- **Example** buttons (top) show the car example, the drone example, or both side by side.
- **Depth** buttons hide the dashed “Advanced” boxes for a first read.
- Each section heading has a # link that jumps straight to it. The list on the left (or at the top on a phone) jumps to any section.
- Wide tables and diagrams scroll sideways on a phone.
- To save a copy, use your browser's Print → Save as PDF. The page has a print layout that shows every section in full.

### Conventions

| Item | Convention used on this page |
| --- | --- |
| **Units** | SI throughout: m, s, m/s, m/s², rad. Degrees are marked with °. Road speeds are sometimes also given in km/h, and are labelled. |
| **Car frames** | World frame ENU (x east, y north, z up); body frame x forward, y left. Yaw ψ measured from x, counter-clockwise positive. Curvature κ > 0 for a left turn. |
| **Aircraft frames** | World frame NED (x north, y east, z down), as in PX4 and flight mechanics; body frame FRD (x forward, y right, z down). Bank φ positive right wing down. |
| **Frenet frame** | s = arc length along the reference line; d = lateral offset, positive to the left. |
| **Notation** | x̂ = estimated state; x_ref(t) = reference trajectory; σ(s) = path; L0–L5 = the layer numbers of this page. All abbreviations are in the [glossary](#s-gloss). |
| **Numbers** | Rates, horizons and limits are typical order-of-magnitude values for teaching. Real vehicles are tuned case by case. |
| **Software names** | MATLAB/Simulink block, function, toolbox and app names follow the MathWorks documentation as of September 2026. They can change between releases, so check any name with `doc <name>` in your MATLAB version. |
| **External links** | Videos and references are third-party pages. They were checked when this version was published but may move over time. |

A–Z

## Glossary of abbreviations

[#](#s-gloss)

Every abbreviation and symbol used on this page. Type to filter.

Filter

#### Architecture & layers

GNC

Guidance, Navigation and Control: the three functions of an autonomous vehicle's flight/driving software.

L0–L5

This page's layer numbers: L0 world model, L1 mission, L2 behaviour, L3 local planning, L4 velocity profile, L5 control handoff.

FSM / HFSM

Finite-State Machine / Hierarchical FSM: states plus guarded transitions; hierarchical = states inside states (Stateflow style).

BT

Behaviour Tree: a tree of conditions and actions, an alternative to FSMs for large decision logic.

MDP / POMDP

(Partially Observable) Markov Decision Process: decision-making when outcomes, or the state itself, are uncertain.

RTL

Return To Launch: the drone flies home and lands, usually a failsafe or low-battery action.

AEB

Automatic Emergency Braking: the last reactive safety layer on a car.

CBF

Control Barrier Function: a safety filter that minimally edits a command so the vehicle stays in a safe set.

Simplex / RTA

Run-Time Assurance: a verified simple fallback controller takes over when the advanced stack misbehaves.

#### Planning algorithms

A\*

Graph search that expands nodes in order of cost-so-far + heuristic estimate to goal. Optimal if the heuristic never overestimates.

Dijkstra

A\* with a zero heuristic: explores uniformly outward.

D\* Lite / LPA\*

Incremental versions of A\* that repair the previous search when edge costs change.

Hybrid A\*

A\* over (x, y, heading) with vehicle-feasible motion primitives; for parking and unstructured spaces.

RRT / RRT\*

Rapidly-exploring Random Tree: grows a tree by random sampling. RRT\* rewires the tree and converges to the optimal path.

PRM

Probabilistic Roadmap: random samples connected into a reusable graph.

TSP

Travelling Salesman Problem: visit all points in the cheapest order; used for survey and inspection missions.

DWA

Dynamic Window Approach: reactive planner sampling (v, ω) pairs reachable within one control step.

APF

Artificial Potential Field: goal attracts, obstacles repel; fast but has local minima.

VFH

Vector Field Histogram: reactive avoidance from a polar histogram of obstacle density.

BVP

Boundary-Value Problem: find a curve meeting conditions at both ends (e.g. a cubic spiral from ego pose to goal pose).

Frenet (s, d)

Road-aligned coordinates: s = distance along the reference line, d = lateral offset from it.

Dubins path

Shortest path for a vehicle with a minimum turn radius that can only drive forward: arcs and straight lines.

Min-snap

Polynomial trajectory minimizing the 4th derivative of position; standard for quadrotors.

CHOMP / TrajOpt / iLQR

Trajectory-optimization methods that refine an initial path by minimizing smoothness, collision and dynamics costs.

TOPP

Time-Optimal Path Parameterization: the fastest speed profile along a fixed path within limits.

QP

Quadratic Program: optimization with a quadratic cost and linear constraints; solved fast and reliably.

#### Guidance & control laws

PID / PI

Proportional-Integral-Derivative feedback controller.

LQR

Linear Quadratic Regulator: optimal state-feedback gain for a linear model.

MPC / NMPC

(Nonlinear) Model Predictive Control: solves an optimal control problem over a short horizon every step, with constraints.

Pure pursuit

Steers toward a point a lookahead distance ahead on the path.

Stanley

Steering law combining heading error and cross-track error (from Stanford's DARPA car).

L1 guidance

Fixed-wing lateral guidance law: a_cmd = 2V²/L₁·sin η, converted to a bank-angle command.

PN

Proportional Navigation: missile guidance a = N·V_c·λ̇ (line-of-sight rate).

TECS

Total Energy Control System: fixed-wing longitudinal control; throttle manages total energy, pitch manages its split between speed and height.

IDM

Intelligent Driver Model: closed-form car-following acceleration law.

Feed-forward (FF)

The part of a command computed from the reference (e.g. planned acceleration), not from the error.

#### Estimation, sensing & maps

x̂ / P

Estimated state and its covariance (uncertainty) from the estimator.

KF / EKF / ES-EKF / UKF

Kalman Filter; Extended KF (linearized); Error-State EKF (filters the error, used for IMU/quaternion INS); Unscented KF.

INS / IMU

Inertial Navigation System / Inertial Measurement Unit (gyros + accelerometers).

GNSS / GPS

Global Navigation Satellite System; GPS is the US constellation.

VIO / SLAM

Visual-Inertial Odometry / Simultaneous Localization And Mapping: how to navigate without GPS.

HD map

High-definition map with lane geometry, rules and stop lines at centimetre level.

Occupancy grid / costmap

2-D grid of cells marked free/occupied (costmap: with a cost for proximity).

OctoMap / TSDF / ESDF

3-D voxel map; Truncated / Euclidean Signed Distance Field: distance to nearest obstacle at every point.

TTC

Time-To-Collision = gap / closing speed.

ENU / NED

East-North-Up / North-East-Down frames (cars and ROS typically use ENU; aircraft and PX4 use NED).

#### Vehicle & flight terms

κ, ψ, δ

Path curvature (1/radius), heading (yaw) angle, steering angle.

s, v, a, j

Arc length, speed, acceleration, jerk (and snap = 4th derivative).

AGL

Above Ground Level (altitude).

AoA / β

Angle of attack / sideslip angle.

V_s / V_NE

Stall speed / never-exceed speed of an aircraft.

V–n diagram

An aircraft's allowed load factor n versus airspeed V: its flight envelope.

g-g diagram

A car's friction ellipse: allowed longitudinal vs lateral acceleration.

n, φ, θ

Load factor (lift/weight), bank angle, pitch/tilt angle.

Differential flatness

A quadrotor's full state and inputs follow from position p(t) and yaw and their derivatives.

#### Tools & process

MBD

Model-Based Design.

MIL / SIL / PIL / HIL

Model-, Software-, Processor-, Hardware-in-the-Loop testing stages.

SITL

Software-In-The-Loop simulation of the real autopilot firmware (PX4/ArduPilot).

PX4 / MAVLink / MAVSDK

Open-source autopilot / its message protocol / a SDK to command it.

ROS 2

Robot Operating System 2: middleware connecting planner, perception and control nodes.

CARLA

Open-source driving simulator used by the Toronto course.

DO-178C / ISO 26262

Software-certification standards for airborne systems / road vehicles.

No match.

00

## What guidance is

[#](#s-frame)

**Navigation** answers *where am I, and what is around me?* **Guidance** answers *where should I be, and at what speed, over the next few seconds to the next few hours?* **Control** answers *what actuator commands make the vehicle actually do that?*

Guidance is the part that turns an intention ("deliver the passenger", "inspect the bridge pier") into a **reference signal** the controller can track. It does this in stages. Each stage looks further ahead than the one below it, runs more slowly, and uses a simpler model of the vehicle. Each stage passes down a **goal plus constraints**, and each stage reports back up when it cannot meet them.

**FIG 0**Guidance sits between the state estimate and the controller. The dashed amber arrow is often left out of diagrams, but a guidance stack that ignores it will plan maneuvers the vehicle cannot fly.

The central idea

Each layer **narrows the solution space** for the layer below. The route rules out 99.9% of the map, the behaviour rules out most maneuvers, and the local planner picks one geometry. The velocity profile then adds time, and control makes the vehicle follow it. No layer solves the whole problem, and no layer is allowed to break the constraints it received from above.

Aerospace ⟷ self-driving vocabulary

The same ideas go by different names. In aircraft/missile GNC, a *guidance law* (pure pursuit, proportional navigation, L1) usually outputs an **acceleration or bank-angle command**. In the self-driving stack, the same job is done by a *path-tracking controller* (pure pursuit, Stanley). The algorithm is the same; the two communities put the guidance/control boundary in different places. This page puts the boundary at the **reference trajectory**: everything that produces it is guidance, and everything that tracks it is control. L5 covers the grey zone.

| Self-driving (Toronto course) | Aerospace / UAV term | Question it answers |
| --- | --- | --- |
| Mission planning | Flight plan, route, waypoint list | Which roads / which waypoints? |
| Behaviour planning | Mode logic, flight-phase manager, mission sequencer | What maneuver now? Which rules apply? |
| Local / motion planning | Trajectory generation, terrain/obstacle avoidance | What exact geometry for the next few seconds? |
| Velocity profile generation | Speed schedule, time allocation, energy management | How fast at each point along it? |
| Lateral / longitudinal control | Guidance law + autopilot inner loops | Which δ, throttle / thrust, attitude? |
| Occupancy grid, HD map | Terrain DB, geofence, voxel / ESDF map | Where is free space? |

MAP

## Mind map of the guidance layer

[#](#s-map)

Read it top to bottom: inputs, then the four guidance layers from longest horizon to shortest, then the handoff. Leaves marked ✈ are drone or aerospace additions to the car course.

**FIG 1**Obstacle avoidance and optimization are not separate branches. Both appear inside several layers; see [Cross-cutting](#s-cross). On a phone, scroll the map sideways.

EX

## The running example

[#](#s-mission)

Every layer below uses the same two missions. The car mission follows the Toronto course scenarios: lane following, a lead vehicle, a stop sign, an intersection, and a static obstacle. The drone mission applies the same stack to an aerial vehicle.

Car · Mission C

**Drive a passenger 4.2 km across town.**

- Start: curb at A. Goal: university gate B.
- 50 km/h arterial, then a 30 km/h school zone.
- Lead vehicle on the arterial, braking unpredictably.
- A 4-way stop, then a protected left turn at a signal.
- A double-parked truck blocks the right lane 200 m before B.
- A cyclist appears from behind the truck.

Quadrotor · Mission D

**Inspect a site, then return home.**

- Take off from H, transit 600 m at 40 m AGL to structure S.
- Fly a 3-waypoint inspection pass at 8 m standoff, then hover 12 s for photos.
- The final leg goes through a 3 m gap under a cable into a GPS-denied hall, where localization switches to VIO/SLAM.
- Onboard detector spots a target → divert to it, then resume.
- 7 m/s crosswind; battery reserve 30% forces RTL.

∑

## The whole stack at a glance

[#](#s-glance)

**FIG 2**A timescale cascade. Each layer runs roughly 3–10× faster than the one above it. That separation is why each layer can treat the one above as fixed and the one below as ideal.

| Layer | Input | Output | Vehicle model used | Typical algorithms | Replans when… |
| --- | --- | --- | --- | --- | --- |
| **L1 Global** | start, goal, road/waypoint graph, edge costs | ordered route (edges or waypoints) | none (a point on a graph) | Dijkstra, A\*, D\* Lite, TSP | road closed, goal changed, battery low |
| **L2 Behaviour** | route, ego state, objects + predictions, rules, health | maneuver, goal set, speed limit, stop point, corridor | kinematic point + TTC | FSM, HFSM, behaviour tree, rule costs, POMDP | every cycle; switches state only on guarded transitions |
| **L3 Local** | maneuver + constraints, costmap, predictions, ego state | path σ(s) with κ(s) | kinematic bicycle / ✈ flat outputs | rollout, DWA, lattice, hybrid A\*, RRT\*, spirals, min-snap | every cycle (receding horizon) |
| **L4 Velocity** | path κ(s), limits, lead vehicle, stop point | v(s), a(s) → trajectory x_ref(t) | point-mass with accel limits | fwd/bwd pass, trapezoid, IDM, QP / ✈ time allocation | with L3 |
| **L5 Control** | x_ref(t), x̂ | actuator commands | dynamic bicycle / ✈ rigid-body 6-DoF | pure pursuit, Stanley, PID, LQR, MPC, L1, cascaded PID | never plans; reports saturation up |

ZOOM

## Five zooms: how each layer builds on the one above

[#](#s-zoom)

Mission planning and local planning both seem to output "a route". They don't. **Mission planning outputs a list of places** (junctions or waypoints) that you cannot drive directly. **Local planning outputs a curve** with exact x, y, heading and curvature that the car can physically follow. Everything between them adds information the layer above didn't have.

The navigation-app analogy

**L1** is the directions list: "continue 800 m on Main St, turn left on Park Ave". **L2** is you deciding "that truck is double-parked, I'll go around it and get into the left lane for the turn". **L3** is the line your eyes trace on the asphalt. **L4** is how hard you press the pedal along that line. **L5** is your hands on the wheel.

Below, the same moment of Mission C is drawn five times. Each dashed amber box marks the part of that view the next view zooms into.

1

### L1 · Mission planning

≈ 4 km · once per trip

Knows about

Road connectivity, one-way streets, speed limits, travel-time costs.

Ignores

Lanes, the truck, the cyclist, the car's size and turning radius. The car is a point on a graph.

Hands down

An ordered list of junctions: “A → … → n5, turn left, → B”. Like the directions list in a navigation app.

**✈ Drone:** An ordered waypoint list H → S₁ → S₂ → S₃ → gap → hall, each with an altitude, a task and an acceptance radius.

↓ zoom in: less area, more detail, faster updates

2

### L2 · Behaviour planning

≈ 200 m · 10 Hz

Knows about

The current route edge, traffic rules, stop line, the truck and its predicted motion, the possible occluded cyclist.

Ignores

The exact geometry. It does not pick a curve, only fences off a region and sets rules for it.

Hands down

A maneuver label and constraints: allowed region, target lane, speed cap, stop point.

**✈ Drone:** Flight phase = TRANSIT, altitude band 35–45 m, geofence, standoff ≥ 8 m from the structure, yaw mode.

↓ zoom in: less area, more detail, faster updates

3

### L3 · Local / motion planning

≈ 130 m · 10 Hz

Knows about

Lane geometry, occupancy grid, the truck's footprint, the car's curvature limit κ_max and footprint circles.

Ignores

Time and speed. Every candidate is geometry only.

Hands down

One collision-free curve σ(s) = (x, y, ψ, κ): the first object in the stack you could actually draw on the road.

**✈ Drone:** A 3-D curve through free space in the ESDF, usually polynomial segments through a safe corridor.

↓ zoom in: less area, more detail, faster updates

4

### L4 · Velocity profile

same 130 m · 10 Hz

Knows about

The curve's κ(s), v_limit, the speed cap near the truck, acceleration, braking and comfort limits.

Ignores

Nothing new about geometry: the path is kept exactly as L3 gave it.

Hands down

A trajectory: position and speed as a function of time. Same line, now with a clock on it.

**✈ Drone:** Time allocation per polynomial segment so tilt and thrust stay within limits: p(t), v(t), a(t).

↓ zoom in: less area, more detail, faster updates

5

### L5 · Control

≈ 12 m · 50 Hz

Knows about

The reference at the current time, the measured state x̂ and the vehicle parameters (wheelbase).

Ignores

Everything upstream. It just tracks.

Hands down

Steering angle δ, throttle and brake: actuator commands, not a plan.

**✈ Drone:** Position → velocity → attitude → rate loops → four motor commands, at 250–1000 Hz.

### Mission route vs local path, side by side

|  | L1 mission route | L3 local path |
| --- | --- | --- |
| **Data type** | Discrete: a list of graph nodes / edges (topological) | Continuous: dense samples (x, y, ψ, κ) along arc length |
| **Spatial scale** | Whole trip (km) | Next 50–150 m |
| **Resolution** | Junction to junction (hundreds of metres) | 0.1–0.5 m |
| **Obstacles** | Only permanent ones: closed roads, no-fly zones | Everything perceived right now: truck, cyclist, cones |
| **Vehicle model** | None, a point on a graph | Footprint + curvature limit (✈ flat outputs, tilt limit) |
| **Updated** | Once, plus rare reroutes | Every 100 ms |
| **Can the vehicle follow it directly?** | No: it says "go to n5", not how | Yes, once L4 adds speed |
| **Typical algorithm** | A\*, Dijkstra on a road graph | Lattice + spirals, hybrid A\*, RRT\*, optimization |

One source of confusion: some algorithms (A\*, RRT\*) appear in both layers. What differs is *what they search over*. At L1, A\* searches a graph of junctions. At L3, hybrid A\* searches a fine grid of (x, y, heading) states with vehicle-feasible moves. Same idea, different space and scale.

✈×2

## Five zooms: quadcopter vs fixed-wing

[#](#s-qfw)

The same guidance layers applied to Mission D, flown once by a quadcopter and once by a fixed-wing UAV. The layers and their interfaces stay the same. What changes is **which constraints each layer must respect**. A quad can hover and is differentially flat; a fixed-wing must keep flying, above stall speed, turning only by banking.

QUADCOPTERFIXED-WING

1

### L1 · Mission

**Difference:** The quad's route is just points joined by straight lines. The fixed-wing's route already contains its minimum turn radius: every corner becomes an arc, inspection becomes an orbit, and take-off needs a runway pointed into wind.

2

### L2 · Behaviour / flight phases

**Difference:** Both run a Stateflow-style phase machine, but the fixed-wing has phases the quad never needs (roll, flare, go-around) and no 'stop' state: its safe default is loiter, not hover.

3

### L3 · Local planning

**Difference:** Differential flatness lets the quad plan any smooth curve and just tilt. The fixed-wing's curvature is bounded by bank angle and speed, so the geometry itself is constrained and avoidance must start much earlier.

4

### L4 · Velocity / time

**Difference:** Quad speed is limited from above only (tilt, thrust). Fixed-wing airspeed is limited from both sides, and the lower bound rises in turns by √n.

5

### L5 · Control handoff

**Difference:** The quad tracks a time-stamped trajectory through a position→attitude cascade. The fixed-wing follows a path with L1 (lateral) and TECS (longitudinal), and turns only by banking.

Summary

For a quadcopter, feasibility is mostly a question of **time**: fly the same path slower and it becomes feasible. For a fixed-wing, feasibility is a question of **geometry** (turn radius, climb gradient, runway) and of **speed band**. Slowing down is not always an option, because below the stall margin it stops flying.

L0

## World model: what guidance consumes

[#](#s-l0)

Owned by navigation / perception10–100 Hz

Guidance never looks at raw sensor data. It consumes a **world model**, and the quality of that model sets an upper bound on what any planner can do. The Toronto course covers this in *Mapping for Planning* (occupancy grids) and *Dynamic Object Interactions* (prediction, time-to-collision).

#### From sensors / estimators

- IMU, GNSS, wheel odom *→ EKF*
- LiDAR / camera / radar
- HD map, rules *(prior)*
- ✈ baro, magnetometer, VIO

#### To guidance

- x̂ = \[x, y, z, ψ, v, …\] + P *(covariance)*
- occupancy grid / costmap *(2-D)*
- object list: id, class, pose, v, predicted paths
- lanes, stop lines, signal states
- ✈ voxel map / ESDF, geofence, wind estimate

Car

The 3-D LiDAR scan is filtered (ground removed, points above the roof dropped) and projected into a 2-D occupancy grid with log-odds updates. The truck becomes occupied cells. The cyclist is a *tracked object* with a predicted path, not grid cells, because it moves.

Quadrotor

There are no lanes, so the world is a 3-D voxel map (OctoMap or a TSDF) converted to an **ESDF**, the distance to the nearest obstacle at every point. Planners use the ESDF's gradient to push paths away from obstacles. In the GPS-denied hall, x̂ comes from VIO/SLAM with a growing covariance, so guidance must add larger clearance margins there.

**Uncertainty flows into guidance.** A mature stack does not treat x̂ as the truth. It inflates obstacles by k·σ from the covariance P (chance constraints), and it plans against a *set* of predicted object trajectories (multi-modal prediction), weighting each by its probability. If you ignore this, the vehicle behaves as if it is overconfident: it passes too close and brakes late.

[▶Understanding SLAM Using Pose Graph OptimizationMATLAB · Autonomous Navigation, Part 3](https://www.youtube.com/watch?v=saVZtgPyyJQ) [FIGCourse notes and figures: The Planning ProblemToronto Course 4 · Module 1 notes](https://github.com/qiaoxu123/Self-Driving-Cars/blob/master/Part4-Motion_Planning_for_Self-Driving_Cars/Module1-The_Planning_Problem/Module1-The_Planning_Problem.md)

L1

## Global / mission planning

[#](#s-l1)

Which roads? Which waypoints?event-drivenhorizon: whole missionCourse 4 · Module 3

This layer is the high-level, "global" planner. It treats the vehicle as a **point moving on a graph** and ignores dynamics, other traffic and time. It answers one question: what is the cheapest ordered sequence of places to pass through? Because the problem is this simple, it can search the whole map, which no lower layer can afford to do.

#### Inputs

- start node, goal node
- graph G = (V, E) *road network / waypoint graph*
- edge cost w(e) *time, distance, energy, risk*
- closures, traffic *(optional, live)*

#### Outputs

- ordered route \[e₁, e₂, … eₙ\] or waypoints \[w₁ … wₙ\]
- per-segment metadata: speed limit, lane id, altitude
- ✈ also: loiter / hover / photo tasks per waypoint
- *No timing. No trajectory.*

Car · Mission C

A\* over the lanelet graph from A to B with cost = expected travel time. The heuristic is straight-line distance ÷ maximum speed, which is admissible because no road is shorter than a straight line. The output is the sequence arterial → stop-sign street → left at signal → school zone → gate. The route says "turn left at the signal". It does *not* say which lane to be in 300 m before the turn; L2 decides that.

Quadrotor · Mission D

The waypoint list H → S₁ → S₂ → S₃ → gap → hall → H. Edge cost = energy, which depends on the wind: flying upwind costs more, so the graph is *directed*. For area surveys this layer becomes a coverage or TSP problem (visit all points, in any order). Each waypoint carries a task (`HOVER 12s`, `PHOTO`) and an acceptance radius. This is the PX4 / ArduPilot *mission*.

```
# A* — the heart of L1. Nodes are road junctions or waypoints.
import heapq, math
def astar(G, start, goal, pos):
    h = lambda n: math.dist(pos[n], pos[goal])          # admissible: never overestimates
    open_ = [(h(start), 0.0, start)]; g = {start: 0.0}; parent = {}
    while open_:
        _, gc, u = heapq.heappop(open_)
        if u == goal: break
        if gc > g[u]: continue                           # stale entry
        for v, w in G[u]:                                  # w = edge cost (time/energy)
            if gc + w < g.get(v, math.inf):
                g[v] = gc + w; parent[v] = u
                heapq.heappush(open_, (g[v] + h(v), g[v], v))
    path = [goal]
    while path[-1] != start: path.append(parent[path[-1]])
    return path[::-1]
```

- **D\* Lite / LPA\*** repair the previous search when edge costs change, which matters when a road closes mid-mission. They are much cheaper than re-running A\* from scratch.
- **Contraction hierarchies** are what real navigation apps use to answer continent-scale queries in milliseconds.
- **✈ Fixed-wing**: consecutive waypoints must be joined by feasible turns, so edges become **Dubins paths** (minimum turn radius R = V²/(g·tan φ_max)). This is the first place vehicle dynamics leak into the global layer.
- **✈ Energy-aware routing**: edge cost = ∫P(V_air, wind) dt. If the remaining energy along the route drops below the reserve, L1 re-plans to RTL. That is a replan *trigger* coming from the health monitor.

[▶Path Planning with A\* and RRTMATLAB · Autonomous Navigation, Part 4](https://www.youtube.com/watch?v=QR3U1dgc5RE) [FIGPath planning overview with figuresMathWorks · Path Planning](https://www.mathworks.com/discovery/path-planning.html)

L2

## Behaviour planning

[#](#s-l2)

What maneuver, under which rules?1–10 Hzhorizon 5–20 sCourse 4 · Modules 4–5

Behaviour planning is **discrete decision-making**. It turns "follow this route" plus "here is what everyone else is doing" into one maneuver: keep lane, follow the lead car, stop at the line, yield, change lane, nudge around an obstacle. Its most important property is also the one students most often miss: **behaviour outputs constraints and a goal set, not a path.**

#### Inputs

- route from L1 *(next turn, target lane)*
- ego state x̂
- objects + predictions *→ TTC, gaps*
- rules: signals, stop signs, right-of-way, limits
- system health *(battery, sensor status)*

#### Outputs (the "maneuver message")

- maneuver id *(e.g. DECEL_TO_STOP)*
- goal set: target lane / region, lateral offsets allowed
- v_limit, stop point s_stop, lead vehicle id
- drivable corridor boundaries
- ✈ flight phase + setpoint mode + geofence

**FIG 3**The car FSM on the left follows the course's example. The drone phases on the right correspond to PX4's commander/navigator. Each arrow is a *guarded* transition, a condition that must hold before the state changes.

Car · Mission C

- **Arterial:** the lead car is within the following distance → `FOLLOW_LEADER`. Output: lead id, time gap 1.8 s, v_limit 50 km/h.
- **Stop sign:** the stop line enters the lookahead → `DECEL_TO_STOP` with s_stop = stop line − 1 m. It waits in `STOPPED` for ≥ 3 s *and* until the intersection is clear by TTC.
- **Truck:** the lane is blocked and the left lane gap is acceptable → `NUDGE`, allowing lateral offsets d ∈ \[0, 3.2\] m. It does not pick the geometry.
- **Cyclist:** the predicted path intersects the corridor with TTC \< 3 s → `YIELD`: the speed cap drops and a stop point is placed behind the conflict.

Quadrotor · Mission D

- **TRANSIT:** output a goal = S₁, cruise 8 m/s, altitude band 35–45 m, and the geofence.
- **INSPECT:** standoff constraint ‖p − structure‖ ≥ 8 m, yaw locked to face the structure, 12 s hover dwell.
- **Target detected:** the detector's confidence stays above threshold for N frames (hysteresis) → `DIVERT`, with the goal set to the target's estimated position. Resume the route afterwards.
- **Battery reserve reached:** → `RTL`. This outranks everything except `FAILSAFE`. The reserve is computed from the energy needed to fly home into the 7 m/s wind, not a fixed percentage.

```
# Behaviour FSM skeleton (Toronto assignment style)
class BehaviouralPlanner:
    def step(self, ego, route, objs, rules):
        if self.state == "TRACK_SPEED":
            if rules.stop_line_within(ego, self.lookahead(ego.v)):
                self.state, self.s_stop = "DECEL_TO_STOP", rules.stop_s - 1.0
            elif (lead := objs.lead_in_lane(ego)) and lead.gap < self.follow_dist(ego.v):
                self.state, self.lead = "FOLLOW_LEADER", lead
        elif self.state == "DECEL_TO_STOP" and ego.v < 0.1:
            self.state, self.t_stopped = "STOPPED", 0.0
        elif self.state == "STOPPED":
            self.t_stopped += self.dt
            if self.t_stopped > 3.0 and objs.intersection_clear(ttc_min=4.0):
                self.state = "TRACK_SPEED"
        return Maneuver(self.state, goal=self.goal_state(ego, route),
                        v_limit=rules.v_limit, s_stop=self.s_stop, lead=self.lead)

    def lookahead(self, v, a_comf=2.0):   # far enough to stop comfortably
        return v*v / (2*a_comf) + 10.0
```

Common failure: chattering

If a condition flips back and forth around its threshold, the FSM flips state at 10 Hz and the car twitches. Add **hysteresis** (enter at one threshold, leave at another), a **minimum dwell time** in each state, and **commitment**: once a lane change starts, finish it or abort it explicitly, never half-way. The same rules apply to autopilot mode logic.

- **Scaling up:** flat FSMs grow too many states to manage. Production stacks use **hierarchical FSMs** (a scenario layer such as "intersection" or "highway", with sub-states inside) or **behaviour trees**. This is the same idea as Stateflow's hierarchical states and parallel (AND) decomposition.
- **Cost-based arbitration:** generate several candidate maneuvers, have L3 plan each, and choose the cheapest safe one. This blurs the L2/L3 line (Apollo, Baidu's open-source driving stack, does this).
- **Uncertainty:** occlusions (the cyclist hidden behind the truck) call for *phantom agents* or POMDPs, i.e. planning for "someone might be there".
- **Time-to-collision:** TTC = gap / closing speed. Use it as a gate on transitions, together with a margin and the prediction's uncertainty.

[▶Decision Making and Planning: Motion PlanningSelf-Driving Cars · Lecture 12.4 (Geiger, Tübingen)](https://www.youtube.com/watch?v=PSX18U1fYEY) [FIGBehaviour planning by FSM, with diagramsGitHub · illustrated walkthrough](https://github.com/A2Amir/Behavior-Planning-by-Finite-State-Machine)

L3

## Local planning: the geometry

[#](#s-l3)

Which exact curve for the next few seconds?5–20 Hzhorizon 2–8 sCourse 4 · Modules 6–7

The local planner takes the maneuver and its constraints and produces a **collision-free, kinematically feasible curve**. The vehicle model now matters (a car cannot turn tighter than its curvature limit), and so do obstacles and their predicted motion. The Toronto course splits this into two ideas: *reactive planning in static environments* (Module 6) and *smooth local planning* (Module 7). In practice there are three families of methods:

| Family | How it works | Strength | Weakness |
| --- | --- | --- | --- |
| **Reactive** | Sample control inputs (v, ω), roll each out for a short time, score the results. Examples: trajectory rollout, DWA, potential fields, VFH. | Very fast, simple, and handles surprises. | Myopic: gets stuck in local minima (U-shaped obstacles) and ignores the global goal. |
| **Search / sampling** | Discretize the state space and search it: conformal lattice, state lattice with motion primitives, hybrid A\*, RRT / RRT\*. | Complete within its resolution; handles complex free space (parking lots, mazes). | Resolution vs. compute trade-off; raw paths are jerky. |
| **Optimization** | Parameterize a curve (spiral, polynomial, spline) and solve a boundary-value or optimization problem with constraints. | Smooth, dynamically consistent, and optimal locally. | Needs a good initial guess and can converge to a local minimum. |

Real planners **combine** these families. The Toronto course's final project does exactly this. It builds a **conformal lattice**: it samples goal states laterally offset from the lane centre, connects each one to the ego pose with a **cubic spiral** (the optimization part), checks every candidate for collisions using circles along the footprint, and picks the best by cost (distance from centreline plus proximity to obstacles).

#### Inputs

- maneuver + goal set from L2
- corridor / lane boundaries, reference line
- costmap / occupancy grid *✈ ESDF*
- predicted object trajectories
- ego state x̂ *(start of the plan)*
- vehicle limits: κ_max, footprint

#### Outputs

- path σ(s) = {x, y, ψ, κ} sampled along arc length s
- or a full trajectory, if L3 and L4 are coupled
- ✈ polynomial coefficients per segment, p(t)
- status: OK / NO_FEASIBLE_PATH *→ L2*

**FIG 4**Mission C at the truck. L2 said "NUDGE, d ∈ \[0, 3.2\] m". L3 generates the candidates, rejects the two that collide, and picks ★. L3 chooses the geometry; L2 had only set the range.

Path vs trajectory

A **path** is geometry only, parameterized by arc length: σ(s). A **trajectory** adds time: x(t). The Toronto course uses *path–velocity decomposition*: L3 produces σ(s), then L4 assigns v(s). This keeps each problem small. It breaks down when obstacles move fast, because whether a path is safe then depends on *when* you are on it. At that point you plan in (s, t) or plan a full trajectory directly (lattice in space-time, MPC).

Car · the Frenet frame

Roads provide a natural reference line, so plan in **Frenet coordinates**: s is the distance along the lane and d is the lateral offset. "Stay in lane" becomes d ≈ 0, a lane change becomes d: 0 → 3.5 m, and curved roads become straight in (s, d). The curve family is the **cubic spiral** κ(s) = a + bs + cs² + ds³. Curvature is continuous, and κ_max can be enforced directly, which makes the path steerable.

Quadrotor · no reference line

There are no lanes, so Frenet is usually the wrong frame. Plan in 3-D Cartesian space. The standard approach: (1) a front end (A\*, kinodynamic A\*, or RRT\*) finds a rough collision-free path through the ESDF; (2) a back end fits **piecewise polynomials** that minimize **snap** (4th derivative of position) and pass through safe corridors. Why snap? Because a quadrotor is **differentially flat**: p(t) and yaw determine thrust and attitude, and snap maps to the *derivative* of the body rates. Smooth snap means smooth motor commands.

**FIG 5**Differential flatness in one picture. The acceleration the path demands sets the thrust direction. A path that bends sharply demands a large tilt, and a tilt limit is effectively an acceleration limit.

```
# Conformal lattice step (course final project, simplified)
def local_plan(ego, maneuver, obstacles, ref_line):
    goals = [ref_line.offset(maneuver.goal_s, d) for d in np.linspace(*maneuver.d_range, 7)]
    best, best_cost = None, np.inf
    for g in goals:
        path = cubic_spiral(ego.pose, g)                # solve 2-point BVP: κ(s)=a+bs+cs²+ds³
        if path is None or np.max(np.abs(path.kappa)) > KAPPA_MAX: continue
        if collides(path, obstacles, circle_offsets=[-1.0, 1.0, 3.0], r=1.5): continue
        cost = W_CENTER*abs(g.d) + W_OBS*proximity(path, obstacles) + W_SMOOTH*np.trapz(path.kappa**2, path.s)
        if cost < best_cost: best, best_cost = path, cost
    return best   # None → tell L2 "no feasible path" → YIELD / stop
```

- **Hybrid A\*** searches a grid while storing continuous (x, y, ψ) poses in each cell, expanding nodes with motion primitives. It is the standard answer for parking and unstructured spaces, where there is no lane to follow.
- **RRT\*** is asymptotically optimal: it rewires the tree as it grows. Kinodynamic RRT\* samples in state space and connects nodes with dynamically feasible steering. It is useful in high-dimensional or cluttered 3-D space (drones).
- **Trajectory optimization:** CHOMP, TrajOpt, iLQR, and ESDF-gradient B-spline optimization (Fast-Planner, EGO-Planner) refine an initial path by minimizing smoothness + collision + dynamics costs.
- **Consistency:** add a cost for deviating from the previous cycle's plan. Without it, two nearly equal candidates swap every cycle and the vehicle weaves.
- **Local minima:** reactive methods (potential fields, DWA) can trap the vehicle in U-shaped obstacles. The fix is architectural: L1/L3 search supplies subgoals, so the reactive layer only needs to handle short-range surprises.

[▶Frenet Frames · Motion Planning for RobotsYouTube · self-driving local planning](https://www.youtube.com/watch?v=DhP3jiC9YX0) [FIGHighway trajectory planning using a Frenet reference pathMathWorks example with figures](https://www.mathworks.com/help/nav/ug/highway-trajectory-planning-using-frenet.html) [▶Introduction to Motion Planning Algorithms (RRT)MATLAB · Motion Planning with RRT, Part 1](https://www.youtube.com/watch?v=-fePRPyeKnc) [FIG✈ Minimum-snap trajectory generation, derivedTech notes · QP formulation](https://dev10110.github.io/tech-notes/control-theory/min_snap.html) [▶Artificial Potential Field method (reactive)YouTube · robot motion planning](https://www.youtube.com/watch?v=Ls8EBoG_SEQ) [FIG✈ Fast-Planner: quadrotor local plannerHKUST · code, demo GIFs, papers](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

L4

## Velocity profile: adding time

[#](#s-l4)

How fast at every point on the path?same cycle as L3Course 4 · Module 7 (velocity profile generation)

The path σ(s) says *where*. The velocity profile v(s) says *when*. Together they form the trajectory the controller tracks. The speed at each point is capped by several limits at once, and the profile has to respect the tightest one everywhere.

#### Inputs

- path σ(s) with curvature κ(s)
- v_limit, s_stop, lead vehicle (s_lead, v_lead) *from L2*
- limits: a_lat,max, a_accel, a_brake, jerk
- current v₀, a₀ *(continuity with last plan)*
- ✈ tilt/thrust limits, wind, time allocation

#### Outputs = the reference trajectory

- v(s), a(s) along the path
- resampled in time: {t_k, x, y, ψ, κ, v, a}
- ✈ {t_k, p, v, a, jerk, yaw, yaw_rate}
- stamped with plan time *(for the controller to index)*

**FIG 6**Mission C approaching the left turn and then the stop line. The planner has to slow down *before* the turn, which means looking ahead. The backward pass provides that lookahead. Without it the car reaches the corner too fast and brakes inside it.

```
# Forward–backward pass: the workhorse velocity profiler
def velocity_profile(s, kappa, v0, v_lim, a_acc=1.5, a_brk=2.5, a_lat=2.0, v_end=0.0):
    v = np.minimum(v_lim, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-6)))  # curvature ceiling
    v[0] = v0
    for i in range(1, len(s)):                             # forward: acceleration limit
        v[i] = min(v[i], np.sqrt(v[i-1]**2 + 2*a_acc*(s[i]-s[i-1])))
    v[-1] = min(v[-1], v_end)
    for i in range(len(s)-2, -1, -1):                  # backward: braking limit
        v[i] = min(v[i], np.sqrt(v[i+1]**2 + 2*a_brk*(s[i+1]-s[i])))
    t = np.concatenate([[0], np.cumsum(2*np.diff(s) / np.maximum(v[:-1]+v[1:], 1e-3))])
    return v, t                                              # → sample x(t), y(t), ψ(t) for control
```

Car · Mission C profiles

- **Follow leader:** target the lead vehicle's speed while keeping a gap d = d₀ + v·T_gap (T_gap ≈ 1.8 s). The Intelligent Driver Model (IDM) is a smooth closed-form version of this.
- **Stop at line:** the backward pass from v = 0 at s_stop. The course uses a linear ramp: decelerate at a fixed rate and stop exactly at the line.
- **School zone:** v_limit steps from 50 to 30 km/h; the backward pass brakes *before* the sign.
- **Comfort:** jerk ≲ 1–2 m/s³, so trapezoidal acceleration becomes an S-curve.

Quadrotor · time allocation

- In min-snap, time is a *decision variable*: how many seconds each segment gets. Too little time gives accelerations beyond what the tilt allows. Too much wastes battery. Scale the segment durations until max‖a‖ ≤ g·tan(θ_max) and max thrust stays under T_max.
- **Wind:** the planner works in ground frame, but limits apply to *airspeed*. A 7 m/s crosswind means an 8 m/s ground-speed leg can need about 10.6 m/s airspeed.
- **Gap:** slow down through the 3 m gap so that tracking error × speed stays inside the clearance margin.

**FIG 7**The same concept in two domains: a feasible-acceleration envelope. L4 keeps every point of the profile inside it. For a quadrotor the envelope is a cone around −g, with half-angle θ_max and height set by T_max/m.

- **Time-optimal path parameterization (TOPP):** solved exactly as a convex problem in b(s) = ṡ². The forward–backward pass above is its simplest special case.
- **Speed optimization as a QP** in the (s, t) plane, used by Apollo: predicted obstacles become forbidden regions in s–t, and the speed curve must pass around them. This is how you handle a cyclist crossing ahead of the car: decide to go before it or yield after it.
- **✈ Fixed-wing**: the profile stays above stall speed with a margin (≥ 1.3 V_s), and the load factor in turns n = 1/cos φ raises stall speed by √n. Energy management (Total Energy Control System, TECS) trades kinetic and potential energy.

[▶Trapezoidal vs S-Curve motion profilesYouTube · jerk-limited profiles](https://www.youtube.com/watch?v=C0XjXqO6Ji8) [FIGReal-time motion planner with trajectory optimizationCMU · ICRA 2012 (path + speed figures)](https://publications.ri.cmu.edu/storage/publications/pub_files/2012/5/ICRA12_xuwd_Final.pdf)

L5

## Handoff to control

[#](#s-l5)

What does the controller actually receive?20–100 HzCourse 1 · Modules 5–6 (vehicle control)

This interface is where most integration bugs live. Guidance publishes a **time-stamped reference trajectory**. The controller interpolates it at *its own* rate, computes errors against x̂, and commands actuators. Specify the interface as precisely as a hardware bus: units, frames, timestamps, rates, and what happens when a message is late.

#### Guidance → control

- trajectory {t_k, x, y, ψ, κ, v, a} *world/ENU frame*
- plan timestamp + validity horizon
- feed-forward terms: κ (steer), a (throttle) *✈ a → attitude*
- mode flags: normal / stop / emergency

#### Control → guidance (upward)

- tracking error e_lat, e_ψ, e_v
- saturation flags *(steer, torque, thrust, tilt)*
- "cannot track" / fault *→ triggers replan*
- actual state reached *(initial condition for next plan)*

Car · two decoupled loops (Course 1)

- **Longitudinal:** PI(D) on v_ref − v, plus feed-forward from a_ref and a drag/grade model → throttle/brake.
- **Lateral:** *pure pursuit* steers toward a point ℓ_d ahead: δ = atan(2L·sin α / ℓ_d). The *Stanley* controller: δ = ψ_e + atan(k·e / v). Or use **MPC** on a kinematic or dynamic bicycle model to do both loops at once.
- Pure pursuit is literally a guidance law, the same as missile pure pursuit.

Quadrotor · cascaded autopilot

- PX4 offboard: publish `TrajectorySetpoint` (position, velocity, acceleration, yaw) at ≥ 2 Hz; faster, 20–50 Hz, is typical. Use NaN for fields you leave to the autopilot.
- Cascade: position P → velocity PID → **a_cmd + a_ff** → thrust vector → attitude → rate PID → mixer → motors.
- Send the **acceleration feed-forward** from min-snap. Without it the position loop lags on every curve and the drone cuts corners.

The aerospace view of the same boundary

For a fixed-wing UAV, the guidance law is typically **L1 guidance** (Park, Deyst & How): pick a reference point a distance L₁ ahead on the path and command a_cmd = 2V²/L₁ · sin η, which becomes a bank angle φ = atan(a_cmd/g). That is pure pursuit with a lateral-acceleration output. Missiles use **proportional navigation**, a_cmd = N·V_c·λ̇. In both cases the "guidance" output is an acceleration command handed to an attitude autopilot. Hold the **bandwidth rule** fixed: each loop should be 3–10× faster than the loop that commands it.

```
# Pure pursuit (Course 1 lateral control) — a guidance law in disguise
def pure_pursuit(ego, traj, L=2.9, k=0.8, ld_min=3.0):
    ld = max(ld_min, k * ego.v)                          # lookahead grows with speed
    tx, ty = traj.point_at_distance_ahead(ego, ld)
    alpha = math.atan2(ty - ego.y, tx - ego.x) - ego.yaw   # bearing to target in body frame
    return math.atan2(2 * L * math.sin(alpha), ld)          # steering δ
```

- **Timing:** the controller must index the trajectory by *current time − plan timestamp*, not by array index. Otherwise planner latency becomes a steady tracking lag.
- **Replan start state:** start the new plan from the *previous plan's* state at t_now + latency, not from the raw x̂. Starting from the noisy x̂ injects estimator noise into every new plan (a well-known source of jitter). Re-anchor to x̂ only when the tracking error exceeds a threshold.
- **MPC blurs L3–L5:** a nonlinear MPC with obstacle constraints plans and tracks at once. It is still worth keeping a separate L3 for global structure, because MPC horizons are short and non-convex obstacles create local minima.

[▶Vehicle Path Tracking Using Pure PursuitMATLAB · (Stanley version: youtu.be/FHQFya0-JBs)](https://www.youtube.com/watch?v=zMdoLO4kRKg) [▶Why Use Model Predictive Control?MATLAB · Understanding MPC, Part 1](https://www.youtube.com/watch?v=8U0xiOkDcmw) [FIG✈ PX4 Offboard mode: setpoint types and ratesPX4 Guide](https://docs.px4.io/main/en/flight_modes/offboard) [FIGPure pursuit, Stanley and MPC, with geometry figuresMedium · illustrated comparison](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081)

×

## Cross-cutting: avoidance, optimization, reactive vs deliberative

[#](#s-cross)

The request listed *obstacle avoidance*, *optimization*, *reactive* and *static* alongside the layers. None of them is a layer of its own. Each is a concern or a technique that shows up **inside several layers, at different horizons**. This is the main thing to fix in your mental model.

| Concept | L1 Global | L2 Behaviour | L3 Local | L4 Velocity | L5 Control |
| --- | --- | --- | --- | --- | --- |
| **Obstacle avoidance** | closed roads, no-fly zones (static map) | decide: overtake, yield, or stop | geometric clearance, collision check | slow down or stop before conflict | emergency brake / hover (AEB) |
| **Optimization** | shortest path on a graph | min-cost maneuver choice | spiral BVP, min-snap QP, CHOMP | time-optimal / QP speed | MPC, LQR |
| **Static world** | road graph, terrain | rules, stop lines | occupancy grid, ESDF | curvature, speed limits | — |
| **Dynamic world** | traffic-weighted costs | TTC, gaps, predictions | space-time collision check | s–t obstacles, following | disturbance rejection |
| **Reactive (no lookahead)** | — | immediate failsafe triggers | DWA, rollout, potential field, VFH | — | feedback itself |

Deliberative vs reactive

**Deliberative** planners (A\*, lattice, RRT\*) think ahead but are slow. **Reactive** ones (DWA, potential fields) are fast but myopic. A stack that works uses both: deliberative layers choose *where* to go, and a reactive layer keeps the vehicle safe during the \~100 ms between plans. In Mission C, if the cyclist appears suddenly, the automatic emergency brake (AEB) handles the first 200 ms, before the next L2 cycle has even run.

[▶Path Planning and Navigation for Autonomous RobotsMATLAB · global + local planning together](https://www.youtube.com/watch?v=Yse_YDpmsBM) [FIGPotential functions: figures and local-minima examplesCMU · Choset lecture slides](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

⟳

## The replanning loop in time

[#](#s-loop)

The layers do not run once, top to bottom. They run **concurrently at different rates**, and each consumes the latest output of the layer above. Here is one second of Mission C at the truck:

| t (ms) | Event | Who acts | Message |
| --- | --- | --- | --- |
| 0 | Perception flags truck stationary in lane | L0 | obj#17 {v=0, class=truck} |
| 40 | Behaviour cycle: lane blocked, left gap OK | L2 | NUDGE d∈\[0,3.2\] v_lim=40 |
| 90 | Local planner: 7 candidates, 5 valid, pick d=1.7 | L3 | σ(s), 60 m |
| 95 | Velocity profile under v_lim and κ | L4 | x_ref(t), 6 s |
| 100–1000 | Controller tracks at 50 Hz (45 cycles) | L5 | δ, throttle |
| 190, 290, … | L3/L4 re-run on new data, blended with last plan | L3–L4 | x_ref(t) updated |
| 610 | Cyclist emerges, TTC 2.1 s | L2 → YIELD | s_stop behind conflict |
| 640 | Backward pass → braking profile | L4 | a = −3.0 m/s² |

- **Always keep a fallback:** each cycle, also compute a *safe-stop* trajectory (✈ a safe hover or safe-land trajectory). If the next plan is late or invalid, the controller executes the fallback. Real vehicles are built this way; it guards against planner crashes and timing overruns.
- **Watchdogs:** the controller rejects a reference older than its validity horizon, and the autopilot leaves offboard mode if setpoints stop arriving (PX4 does this after roughly 0.5 s).
- **Deterministic scheduling** matters more than average speed. A planner that averages 20 ms but sometimes takes 400 ms is unsafe.

[▶✈ Mapping & planning in dynamic environments with PX4 offboardPX4 Dev Summit · James Strawson](https://www.youtube.com/watch?v=tV8jm8UKyPE) [▶✈ UAV 3-D motion planning and obstacle avoidance in MATLABYouTube · drone planning demo](https://www.youtube.com/watch?v=74ILEYiz24g)

?

## Which layer broke?

[#](#s-debug)

A diagnostic table for your Python and CARLA runs (CARLA is the simulator the Toronto course uses). Start from the symptom and check the suspect layer first.

| Symptom | Suspect | Why / what to check |
| --- | --- | --- |
| Takes an absurd route | L1 | Edge costs or an inadmissible heuristic; one-way edges missing. |
| Stops and never goes; or runs the stop sign | L2 | A transition guard is never true, or a missing state. Log the state every cycle. |
| Vehicle twitches between two choices | L2 / L3 | No hysteresis or dwell time; no consistency cost on the lattice. |
| Clips obstacle corners | L3 | Collision circles too small, or the check only at samples, not along the swath. |
| Enters turns too fast, brakes mid-corner | L4 | No backward pass; curvature not fed into the speed ceiling. |
| Oscillates around a straight path | L5 | Gains too high; lookahead too short at speed. |
| Cuts corners, lags on curves | L4 / L5 | Missing feed-forward (κ or a_ff); reference indexed by array index, not time. |
| Jitter each time a new plan arrives | L3 ↔ L5 | Replanning from the raw x̂ instead of from the previous plan's state. |
| ✈ Drone overshoots waypoints, saturates tilt | L4 | Time allocation too aggressive for tilt/thrust limits; wind not in the limits. |

PY

## Build it in Python, layer by layer

[#](#s-python)

A build order that matches the course and keeps every stage testable on its own. Keep one `dataclass` per interface message (Route, Maneuver, Path, Trajectory) so each layer can be unit-tested with synthetic input.

```
# interfaces.py — the contracts between layers
@dataclass class Route:      waypoints: np.ndarray; v_limits: np.ndarray             # L1 → L2
@dataclass class Maneuver:   name: str; goal_s: float; d_range: tuple; v_limit: float
                                 s_stop: float|None; lead: Obj|None                       # L2 → L3/L4
@dataclass class Path:       s: np.ndarray; x: np.ndarray; y: np.ndarray; yaw: np.ndarray; kappa: np.ndarray  # L3 → L4
@dataclass class Trajectory: t: np.ndarray; x, y, yaw, v, a: np.ndarray; stamp: float  # L4 → L5

# main loop — each layer at its own rate
while sim.running:
    x_hat, objs = nav.update()                                     # 100 Hz
    if tick % 10 == 0:                                            # 10 Hz planning cycle
        man  = behaviour.step(x_hat, route, objs, rules)
        path = local.plan(prev_traj.state_at(now+latency), man, occ_grid, objs)
        traj = (velocity_profile(path, man, x_hat) if path else safe_stop(x_hat))
        prev_traj = traj
    u = controller.track(x_hat, prev_traj, now)                    # 100 Hz
    sim.apply(u)
```

| Step | Build | Test by |
| --- | --- | --- |
| 1 | Kinematic bicycle model + pure pursuit on a fixed spline | Tracking error on a figure-8 |
| 2 | Velocity profile (fwd/bwd pass) on that spline | Plot v(s) vs √(a_lat/κ) ceiling |
| 3 | Occupancy grid from simulated LiDAR | Visual check vs ground truth |
| 4 | A\* on a small road graph | Compare with Dijkstra: same cost, fewer expansions |
| 5 | Behaviour FSM (stop sign + lead car) | Scripted scenarios, assert state sequence |
| 6 | Cubic spirals + conformal lattice + circle collision check | Static obstacle in lane; plot candidates |
| 7 | Close the loop in CARLA (course final project) | All scenarios, no collisions, smooth v |
| 8 ✈ | 3-D point-mass quad, A\* in voxels, min-snap QP (cvxpy) | Max tilt ≤ limit; thread the gap |
| 9 ✈ | Same stack in PX4 SITL via MAVSDK / ROS 2 offboard | Compare setpoint vs flown trajectory |

▶

## Study path by level

[#](#s-study)

#### Beginner

1. The GNC split, and why a path is not a trajectory.[▶ What Is Autonomous Navigation? (MATLAB)](https://www.youtube.com/watch?v=Fw8JQ5Q-ZwU)
2. What each layer takes in and hands down: global vs local planning.[▶ Path Planning and Navigation for Autonomous Robots](https://www.youtube.com/watch?v=Yse_YDpmsBM)
3. Implement A\* on a small road graph.[▶ Path Planning with A\* and RRT (MATLAB)](https://www.youtube.com/watch?v=QR3U1dgc5RE)
4. Implement pure pursuit on a fixed spline.[▶ Vehicle Path Tracking Using Pure Pursuit](https://www.youtube.com/watch?v=zMdoLO4kRKg)
5. Build the stop-sign FSM as a Stateflow chart.[▶ Modeling State Machines with Stateflow](https://www.youtube.com/watch?v=rUeMUCrlxP0)

#### Intermediate

1. Frenet frame, conformal lattice, cubic spirals as a BVP.[▶ Frenet Frames · Motion Planning for Robots](https://www.youtube.com/watch?v=DhP3jiC9YX0)
2. Velocity profiles: trapezoid, S-curve, forward–backward pass.[▶ Trapezoidal vs S-Curve Profiles](https://www.youtube.com/watch?v=C0XjXqO6Ji8)
3. TTC, gap acceptance and hysteresis in behaviour decisions.[▶ Decision Making and Planning (Geiger, L12.4)](https://www.youtube.com/watch?v=PSX18U1fYEY)
4. Stanley controller, and why lookahead scales with speed.[▶ Vehicle Path Tracking Using Stanley](https://www.youtube.com/watch?v=FHQFya0-JBs)
5. MPC for path tracking with constraints.[▶ Why Use MPC? (Understanding MPC, Part 1)](https://www.youtube.com/watch?v=8U0xiOkDcmw)

#### Advanced

1. Hybrid A\* for parking and unstructured spaces.[▶ Smooth optimal paths using Hybrid A\*](https://www.youtube.com/watch?v=QAzyG2DB8Io)
2. Kinodynamic sampling planners (RRT\*, beyond).[▶ MIT 6.8210 L18: Sampling-based kinodynamic planning](https://www.youtube.com/watch?v=ChiQgvVvgKM)
3. Differential flatness and min-snap with time allocation.[▶ Quadrotor tracking & minimum-snap trajectories](https://www.youtube.com/watch?v=RGJT3rWFD9E)
4. NMPC vs flatness-based control for agile flight.[▶ NMPC vs Differential-Flatness Control (TRO 2022)](https://www.youtube.com/watch?v=SEZJ-OIR8Bo)
5. Gradient-based quadrotor replanning (Fast/EGO-Planner).[▶ EGO-Planner: ESDF-free local planner](https://www.youtube.com/watch?v=UKoaGW7t7Dk)
6. Planning under uncertainty: POMDP behaviour.[▶ POMDP explained](https://www.youtube.com/watch?v=-q61H11Lm0s)
7. End-to-end onboard planning with PX4 offboard.[▶ Mapping & planning with PX4 offboard](https://www.youtube.com/watch?v=tV8jm8UKyPE)

### Primary course and references

[EDUMotion Planning for Self-Driving CarsCoursera · University of Toronto (Course 4)](https://www.coursera.org/learn/motion-planning-self-driving-cars) [EDUSampling-based motion planningMIT · Underactuated Robotics (Tedrake)](https://underactuated.mit.edu/planning.html) [PDFIncremental sampling-based algorithms for optimal motion planning (RRT\*)Karaman & Frazzoli · RSS 2010](https://www.roboticsproceedings.org/rss06/p34.pdf) [DOC✈ UAV motion planning in MATLAB / SimulinkMathWorks · UAV Toolbox](https://www.mathworks.com/help/uav/motion-planning.html)

Rates, horizons and limits on this page are typical order-of-magnitude values; real stacks tune them to the vehicle. Figures are drawn for this page; the FIG links open external sources with canonical diagrams.

SL

## Building the Coursera projects in Simulink

[#](#s-simulink)

A direct mapping from every module of the [Self-Driving Cars Specialization](https://www.coursera.org/specializations/self-driving-cars) to the Simulink/MATLAB block, function, toolbox and app that implements it, for the car, a quadcopter and a fixed-wing. Bracketed numbers \[n\] point to the short list of MathWorks reference examples at the end of this section, each a ready-made model you can open and copy the architecture from.

### Model skeleton

One top-level model, one **referenced model per layer**, each at its own rate. Every interface is a `Simulink.Bus` in a data dictionary (Route, Maneuver, Path, Trajectory, VehicleState), with **Rate Transition** blocks between layers.

| Subsystem | Rate | Implemented as |
| --- | --- | --- |
| Scenario + plant | 1 ms fixed-step | Blocks |
| State estimation | 100–200 Hz | MATLAB Function (ES-EKF) |
| Control (L5) | 50–100 Hz | Blocks |
| Local planner + velocity profile (L3/L4) | 10 Hz | MATLAB Function + blocks |
| Behaviour (L2) | 10 Hz | Stateflow chart |
| Mission (L1) | event (function-call) | MATLAB Function |

Block or MATLAB Function?

Use a ready-made block when it implements the same algorithm as the course (Stanley, Velocity Profiler, sensor models, Kalman filters). Use a MATLAB Function for algorithms with no matching block or with course-specific details (ES-EKF with quaternion error state, conformal lattice, cubic-spiral BVP, A\* on your graph). Write MATLAB Functions for code generation from the start: fixed-size arrays, preallocated maximum node counts, plots only through `coder.extrinsic`.

### Python or C++ instead of Simulink?

Each module below also has a verdict on whether Python or C++ should replace the Simulink implementation, and how to connect it if so. First, the ways to bring outside code into Simulink, and what each one costs a model-based GNC workflow:

| Mechanism | Language | Code generation? | Use it for | Difficulty |
| --- | --- | --- | --- | --- |
| **MATLAB Function** with `py.` calls | Python | No: simulation only (`coder.extrinsic`) | Calling a Python reference implementation to compare against | Low |
| **MATLAB System block** / **Python Importer** wizard | Python | No: interpreted simulation only | Wrapping a stateful Python algorithm as a block for testing | Low |
| **C Caller** | C (C++ through custom-code settings) | Yes | Stateless library functions (a math routine, a solver call) | Low |
| **C Function** | C/C++ | Yes | Small stateful algorithms written in C++ | Low–medium |
| **S-Function Builder** / **Legacy Code Tool** | C/C++ | Yes | Wrapping an existing library with states and several ports; the tool writes the S-function for you | Medium |
| **Hand-written C/C++ MEX S-function** | C/C++ | Yes (with a TLC file for inlining) | Full control: multi-rate, continuous states, zero crossings. Rarely needed today | High |
| **FMU block** (FMI 2/3) | Any tool that exports an FMU | Depends on the FMU | Plant models from other tools or suppliers | Low |
| **ROS 2 blocks** (co-simulation / deployment) | Any ROS 2 node (C++ / Python) | Yes: Simulink side generates a ROS 2 node | Perception, SLAM and large planners that stay outside the model; **no S-function at all** | Medium |

What outside code costs you as an MBD / GNC engineer

- **Traceability:** requirements link to model elements, not into your C++ internals. The custom code needs its own requirements and review trail.
- **Verification:** Simulink Design Verifier and model coverage see custom code only partly. Simulink Coverage can measure C/C++ coverage, but property proving is limited.
- **Equivalence:** SIL and PIL back-to-back tests have to include the hand code, and floating-point or library differences show up there.
- **Determinism:** dynamic memory, threads, exceptions and non-fixed loop counts in C++ break the fixed-step, bounded-time assumptions GNC code relies on.
- **Interfaces:** units, frames (ENU vs NED), bus layouts and sample times must match by hand. That is where integration bugs live.
- **Certification:** under DO-178C (with its model-based supplement DO-331) or ISO 26262, model-generated code and hand code follow two verification processes. Mixing them doubles the evidence.

**Rule of thumb:** keep periodic, fixed-size, safety-relevant GNC (estimation, control, guidance laws, mode logic) in Simulink/Stateflow. Put large, dynamic-memory or ML components (perception, SLAM, big maps, sampling planners) in C++ nodes connected over **ROS 2**, not inside S-functions. Keep Python for training, data, analysis and test scripting, outside the deployed loop.

[DOCComparison of custom block optionsMathWorks · C Caller, C Function, S-Function Builder…](https://www.mathworks.com/help/simulink/ug/comparison-of-custom-block-functionality.html) [DOCIntegrating Python code with SimulinkMathWorks · simulation-only paths](https://www.mathworks.com/help/simulink/ug/overview-of-integrating-python-code-with-simulink.html)

### Coursera module → Simulink mapping (car · quadcopter · fixed-wing)

Each module shows how to build it for all three vehicles. The tag says directly whether a ready-made block exists or you need a MATLAB Function.

BLOCK ready-made Simulink block, drop in and configure MATLAB FUNCTION write a MATLAB Function block (may call a toolbox function inside) BLOCK + FUNCTION blocks with a small MATLAB Function for glue or model equations STATEFLOW Stateflow chart OFFLINE / APP run in MATLAB or an app, outside the simulation loop

#### Course 1 · Introduction to Self-Driving Cars

M4 · Vehicle dynamic modeling\[6\]

CARBLOCK

*Bicycle Kinematic Model*; *Vehicle Body 3DOF* (single-track) + tyre blocks.

Robotics System · Vehicle Dynamics Blockset

QUADCOPTERBLOCK + FUNCTION

*6DOF (Quaternion)* block. A MATLAB Function computes rotor thrust T = k_f·ω² and torques, plus the mixer. For guidance design, use the reduced-order *Guidance Model* block (multirotor). Start from the Quadcopter Project.

Aerospace Blockset · UAV Toolbox

FIXED-WINGBLOCK + FUNCTION

*6DOF* + *Aerodynamic Forces and Moments* (or *Digital DATCOM Forces and Moments*). A MATLAB Function computes C_L, C_D, C_m… from α, β, rates and surfaces. Add *COESA Atmosphere* and *WGS84 Gravity*, plus the *Guidance Model* block (fixed-wing) as a surrogate.

Aerospace Blockset · UAV Toolbox · apps: Steady State Manager (trim), Model Linearizer

Python / C++ instead?KEEP IN SIMULINK

Plants belong in Simulink: continuous states, solvers, and the trim/linearize apps all need them. If a supplier gives you a C/C++ model, bring it in as an **FMU** (FMU block) instead of writing an S-function.

M5 · Longitudinal speed control

CARBLOCK

*PID Controller* + *1-D Lookup Table* (throttle/brake map) + feed-forward from a_ref.

Simulink · app: PID Tuner

QUADCOPTERBLOCK + FUNCTION

Altitude and vertical speed through *PID Controller* blocks → collective thrust. A MATLAB Function adds hover feed-forward m·g/(cos φ·cos θ).

Simulink · app: PID Tuner

FIXED-WINGBLOCK + FUNCTION

TECS: a MATLAB Function computes total-energy and energy-balance errors, then two *PID Controller* blocks produce throttle and pitch. An inner pitch loop drives the elevator. Gains are scheduled on airspeed with a *1-D Lookup Table*.

Simulink · apps: PID Tuner, Control System Tuner

Python / C++ instead?KEEP IN SIMULINK

PID and TECS are the textbook MBD case: tuning apps, coverage and code generation come free. Rewriting them in C++ only adds verification work. With PX4, these loops already exist in PX4's C++ firmware, so either use them (you send setpoints) or replace them with Simulink-generated code.

M6 · Lateral control (pure pursuit, Stanley, MPC)\[1\]

CARBLOCK

*Lateral Controller Stanley* or *Pure Pursuit*. MPC option: *Path Following Control System* / *Nonlinear MPC Controller*.

Automated Driving · Robotics System · MPC · app: MPC Designer

QUADCOPTERBLOCK + FUNCTION

*Waypoint Follower* (multirotor) → position/velocity *PID Controller* blocks. A MATLAB Function turns acceleration into a thrust vector and attitude setpoint; rate *PID Controller* blocks follow. MPC option: *Nonlinear MPC Controller*.

UAV Toolbox · Simulink

FIXED-WINGBLOCK

*Waypoint Follower* / *Orbit Follower* (fixed-wing, L1-style lookahead) → bank command → roll and yaw-damper *PID Controller* blocks. To write L1 yourself: a MATLAB Function with a = 2V²/L₁·sin η, φ = atan(a/g).

UAV Toolbox · Simulink

Python / C++ instead?KEEP IN SIMULINK

Keep in Simulink. The one exception is an external MPC solver: acados and similar tools generate C code, and acados can export a Simulink S-function, so you drop it in without writing one yourself.

Final project · Track a racetrack\[1\] \[4\]

CARBLOCK

*Scenario Reader* + the controllers above. Unreal via *Simulation 3D Scene Configuration*, or CARLA through ROS 2 blocks.

Automated Driving · ROS Toolbox · apps: Driving Scenario Designer, Bird's-Eye Scope

QUADCOPTERBLOCK

*Path Manager* + *Waypoint Follower* around a gate course; *Simulation 3D UAV Vehicle* in Unreal, or PX4 SITL.

UAV Toolbox · PX4 support package

FIXED-WINGBLOCK

Same, with the fixed-wing 6DOF and *Orbit Follower* for the turns. Visualize with *FlightGear Preconfigured 6DoF Animation*.

UAV Toolbox · Aerospace Blockset

Python / C++ instead?KEEP IN SIMULINK

Keep the controllers in Simulink. The simulator (CARLA, or Gazebo for drones) is C++/Python; connect it through ROS 2 publish/subscribe blocks. No S-function needed.

#### Course 2 · State Estimation and Localization

M1 · Least squares / recursive least squares

CARBLOCK

*Recursive Least Squares Estimator* online (e.g. mass, tyre stiffness); `lsqlin` offline.

System Identification · Optimization

QUADCOPTEROFFLINE / APP

Identify k_f, k_m and motor time constant from thrust-stand data with `lsqlin`; the block can re-estimate in flight.

System Identification · app: System Identification

FIXED-WINGOFFLINE / APP

Identify aero derivatives (C_Lα, C_mα, C_mq…) from flight logs with `lsqlin` / `greyest`.

System Identification · app: System Identification

Python / C++ instead?PYTHON OFFLINE

Offline identification from logs works equally well in Python (numpy, scipy). The online RLS estimator stays in Simulink, where it can be code-generated.

M2 · KF, EKF, UKF

CARBLOCK + FUNCTION

*Kalman Filter*, *Extended Kalman Filter* or *Unscented Kalman Filter* block; you supply the state and measurement functions as MATLAB functions.

Control System

QUADCOPTERBLOCK

Attitude: *AHRS* block. Altitude: *Kalman Filter* block fusing baro + accelerometer.

Navigation / Sensor Fusion · Control System

FIXED-WINGBLOCK + FUNCTION

*Extended Kalman Filter* block with a MATLAB Function model that adds **wind states** (w_N, w_E) and an airspeed-scale factor, updated by the pitot measurement.

Control System

Python / C++ instead?KEEP IN SIMULINK

Core estimation should stay model-based: it is periodic, fixed-size and safety-relevant, which is exactly what code generation and coverage are good at.

M3 · GNSS / INS sensing

CARBLOCK

*IMU* and *GPS* blocks (noise, bias, latency).

Navigation / Sensor Fusion

QUADCOPTERBLOCK + FUNCTION

*IMU* (includes magnetometer) + *GPS*. Model the barometer in a MATLAB Function (noise + drift), and rotor vibration as band-limited noise on the accelerometer.

Navigation / Sensor Fusion

FIXED-WINGBLOCK + FUNCTION

*IMU*, *GPS*, *Three-axis Accelerometer* / *Gyroscope*. Model the pitot-static airspeed sensor and AoA vane in a MATLAB Function (lag + noise).

Navigation · Aerospace Blockset

Python / C++ instead?KEEP IN SIMULINK

Sensor models are simulation-only blocks; there is nothing to gain from Python or C++ here.

M4 · LiDAR sensing, scan matching

CARBLOCK + FUNCTION

*Simulation 3D Lidar*; a MATLAB Function runs `pcregisterndt` / `pcregistericp` for scan-to-map pose.

Automated Driving · Computer Vision · Lidar · app: Lidar Viewer

QUADCOPTERBLOCK + FUNCTION

*Simulation 3D Lidar* (UAV library) or `uavLidarPointCloudGenerator`. For GPS-denied flight use `lidarSLAM` / `monovslam`, mostly offline first.

UAV Toolbox · Navigation · Computer Vision · app: SLAM Map Builder

FIXED-WINGMATLAB FUNCTION

A LiDAR is rarely the primary sensor. Model a laser rangefinder in a MATLAB Function for height above terrain during approach and flare.

—

Python / C++ instead?C++ OK · WRAP OR ROS 2

Heavy point-cloud work (PCL, KISS-ICP-style odometry) is mature in C++. Run it as a **ROS 2 node** and feed its pose into the Simulink EKF, or wrap a small function with *S-Function Builder*.

Final project · ES-EKF (IMU + GNSS + LiDAR)

CARMATLAB FUNCTION

MATLAB Function running the error-state EKF (quaternion attitude). Ready-made equivalent inside the same block: `insEKF` / `insfilterErrorState`.

Sensor Fusion & Tracking · app: Tracking Scenario Designer (test data)

QUADCOPTERMATLAB FUNCTION

Same ES-EKF with 15 states (p, v, q, b_a, b_g) plus baro and magnetometer updates. Ready-made: `insEKF` with accelerometer, gyroscope, magnetometer and GPS sensor objects, or `insfilterMARG`.

Sensor Fusion & Tracking

FIXED-WINGMATLAB FUNCTION

Same ES-EKF plus airspeed and zero-sideslip pseudo-measurements and wind states. Ready-made: `insEKF` with a custom airspeed measurement model.

Sensor Fusion & Tracking

Python / C++ instead?KEEP IN SIMULINK

This is the GNC core: keep it traceable, tested and code-generated. Python is fine for a reference implementation to compare against, called through `py.` in simulation only.

#### Course 3 · Visual Perception

M1 · Camera model and calibration

CAROFFLINE / APP

`cameraIntrinsics`, `estimateCameraParameters`.

Computer Vision · app: Camera Calibrator

QUADCOPTEROFFLINE / APP

Same for the gimbal / downward camera, plus camera-to-IMU extrinsics for VIO.

Computer Vision · app: Camera Calibrator

FIXED-WINGOFFLINE / APP

Same, plus boresight alignment for a nose or belly camera used in landing.

Computer Vision · app: Camera Calibrator

Python / C++ instead?PYTHON OFFLINE

OpenCV in Python calibrates as well as the MATLAB app. Only the resulting parameters go into the model.

M2 · Visual features and visual odometry

CARMATLAB FUNCTION

`detectORBFeatures`, `matchFeatures`, `estrelpose` inside a MATLAB Function.

Computer Vision

QUADCOPTERMATLAB FUNCTION

Visual / visual-inertial odometry for GPS-denied indoor flight (`monovslam`, stereo VO); fuse the VO pose into the ES-EKF.

Computer Vision

FIXED-WINGOFFLINE / APP

Rarely in the loop. Optical flow or feature tracking only for vision-aided landing.

Computer Vision

Python / C++ instead?C++ / PYTHON PREFERRED

Real-time VO/VIO/SLAM is a C++ ecosystem (ORB-SLAM3, VINS-Fusion, OpenVINS). Run it as a ROS 2 node, publish the pose, and fuse it in the Simulink ES-EKF. Rewriting it in Simulink is not worth it. This is the right split for a GPS-denied drone.

M3–M4 · Neural networks and 2-D object detection

CARBLOCK

*Deep Learning Object Detector* / *Predict* block with `yolov4ObjectDetector`.

Computer Vision · Deep Learning · apps: Deep Network Designer, Image Labeler

QUADCOPTERBLOCK

Same blocks for target detection (flags, people). The detection is the event that sends L2 into DIVERT.

Computer Vision · Deep Learning

FIXED-WINGBLOCK

Same blocks for ground-target or runway detection at longer range and lower frame rate.

Computer Vision · Deep Learning

Python / C++ instead?C++ / PYTHON PREFERRED

Train in Python (PyTorch). Deploy either as a C++ runtime node (ONNX Runtime / TensorRT) on ROS 2, or import the ONNX file into MATLAB (`importNetworkFromONNX`) and generate code. Only the detections (class, box, confidence) cross into Simulink.

M5 · Semantic segmentation

CARMATLAB FUNCTION

`semanticseg` (DeepLab v3+) for drivable space.

Deep Learning · app: Ground Truth Labeler

QUADCOPTERMATLAB FUNCTION

Segmentation for safe landing zones and obstacles.

Deep Learning · app: Image Labeler

FIXED-WINGOFFLINE / APP

Runway / terrain segmentation for landing, usually offline-trained and rarely in the control loop.

Deep Learning

Python / C++ instead?C++ / PYTHON PREFERRED

Same as detection: Python training, C++ or GPU runtime, masks or derived free space passed into the model.

Final project · Stereo depth, drivable space, lanes, obstacle distance

CARMATLAB FUNCTION

`disparitySGM` + `reconstructScene`. *Simulation 3D Camera* provides ground-truth depth and labels.

Computer Vision · Automated Driving · app: Stereo Camera Calibrator

QUADCOPTERBLOCK + FUNCTION

*Simulation 3D Camera* depth output. A MATLAB Function converts depth to a point cloud and inserts it into `occupancyMap3D`.

UAV Toolbox · Navigation

FIXED-WINGBLOCK + FUNCTION

Camera only for landing alignment; distance to terrain comes from GNSS/INS + rangefinder.

UAV Toolbox

Python / C++ instead?C++ / PYTHON PREFERRED

Perception pipelines usually live in C++/Python nodes (OpenCV). Simulink receives the obstacle list or occupancy, not images.

#### Course 4 · Motion Planning for Self-Driving Cars

M1 · The planning problem (hierarchy)\[2\]

CARBLOCK

One referenced model per layer, bus interfaces, *Rate Transition* blocks.

Simulink · app: Simulation Data Inspector

QUADCOPTERBLOCK

Same skeleton. For hardware, PX4 *uORB Read* / *uORB Write* blocks replace the plant interface.

UAV Toolbox · PX4 support package

FIXED-WINGBLOCK

Same skeleton. L1 lateral guidance and TECS replace the trajectory-tracking controller.

UAV Toolbox

Python / C++ instead?KEEP IN SIMULINK

Keep the architecture, buses and rates in Simulink even if some layers are external. An external layer plugs in as a ROS 2 node or a C Function block behind the same bus.

M2 · Mapping for planning (occupancy grids)

CARMATLAB FUNCTION

`occupancyMap` / `binaryOccupancyMap` + `insertRay`; `vehicleCostmap` for collision checks.

Navigation · Automated Driving · apps: SLAM Map Builder, Lidar Viewer

QUADCOPTERMATLAB FUNCTION

`occupancyMap3D` + `insertPointCloud` + `inflate`. ESDF: `bwdist` on the voxel grid inside a MATLAB Function.

Navigation · Image Processing

FIXED-WINGMATLAB FUNCTION

2.5-D: terrain elevation grid + geofence polygons (`inpolygon`) + airspace volumes.

Mapping · Navigation

Python / C++ instead?C++ OK · WRAP OR ROS 2

A 2-D grid is fine in Simulink. Large 3-D maps (OctoMap, voxblox, nvblox) are C++ libraries with dynamic memory; run them as ROS 2 nodes, not S-functions.

M3 · Mission planning (Dijkstra, A\*)\[5\]

CARMATLAB FUNCTION

`navGraph` + `plannerAStar` in a MATLAB Function.

Navigation

QUADCOPTERBLOCK + FUNCTION

Waypoint matrix → *Path Manager* block (multirotor). A MATLAB Function runs `uavCoveragePlanner` for surveys.

UAV Toolbox

FIXED-WINGBLOCK + FUNCTION

`uavDubinsConnection` between waypoints (MATLAB Function) → *Path Manager* block (fixed-wing: fly-by, loiter/orbit commands).

UAV Toolbox

Python / C++ instead?C++ OK · WRAP OR ROS 2

A small waypoint graph belongs in a MATLAB Function. City-scale routing is event-driven, not periodic: call a C++ routing service over ROS 2 rather than forcing it into a fixed-step block.

M4 · Dynamic object interactions (prediction, TTC)

CARBLOCK + FUNCTION

*Multi-Object Tracker* block + a MATLAB Function for constant-velocity prediction and TTC.

Automated Driving / Sensor Fusion · app: Bird's-Eye Scope

QUADCOPTERBLOCK + FUNCTION

Same tracker for other drones and birds. TTC becomes 3-D closest point of approach (CPA) in a MATLAB Function.

Sensor Fusion & Tracking

FIXED-WINGMATLAB FUNCTION

Detect-and-avoid: CPA with TCAS-like thresholds in a MATLAB Function. Closure rates are much higher, so the look-ahead is longer.

Sensor Fusion & Tracking

Python / C++ instead?KEEP IN SIMULINK

TTC/CPA logic stays in Simulink. A learned trajectory predictor follows the ML pattern: Python training, C++ runtime node.

M5 · Behaviour planning (FSM)\[3\]

CARSTATEFLOW

Stateflow chart: track speed / follow leader / decel to stop / stopped / yield / nudge.

Stateflow · Simulink Design Verifier · app: Sequence Viewer

QUADCOPTERSTATEFLOW

Flight-phase chart: arm → takeoff → transit → inspect → divert → RTL → land, plus a parallel failsafe state (link loss, geofence, EKF fault).

Stateflow · Simulink Design Verifier

FIXED-WINGSTATEFLOW

Roll → climb → cruise → loiter → approach → flare → land, plus go-around and a parallel stall-protection state. RTL ends in loiter over home.

Stateflow · Simulink Design Verifier

Python / C++ instead?KEEP IN SIMULINK

The strongest case for MBD: a readable chart, formal property proving with Simulink Design Verifier, and transition coverage. The C++ equivalent in ROS is BehaviorTree.CPP; use it only if the rest of the stack is ROS-native.

M6 · Reactive planning (rollout, dynamic window)

CARBLOCK + FUNCTION

Trajectory rollout / DWA in a MATLAB Function; or the *Vector Field Histogram* block.

Navigation

QUADCOPTERBLOCK

*Obstacle Avoidance* block (3-D VFH): it can also stop and hover.

UAV Toolbox

FIXED-WINGMATLAB FUNCTION

It cannot stop, so reactive avoidance is a MATLAB Function that picks an escape maneuver (max-bank turn, climb) from a few fixed primitives.

—

Python / C++ instead?KEEP IN SIMULINK

Small, periodic and deterministic, which suits a MATLAB Function well.

M7 · Smooth local planning + velocity profile\[2\] \[3\] \[5\]

CARBLOCK + FUNCTION

Cubic-spiral BVP with `fmincon` in a MATLAB Function, or `trajectoryOptimalFrenet`; then *Path Smoother Spline* → *Velocity Profiler* blocks.

Optimization · Navigation · Automated Driving

QUADCOPTERBLOCK + FUNCTION

`plannerRRTStar` in `occupancyMap3D` → `minsnappolytraj` with time allocation (MATLAB Function); or the *Polynomial Trajectory* / *Minimum Jerk Polynomial Trajectory* blocks.

Navigation · Robotics System · UAV Toolbox

FIXED-WINGMATLAB FUNCTION

`plannerRRTStar` with a Dubins state space; a MATLAB Function schedules airspeed in the band 1.3·V_s·√n … V_NE and feeds TECS.

Navigation · UAV Toolbox

Python / C++ instead?C++ OK · WRAP OR ROS 2

The spiral lattice and velocity profile fit a MATLAB Function. Sampling planners with growing trees (OMPL, Fast-Planner, EGO-Planner) need dynamic memory. Run them as a 10 Hz **ROS 2 node**, or wrap them with *S-Function Builder* using fixed maximum sizes.

Final project · Full planner in CARLA\[2\] \[4\] \[5\]

CARBLOCK + FUNCTION

All of the above + *Lateral / Longitudinal Controller Stanley*; `checkFree` on `vehicleCostmap`.

Automated Driving · Simulink Test · apps: Driving Scenario Designer, Test Manager

QUADCOPTERBLOCK + FUNCTION

Full multirotor stack in Unreal or PX4 SITL; test cases in Test Manager.

UAV Toolbox · Simulink Test · app: Flight Log Analyzer

FIXED-WINGBLOCK + FUNCTION

Full fixed-wing stack with the fixed-wing *Guidance Model* first, then the 6DOF; FlightGear for visualization.

UAV Toolbox · Aerospace Blockset · Simulink Test

Python / C++ instead?C++ OK · WRAP OR ROS 2

A realistic mix: Stateflow behaviour, Simulink control and estimation, and C++ perception/planning nodes, all joined over ROS 2.

### Quadcopter vs fixed-wing: summary by layer

| Layer | Quadcopter | Fixed-wing | Ref |
| --- | --- | --- | --- |
| **Plant** | *6DOF (Quaternion)* + thrust/torque mixer + motor dynamics; start from the Quadcopter Project (`asbQuadcopter`) | *6DOF* + aero database (*Digital DATCOM Forces and Moments* or *Aerodynamic Forces and Moments* from `Aero.FixedWing`) + propulsion; template `asbSkyHogg` | \[6\] |
| **Environment** | *COESA Atmosphere*, *WGS84 Gravity*, *Dryden Wind Turbulence*, *Discrete Wind Gust*, *Wind Shear*: all in Aerospace Blockset; they matter more for the fixed-wing |  |  |
| **Trim / linearize** | Hover trim is trivial | Essential: trim at several airspeeds (Steady State Manager app), linearize (Model Linearizer app) for short-period, phugoid and Dutch roll; gain-schedule on airspeed |  |
| **Guidance-design surrogate** | *Guidance Model* block (UAV Toolbox), multirotor or fixed-wing configuration. Tune guidance on it, then swap in the full 6DOF | \[7\] |  |
| **L1 mission** | Waypoint list, can stop and turn | Waypoints joined by `uavDubinsConnection`; fly-by / fly-over; loiter circles. `uavCoveragePlanner` for surveys (both) | \[5\] |
| **L2 behaviour (Stateflow)** | arm → takeoff → transit → hover/inspect → RTL → land | roll → climb → cruise → loiter → approach → flare → land, plus go-around and stall protection; RTL ends in loiter over home |  |
| **L3 local** | `plannerRRTStar` / A\* in `occupancyMap3D` → `minsnappolytraj`; differentially flat | RRT\* with a Dubins state space, or line + arc primitives with R ≥ V²/(g·tan φ_max) | \[5\] |
| **L4 speed / time** | Time allocation limited by tilt and thrust; v = 0 allowed | Airspeed band 1.3 V_s … V_NE (stall rises √n in turns); energy management with TECS |  |
| **L5 guidance + control** | Trajectory tracking: position → velocity → attitude → rate → mixer. *Waypoint Follower*, *Orbit Follower*, *Path Manager* blocks | Path following: L1 → bank; TECS → pitch and throttle; attitude autopilot → surfaces. Same follower blocks in fixed-wing mode | \[4\] |
| **Estimation** | INS core; vibration and hover attitude are the hard part | INS + pitot airspeed, often AoA/sideslip; wind estimation is essential |  |
| **Avoidance** | Can stop / hover / back up; *Obstacle Avoidance* block (3-D VFH) | Cannot stop: sensing range must cover at least one full turning circle |  |
| **Hardware path** | UAV Toolbox Support Package for PX4 (SITL → HIL → Pixhawk); Flight Log Analyzer app for logs; FlightGear interface (Aerospace Blockset) for visualization | \[4\] \[7\] |  |

### Minimum license set

1. Simulink, Stateflow, Control System Toolbox
2. Automated Driving Toolbox (car) or UAV Toolbox (drones)
3. Navigation Toolbox (maps, planners, Frenet)
4. Aerospace Blockset (6DOF, atmosphere, wind)
5. Optimization Toolbox (spirals, min-snap)
6. Nice to have: Sensor Fusion & Tracking, Model Predictive Control, Simulink Control Design, Vehicle Dynamics Blockset, Computer Vision + Deep Learning (Course 3)

### Reference examples

[\[1\]Automated Driving · Planning and Control (Stanley, MPC, velocity profiling examples)](https://www.mathworks.com/help/driving/planning-and-control.html) [\[2\]Automated Parking Valet in Simulink: route → Stateflow behaviour → RRT\* → smoother → velocity profiler → Stanley. The course architecture, ready-made](https://www.mathworks.com/help/driving/ug/automated-parking-valet-in-simulink.html) [\[3\]Highway Lane Change: Stateflow decisions + Frenet lattice planner](https://www.mathworks.com/help/driving/ug/highway-lane-change.html) [\[4\]UAV Package Delivery: full multirotor stack with PX4 and Unreal](https://www.mathworks.com/help/uav/ug/uav-package-delivery.html) [\[5\]Motion Planning with RRT for Fixed-Wing UAV: Dubins state space](https://www.mathworks.com/help/uav/ug/motion-planning-with-rrt-for-fixed-wing-uav.html) [\[6\]Model a Quadcopter Based on Parrot Minidrones (the Quadcopter Project, `asbQuadcopter`): 6DOF plant, estimation, control](https://www.mathworks.com/help/aeroblks/quadcopter-project.html) [\[7\]UAV Toolbox: Guidance Model, followers, PX4 support, Flight Log Analyzer](https://www.mathworks.com/products/uav.html)

Block and app names follow recent MATLAB releases; in yours, check any name with `doc <name>`. Open an example with `openExample` or from its page.

IND

## How industry splits the tools

[#](#s-industry)

There are three broad ways to build the software, and most real products use a mix: a **C++ + ROS 2** stack, a **Simulink / Stateflow / MATLAB** model-based stack, or a **combination** with a clear boundary between them. The descriptions below are typical patterns from public documentation, open-source projects and conference talks. Individual companies differ.

|  | C++ + ROS 2 | Simulink + Stateflow + MATLAB | Combination |
| --- | --- | --- | --- |
| **Strengths** | Free and open source; huge ecosystem (perception, SLAM, planners, drivers); runs on any Linux computer; easy to hire for | Design and verification in one place: trim and linearize, tuning apps, coverage, property proving, certified code generation, requirements traceability | Each part uses the tool it is best at. Model-based GNC core, C++ ecosystem around it |
| **Weaknesses** | Verification and determinism are up to you; hard to certify; controllers are often hand-tuned in code | License cost; weak for perception, large dynamic data structures and ML; closed ecosystem | Interface discipline (buses ↔ messages, units, frames, timing) and two toolchains to maintain |
| **Certification** | Possible but costly: every line is hand-verified | The established path for DO-178C/DO-331 and ISO 26262 control software (qualified code generators) | Certify the model-based core; keep the C++ parts at lower criticality or behind a safety monitor |
| **Typical home** | Robotics startups, research labs, robotaxi autonomy stacks, companion computers on drones | Automotive ECUs (powertrain, chassis, ADAS control), flight control and GNC, space GNC, HIL rigs | Most serious drone, ADAS and aerospace autonomy programs |

### Small-scale: startups, university teams, small drone and robotics firms

- **Flight controller:** PX4 or ArduPilot. Both are open-source C++ autopilots with their own estimators (EKF2 / EKF3), controllers and flight modes. Teams rarely rewrite L5; they tune it.
- **Companion computer** (Jetson, Raspberry Pi): ROS 2 nodes in C++ and Python for perception, SLAM and mission logic, talking to the autopilot over MAVLink / uXRCE-DDS in offboard mode.
- **Python** for everything data: ML training, log analysis, tuning scripts, simulation orchestration (Gazebo, AirSim, CARLA).
- **MATLAB/Simulink** where licences allow (student and startup programs help): control design, system identification, trade studies and flight-log analysis, often offline rather than on the vehicle. Some teams generate PX4 modules from Simulink through the UAV Toolbox PX4 support package.
- **Why:** speed of iteration, low cost, and no certification requirement yet.

### Large companies: automotive OEMs and Tier-1s, aerospace primes, robotaxi companies

- **Automotive control and ADAS functions** (adaptive cruise, lane keeping, braking, powertrain, chassis): overwhelmingly Simulink/Stateflow with Embedded Coder into AUTOSAR components, verified MIL → SIL → PIL → HIL under ISO 26262.
- **Automated-driving perception and planning:** mostly C++ on in-house middleware or ROS 2-derived frameworks (the open-source reference is Autoware, which is ROS 2 / C++). ML is trained in Python and deployed on GPU runtimes. Simulink appears in the vehicle-control layer and in test and simulation infrastructure.
- **Aerospace flight control and GNC:** model-based, with Simulink + Embedded Coder under DO-178C / DO-331, or Ansys SCADE for the highest criticality levels. Mission and autonomy software around it is usually C++, often with a safety monitor or run-time assurance guarding it. HIL rigs (e.g. dSPACE, Speedgoat) run Simulink plant models against the real flight computer.
- **Robotaxi / pure-autonomy companies:** mostly C++ runtime + Python ML + large custom simulation. MATLAB is used for analysis rather than as the product toolchain.
- **Why:** certification evidence, many engineers working on the same interfaces, and long product life. In these settings the upfront cost of model-based tooling pays off.

### Where each tool usually sits in the guidance stack

| Layer | Small teams (typical) | Large companies (typical) |
| --- | --- | --- |
| **Plant model / simulation** | Gazebo, AirSim, CARLA (C++/Python); Simulink for design studies | Simulink plants on HIL rigs; in-house C++ simulators for autonomy |
| **L0 perception, SLAM** | ROS 2 C++/Python nodes, PyTorch-trained models | C++ + GPU runtimes; Python for training and data |
| **L0 state estimation** | Autopilot's EKF (PX4 / ArduPilot) | Simulink or hand C/C++ estimators, certified with the flight/ECU software |
| **L1 mission** | Python/C++ mission scripts, QGroundControl / MAVSDK | C++ mission management; routing services |
| **L2 behaviour** | C++/Python state machines, BehaviorTree.CPP | Stateflow for mode logic in control software; C++ decision modules in autonomy stacks |
| **L3–L4 local planning, speed** | C++ planners (Fast-Planner, EGO-Planner, Nav2) | C++ planners; Simulink for ADAS-level planners (lane change, parking) with generated code |
| **L5 control** | Autopilot's C++ controllers, tuned | Simulink / SCADE generated code: the core model-based domain |
| **Verification** | Simulation scenarios, flight tests, pytest / gtest | Requirements-based MIL/SIL/PIL/HIL, coverage, formal checks, certification evidence |

### The combination pattern, drawn

**FIG**The boundary sits at the interfaces you already defined: object list, pose and path flow as ROS 2 messages into a model-based core that is generated to C/C++. Python never runs in the deployed loop.

Positioning for a GNC / MBD engineer

Your strongest position is the **combination**: own the model-based GNC core (Simulink/Stateflow, code generation, V&V), and be able to plug it into a ROS 2 / C++ system. That means reading and writing basic ROS 2 C++ nodes, generating a ROS 2 node or PX4 module from Simulink, and knowing where the certification boundary is. Pure-Simulink or pure-C++ profiles each cover only half of what drone and ADAS teams need.

[DOCGenerate a standalone ROS 2 node from SimulinkMathWorks · ROS Toolbox](https://www.mathworks.com/help/ros/ug/generate-a-standalone-ros2-node-from-simulink.html) [DOCAutomated Parking Valet with ROS 2 in SimulinkMathWorks · the combination pattern, end to end](https://www.mathworks.com/help/ros/ug/automated-valet-using-ros2-simulink.html)

## Author, licence and citation

**Author:** Ahmed Mohamed Ahmed Hassan, Aerospace Engineer.

**Licence:** [Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/). You may copy, share, adapt and use this material for any purpose, including teaching and commercial use, as long as you credit the author and indicate if changes were made. Third-party videos, documents and trademarks linked from this page belong to their owners and are not covered by this licence.

**Suggested citation:**

```
Hassan, A. M. A. (2026). The Guidance Stack: one mission, every layer.
A visual guide to the GNC guidance layer for cars, quadcopters and fixed-wing UAVs.
Version 1.0, last reviewed 19 September 2026. Licensed under CC BY 4.0.
```

Built around the University of Toronto's Self-Driving Cars Specialization on Coursera; not affiliated with or endorsed by the University of Toronto, Coursera, MathWorks, PX4 or any other organization named here. MATLAB, Simulink and Stateflow are trademarks of The MathWorks, Inc.