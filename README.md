# Multi-Agent Robot System for Hazard Detection & Assessment in Ore Mining
**ROS2 · Nav2 · Gazebo Fortress · Python · AI/ML · Multi-Agent Systems**

> **MSc Robotics & Artificial Intelligence — University College London (Distinction)**

---

## Overview

A multi-agent robotic framework for autonomous hazard detection and assessment in open-pit ore mining environments. The system integrates three coordinated agents — an aerial drone, a wheeled ground robot, and a human supervisory agent — operating within a ROS2 Iron and Gazebo Fortress simulation.

The system goes beyond simple waypoint navigation to create an intelligent, safety-critical inspection framework capable of detecting, assessing, and responding to multiple hazard types in real time.

---

## Hazards Addressed

| Hazard | Detection Method |
|--------|----------------|
| Fibrous materials (asbestos) | AI classification + YOLOv8 detection + MLP index regression |
| Dust plumes | YOLOv8 detection + DeepLabV3+ segmentation + density estimation |
| Toxic gases | Stochastic concentration modelling + electrochemical sensor simulation |
| Conveyor belt anomalies | ResNet-18 classifier (5 operational categories) |

---

## Agents

### Aerial Agent — Crazyflie Drone
- Continuously patrols the mining site following predetermined patrol paths
- Detects fibrous hazards and dust plumes using onboard AI pipelines
- Monitors toxic gas concentrations using simulated electrochemical sensors
- Publishes hazard notifications (location, type, diameter) to ground and supervisory agents
- Executes return-to-base on hazard confirmation or emergency gas levels

### Ground Agent — Agilex Wheeled Robot
- Awaits hazard notifications from the aerial agent
- Navigates autonomously to hazard locations using Nav2 and computed waypoints
- Executes hazard-specific on-arrival behaviour:
  - **Fibrous hazards** — initial inspection spin, return to base, full orbital survey for asbestos analysis
  - **Dust plumes** — real-time density measurement, threshold-based alarm triggering
- Returns to base on mission completion

### Supervisory Agent — Human Expert (Tkinter Dashboard)
- Receives real-time hazard alerts with location and visual context
- Manages override window — approve, cancel, or modify ground missions
- Issues manual "Scan now" commands to the aerial agent
- Monitors mission status, toxicity readings, and dust density in real time

---

## System Architecture

The system is structured into three coordinated agent packages plus shared communication interfaces:

**Aerial Agent (Crazyflie)**
- `CrazyfliePatrol` — autonomous patrol with scan incident interpolation
- `HazardDetection` — AI-based classification and detection of fibrous and dust hazards
- `ToxicityMeasurement` — gas concentration monitoring and threshold-based alerting
- `Notifier` — publishes hazard pose, type, and parameters on detection
- `TFPublisher` — broadcasts drone position for RViz visualisation

**Ground Agent (Agilex)**
- `MissionHandlerNode` — central orchestrator, parses notifications, manages override window, coordinates mission execution
- `PathPlannerNode` — computes optimal navigation paths to hazard locations
- `NavigationNode` — executes autonomous movement via Nav2
- `FibrousHazardOrbitTwistCommander` — commands 360-degree orbital surveys for asbestos analysis
- `DustPlumeDensityEstimation` — estimates dust density at hazard site
- `DustSensorRelay` — publishes real-time dust sensor readings
- `OnArrivalTask` — coordinates hazard-specific actions on arrival
- `InitialInspection` — performs initial survey and image capture for fibrous hazards

**Supervisory Agent**
- `AlertReceiver` — displays hazard notifications with location and visual context
- `MissionOverrider` — override window management, mission approval/cancellation
- `UIInterface` (Tkinter) — real-time dashboard for alerts, toxicity, and mission status

See [System Architecture](docs/architecture/system-architecture.md) for full detail.

---

## AI Pipelines

### Fibrous Hazard (Asbestos) Detection
- **Classification** — ResNet-based model: F1 > 0.99 across all classes
- **Detection** — YOLOv8l: precision 0.999, recall 1.000, mAP@0.5 = 0.995
- **Index Regression** — MLP pixel-wise asbestos index estimation ∈ [0,1]
- **Segmentation** — DeepLabV3+ for spatial analysis and hazard boundary mapping
- Offline hyperspectral-based proxy asbestos index mapping

### Dust Plume Analysis
- **Detection** — YOLOv8x: mAP@0.5 = 0.995, mAP@0.5:0.95 = 0.994
- **Segmentation** — DeepLabV3+ / SAM for plume boundary extraction
- **Density Estimation** — regression model for plume density quantification
- **Measurements** — plume area, centroid, spread direction across sequential frames

### Toxic Gas Monitoring
- Stochastic concentration field modelling with distance-dependent sensing
- Four-state classification: Safe → Elevated → Dangerous → Emergency
- Conservative safety logic: emergency landing on 3× Dangerous or single Emergency reading

### Conveyor Belt Monitoring
- **ResNet-18 classifier** — 5 categories: normal, blocked, spillage, overloaded, misalignment
- Synthetic dataset generation with scrolling simulation for realistic training
- Perfect accuracy achieved across all five operational categories

See [AI Detection Strategy](docs/design/ai-detection-strategy.md) for full detail.

---

## Communication Architecture

ROS2 publish-subscribe and action-based coordination across all three agents.

### Key Topics

| Topic | Publisher | Subscriber(s) | Purpose |
|-------|-----------|--------------|---------|
| `/hazard_notification` | Aerial Agent (Notifier) | Ground Agent, Supervisory Agent | Hazard location, type, parameters |
| `/override_command` | Supervisory Agent | Ground Agent (Mission Handler) | Mission approve / cancel / modify |
| `/mission_status` | Ground Agent | Supervisory Agent | Mission progress and completion |
| `/toxicity_result` | Aerial Agent | Supervisory Agent | Gas readings and hazard location |
| `/density_sensor_result` | Ground Agent | Supervisory Agent | Real-time dust sensor readings |
| `/density_estimation_result` | Ground Agent | Supervisory Agent | Estimated dust plume density |
| `/navigation_result` | Ground Agent (Navigation) | Mission Handler, UI | Navigation completion status |
| `/base_return` | Ground Agent | Supervisory Agent | Return-to-base notification |

### Key Actions & Services

| Action / Service | Client | Server | Purpose |
|-----------------|--------|--------|---------|
| `navigate_to_pose` | Mission Handler | Navigation | Goal pose after path planning |
| `cancel_mission` | Supervisory Agent | Mission Handler | Cancel within override window |
| `approve_mission` | Supervisory Agent | Mission Handler | Immediate approval |
| `scan_incident` | Supervisory Agent | Crazyflie Patrol | Manual drone scan request |
| `classify_hazard` | Crazyflie Patrol | Hazard Detection | Classification of candidate hazard |
| `detect_hazard` | Crazyflie Patrol | Hazard Detection | Object detection for identification |
| `request_path_plan` | Mission Handler | Path Planner | Optimal path computation |
| `orbit_hazard` | Fibrous Orbit Commander | Navigation | 360-degree orbit manoeuvre |

---

## Mission Workflows

### Fibrous Hazard Mission
1. Aerial agent detects fibrous hazard, publishes notification
2. Override window opens (5 minutes, configurable)
3. Supervisory agent approves / cancels / modifies
4. Ground agent navigates to hazard location
5. Initial inspection — spin in place, image capture
6. Return to base → second override window
7. If approved: navigate back, execute full orbital survey
8. Return to base, mission complete

### Dust Plume Mission
1. Aerial agent detects dust plume, publishes notification
2. Override window → supervisory decision
3. Ground agent navigates to hazard site
4. Real-time dust density measurement
5. Alarm triggered if density exceeds safety threshold
6. Return to base regardless of measurement outcome

### Toxic Gas Mission (Aerial Only)
1. Aerial agent monitors gas concentrations during patrol
2. Elevated / Dangerous / Emergency states published to supervisory dashboard
3. Three Dangerous readings or single Emergency → immediate return to base
4. Ground agent does not participate in toxic gas missions

---

## Key Results

| Subsystem | Result |
|-----------|--------|
| Hazard classification (F1) | > 0.99 across all classes |
| Hazard detection (YOLOv8x mAP@0.5) | 0.995 |
| Fibrous detection (YOLOv8l mAP@0.5) | 0.995 |
| Conveyor belt classification | Perfect accuracy — 5 categories |
| End-to-end mission completion | Reliable across all hazard types |
| Fibrous hazard mission runtime | ~16 minutes |
| Dust plume mission runtime | ~9 minutes |
| Toxic gas mission runtime | ~5 minutes |

---

## Technical Challenges & Solutions

| Challenge | Solution |
|-----------|----------|
| Gazebo Classic / Fortress plugin incompatibilities | URDF corrections and plugin migration |
| GPS–IMU fusion for outdoor navigation | Sensor fusion tuning for realistic localisation |
| Nav2 instability on rough mining terrain | Transparent terrain overlay — halved mission runtimes while preserving visual realism |
| Complex fibrous hazard orbit manoeuvres | Custom orbit twist commander with safety radius calculation |

---

## Tech Stack

| Category | Technologies |
|----------|-------------|
| Robotics Framework | ROS2 Iron, Nav2 |
| Simulation | Gazebo Fortress |
| Language | Python |
| AI / Detection | YOLOv8x, YOLOv8l, ResNet-18, DeepLabV3+, SAM |
| ML / Regression | MLP (asbestos index), density regression |
| Data / Annotation | Roboflow, synthetic dataset generation |
| Visualisation | RViz, Tkinter, Matplotlib, OpenCV |
| Platforms | Crazyflie (aerial), Agilex (ground) |

---

## Project Status

Completed — MSc submission, UCL, 2025. Distinction awarded.

Simulation-only implementation. Future work identified:
- Migration to real Crazyflie and Agilex hardware platforms
- Advanced Nav2 parameter tuning for real-world terrain
- Lightweight model deployment (YOLOv8s) for embedded hardware constraints
- Validation against real-world field data

---

*MSc Robotics & Artificial Intelligence — University College London — Distinction*
