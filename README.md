# Multi-Agent Robot System for Hazard Detection & Assessment in Ore Mining
**ROS2 · Nav2 · Gazebo Fortress · Python · AI/ML · Multi-Agent Systems**

> **MSc Robotics & Artificial Intelligence — University College London (Distinction)**

---

## Overview

A multi-agent robotic framework for autonomous hazard detection and 
assessment in open-pit ore mining environments. The system integrates 
three coordinated agents — an aerial drone, a wheeled ground robot, and 
a human supervisory agent — operating within a ROS2 Iron and Gazebo 
Fortress simulation.

The system goes beyond simple waypoint navigation to create an 
intelligent, safety-critical inspection framework capable of detecting, 
assessing, and responding to multiple hazard types in real time. A 
standalone material handling agent extends the framework to conveyor 
belt monitoring, classifying operational hazards including blockages, 
spillage, overloading, and misalignment using a ResNet-18 classifier 
trained on synthetic ore datasets.

---

## Hazards Addressed

| Hazard | Detection Method |
|--------|----------------|
| Fibrous materials (asbestos) | AI classification + YOLOv8 detection + MLP index regression |
| Dust plumes | YOLOv8 detection + DeepLabV3+ segmentation + density estimation |
| Toxic gases | Stochastic concentration modelling + electrochemical sensor simulation |
| Conveyor belt anomalies | ResNet-18 classifier (5 operational categories) |

---

## Agents & Architecture

Three coordinated agents form the system:

**Aerial Agent (Crazyflie)** — primary sensing platform, continuously 
patrols the mining site, detects hazards using onboard AI pipelines, 
monitors toxic gas concentrations, and publishes hazard notifications.

**Ground Agent (Agilex)** — responds to aerial notifications, navigates 
autonomously to hazard locations via Nav2, and executes hazard-specific 
on-arrival tasks (orbital survey for fibrous hazards, density measurement 
for dust plumes).

**Supervisory Agent** — human oversight via Tkinter dashboard, manages 
mission override window (approve / cancel), and issues manual scan 
commands to the aerial agent.

See [System Architecture](docs/architecture/system_architecture.md) for 
full component breakdown, communication tables, and coordination model.

---

## Mission Workflows

Three mission workflows are supported, each triggered by aerial agent detection:

**Fibrous Hazard** — two-phase mission: initial inspection (spin, image 
capture) followed by a full 360-degree orbital survey for asbestos 
analysis. Two override windows — one before each phase.

**Dust Plume** — single-phase mission: ground agent navigates to hazard 
site, performs real-time AI density estimation and dust sensor 
measurements. Alarm triggered if density threshold exceeded.

**Toxic Gas** — aerial only. Four-state classification (Safe → Elevated 
→ Dangerous → Emergency). Three Dangerous readings or single Emergency 
triggers immediate return to base.

See [Mission Workflows](docs/design/mission_workflows.md) for full detail.

---

## AI Pipelines

Three hazard-specific AI pipelines run across aerial and ground agents:

**Fibrous Hazard** — YOLOv8l real-time detection (mAP@0.5 = 0.995), 
followed by offline MLP asbestos index regression, DeepLabV3+ 
segmentation, and spatial analysis.

**Dust Plume** — YOLOv8x detection (mAP@0.5 = 0.995), ResNet18 
real-time density estimation, DeepLabV3+ segmentation, and plume 
spatial measurement (area, centroid, spread direction).

**Toxic Gas** — stochastic concentration modelling with distance-based 
sensing. Four-state classification (Safe → Elevated → Dangerous → 
Emergency). Conservative safety logic enforced throughout.

See [AI Detection Strategy](docs/design/ai_detection_strategy.md) for 
full pipeline detail and performance metrics.

---


## Conveyor Belt Monitoring++++++++++++++++++

A standalone material handling agent monitors industrial conveyor belts 
using a ResNet-18 classifier trained on synthetic ore datasets generated 
in Blender. Five categories: Normal, Blocked, Spillage, Overloaded, 
Misalignment. Perfect accuracy achieved across all categories on the 
held-out test set.

See [Conveyor Belt Classifier](docs/design/conveyor_belt_classifier.md) 
for full detail.


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
| Language | Python , C++ (plugins) |
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
