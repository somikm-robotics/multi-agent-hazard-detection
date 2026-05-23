# AI Detection Strategy

## Overview

The system employs multiple AI pipelines, each tailored to a specific hazard type. All pipelines were trained on synthetic datasets generated within Gazebo and Blender simulation environments and validated through end-to-end mission trials.

Detection is structured into two layers:

- **Real-time inference** — executed during aerial patrol and ground agent on-arrival tasks
- **Offline analysis** — executed post-mission on captured imagery for detailed hazard assessment

---

## Pipeline 1 — Fibrous Hazard (Asbestos) Detection

The fibrous hazard pipeline is a multi-stage system combining real-time detection, pixel-wise index regression, semantic segmentation, and spatial analysis.

### Dataset Preparation

- Images captured in Gazebo simulation at a cruise altitude of 4 metres
- Organized into three classes: `dust_plume`, `fibrous_hazard`, `background`
- Perceptual hashing (imagehash) used to identify and remove duplicate images
- Data augmentation: rotations, flips, HSV adjustments, scaling, brightness and contrast modifications — aggressive cropping avoided to preserve textural detail
- Dataset split: 70% training, 20% validation, 10% testing

### Stage 1 — Classification (Real-Time, Aerial)

**Model:** YOLOv8m-cls (Medium, Classification head)

| Class | Precision | Recall | F1-score |
|-------|-----------|--------|----------|
| background | 1.000 | 0.997 | 0.998 |
| dust_plume | 0.982 | 1.000 | 0.991 |
| fibrous_hazard | 1.000 | 1.000 | 1.000 |

All classes achieved F1 > 0.99. No confusion between dust plume and fibrous hazard classes. Classification triggers low-altitude scanning for confirmed hazard candidates.

### Stage 2 — Detection (Real-Time, Aerial)

**Model:** YOLOv8l (Large, Detection head) — 700 manually labelled images, Roboflow annotation

| Class | Precision | Recall | mAP@0.5 | mAP@0.5:0.95 |
|-------|-----------|--------|---------|--------------|
| fibrous_hazard | 0.999 | 1.000 | 0.995 | 0.989 |

Near-perfect performance across all IoU thresholds. Robust to shape, size, camera angle, and partial occlusion variations.

### Stage 3 — Asbestos Index Regression (Offline)

**Model:** MLP Regressor (3-layer) — pixel-wise asbestos index estimation ∈ [0, 1]

- 25 representative fibrous regions manually labelled with grayscale index maps
- RGB–index pairs extracted from 700 cropped detection patches → over 2.4 million training samples
- K-Means clustering (k=25) used to curate representative training images

| Metric | Value |
|--------|-------|
| MSE | 0.0665 |
| MAE | 0.1799 |
| R² | 0.4188 |

The regressor captured overall color-to-index mapping effectively, particularly at high and low index regions. Auto-labelled an additional 1,400+ patches to expand the segmentation training set.

### Stage 4 — Segmentation (Offline)

**Model:** DeepLabV3+ (ResNet50 backbone)

- Trained on 700 index maps (560 training, 70 validation, 70 testing)
- Produces dense per-pixel asbestos index maps
- Converged stably, generalizing across textures and illumination changes

### Stage 5 — Spatial Analysis (Offline)

Predicted index maps binarized at threshold 0.6, analyzed using OpenCV and skimage regionprops:

| Metric | Value |
|--------|-------|
| Min area | 0.13 m² |
| Max area | 3.22 m² |
| Mean area | 0.65 m² |
| Std deviation | 0.58 m² |
| Total regions identified | 753 |

Outputs: area (m²), centroid locations, bounding box dimensions, spread characteristics.

---

## Pipeline 2 — Dust Plume Detection & Density Estimation

The dust plume pipeline combines real-time detection, AI-based density estimation, and simulated sensor measurements.

### Stage 1 — Classification (Real-Time, Aerial)

Shared with fibrous hazard pipeline — YOLOv8m-cls. Dust plume classification F1 = 0.991.

### Stage 2 — Detection (Real-Time, Aerial)

**Model:** YOLOv8x (Extra Large, Detection head) — highest accuracy variant, 150 training epochs

| Class | Precision | Recall | mAP@0.5 | mAP@0.5:0.95 |
|-------|-----------|--------|---------|--------------|
| dust_plume | 1.000 | 1.000 | 0.995 | 0.994 |

Complete absence of false positives and false negatives. Robust across shape, size, and camera angle variations.

### Stage 3 — Density Estimation (Real-Time, Ground Agent On-Arrival)

**Model:** ResNet18 (pretrained on ImageNet, FC layer replaced with single continuous output)

Dataset preparation:
- 27 distinct dust plume textures generated via Gazebo and Blender pipeline
- ~90,000 synthetic images captured from multiple camera angles
- Subsampled 1-in-10 → ~9,000 images
- Density labels extracted from folder naming conventions via regex
- Split: 70% training / 20% validation / 10% testing

| Metric | Value |
|--------|-------|
| MSE | 0.0375 |
| MAE | 0.1548 |
| R² | -1.5811 |

Absolute error metrics confirm reasonable accuracy. Negative R² reflects dataset variance constraints rather than model failure — model successfully separated three density classes (0.60, 0.75, 0.90) with minimal overlap across 27 textures and multiple camera viewpoints.

Density outputs classified into severity categories: Low, Moderate, High, Very High, Severe — published in real time to supervisory dashboard. Alarm triggered if threshold exceeded.

### Stage 4 — Segmentation & Spatial Measurement (Offline)

- **YOLOv8x** bounding box detection → binary segmentation masks via **SAM** (prototyping) and **DeepLabV3+** (quantitative results)
- Plume area computed via non-zero pixel count in binary masks (calibratable to m²)
- Centroid extracted via skimage regionprops
- Spread direction computed from centroid displacement across sequential frames (arctan2)

| Metric | Value |
|--------|-------|
| Mean area | 25,187.75 px |
| Std dev area | 6,627.79 px |
| Mean speed | 4.94 px/frame |

### Dust Plume Sensor Simulation

A dedicated C++ Gazebo plugin (`DustPlumeSensorPlugin`) simulates realistic dust sensor fields:

- Embedded in the dust plume SDF model
- Calculates robot proximity to plume center on every simulation tick
- `InsideFactor` function returns value decreasing linearly from plume center outward (0 outside plume)
- PM1.0, PM2.5, PM10 values computed as `max_PM × inside_factor`
- Publishes `DustSensorResult` continuously to `/dust/agilex_diff_drive`
- Operates passively — no external triggers required

`DustSensorRelayNode` (Python) subscribes to plugin output and relays the first valid non-zero reading per mission to `/dust_sensor_reading`.

---

## Pipeline 3 — Toxic Gas Detection

Toxic gas detection does not use a trained AI model — it uses stochastic concentration modelling with distance-based sensing, implemented via C++ Gazebo plugins.

### Simulation Architecture

**GasEmitterPlugin** (embedded in hazard marker SDF):
- Publishes baseline gas concentration values with parameterized random noise
- Gases modelled: CO, NH₃, NO₂, VOCs
- Emission data associated with precise marker coordinates

**GasSensorPlugin** (embedded in Crazyflie SDF):
- Subscribes to GasEmitterPlugin output
- Calculates real-time spatial distance between drone and emission source
- Applies exponential decay model to simulate gas dispersion over distance
- Publishes `ToxicityResult` message with adjusted concentrations and toxicity status

### Classification Logic

`ToxicityMeasurementNode` classifies readings into four states:

| State | Response |
|-------|----------|
| Safe | No action |
| Elevated | Readings published to supervisory dashboard |
| Dangerous | Alarm triggered, readings published, hazard search suspended |
| Emergency | Immediate return-to-base initiated |

**Safety logic:** 3× Dangerous readings OR 1× Emergency reading → immediate emergency landing procedure, overriding all ongoing patrol and mission activities.

### Validation Results

- Stochastic emission values combined with distance-based sensing produced realistic variation in state transitions across missions
- Some missions: Elevated → Dangerous → return-to-base
- Other missions: direct escalation to Emergency → immediate landing
- Safety logic consistently enforced conservative mission behaviour across all test runs

---

## Summary of AI Performance

| Pipeline | Model | Key Metric | Result |
|----------|-------|-----------|--------|
| Hazard classification | YOLOv8m-cls | F1 (all classes) | > 0.99 |
| Hazard detection (general) | YOLOv8x | mAP@0.5 | 0.995 |
| Fibrous detection | YOLOv8l | mAP@0.5 | 0.995 |
| Asbestos index regression | MLP | MAE | 0.1799 |
| Asbestos segmentation | DeepLabV3+ | Qualitative | Consistent across textures |
| Dust density estimation | ResNet18 | MAE | 0.1548 |
| Toxic gas detection | Stochastic modelling | Safety logic | Consistently enforced |
