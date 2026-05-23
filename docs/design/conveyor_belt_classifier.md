# Conveyor Belt Monitoring

## Overview

The conveyor belt monitoring system is a standalone material handling agent (`material_handling_agent`) within the multi-agent framework. It uses a vision-based AI classifier to autonomously detect and classify operational hazards on industrial conveyor belts in real time.

The system was developed entirely using synthetic datasets generated in Blender, demonstrating the feasibility of AI-based conveyor monitoring without requiring real-world imagery for training.

---

## Hazard Categories

Five operational categories are classified:

| Category | Description |
|----------|-------------|
| Normal | Baseline safe operation |
| Blocked | Obstruction preventing material flow |
| Spillage | Ore spillage off the belt |
| Overloaded | Excessive material load on belt |
| Misalignment | Belt tracking deviation |

---

## Dataset Generation

### Asset Creation (Blender)
- Manual prototypes of ores and conveyor belts modelled in Blender to validate scale, proportions, and belt geometry
- Automated Blender scripts used for bulk asset generation
- Ore palette: 31 colours based on verified mineral references, each scaled into three size variants (small, medium, large)
- Conveyor variants generated with different belt widths
- All assets exported as COLLADA (.dae) meshes for ROS2 integration

### Hazard Configuration
- YAML configuration files defined per hazard type — controlling mesh selection, lighting conditions, and number of conveyors
- Categories with limited visual variability (Normal, Overloaded, Misalignment) received lighting augmentation at render time to expand dataset diversity

### Dataset Assembly
- Images rendered from Blender pipelines and organised into training, validation, and test directories
- Dataset balanced across all five hazard categories
- Normal class intentionally upsampled to reflect real-world operating conditions where safe states occur more frequently than hazards

### Key Design Decision — Belt Colour
Initial experiments introduced bright conveyor belt colours (red, green, blue) to increase variability. The model exploited belt colour as a shortcut, achieving high validation accuracy but poor inference performance. Conveyors were restricted to neutral variants (dark, light, brown), forcing the network to learn hazard-relevant features — ore distribution and pile geometry — rather than belt artefacts.

---

## Model

**Architecture:** ResNet-18 (pretrained on ImageNet)

**Training configuration:**
- 30 epochs
- Optimiser: Adam (learning rate 1×10⁻⁴)
- Scheduler: cosine annealing
- Loss function: cross-entropy
- Data augmentation: random crops, rotations, horizontal flips, mild colour jitter — aggressive erasing avoided to preserve ore features

**Class ordering** (ImageFolder alphabetical sort):
`[Blocked, Misalignment, Normal, Overloaded, Spillage]`

Checkpoints saved for best accuracy, best loss, and final epoch.

---

## Results

| Class | Precision | Recall | F1-score | Support |
|-------|-----------|--------|----------|---------|
| Blocked | 1.00 | 1.00 | 1.00 | 76 |
| Misalignment | 1.00 | 1.00 | 1.00 | 57 |
| Normal | 1.00 | 1.00 | 1.00 | 112 |
| Overloaded | 1.00 | 1.00 | 1.00 | 57 |
| Spillage | 1.00 | 1.00 | 1.00 | 76 |
| **Overall accuracy** | | **1.00** | | **378 images** |

Perfect precision, recall, and F1-score across all five categories on the held-out test set. No confusion between hazard and non-hazard classes.

---

## Inference & Demonstration

### Standalone Inference
`conveyor_classifier.py` runs inference on 45 conveyor images in sequence:
- Loads trained ResNet-18 model
- Classifies each image
- Overlays predicted class label and confidence score onto the frame
- Saves annotated outputs to `/tmp/annotated_images`

### ROS2 Scrolling Demonstration
`ConveyorClassifierSlowScrollNode` presents results as a scrolling video simulating a real conveyor sequence:
- Reads ore order from YAML configuration file
- Loads annotated images and displays in OpenCV window
- Publishes to ROS2 topics:
  - `/conveyor/classifier_output` — classification text
  - `/conveyor/slideshow_image` — annotated images
- First frame appears immediately, subsequent frames every 10 seconds — simulating continuous ore flow on a real conveyor belt

---

## Key Design Characteristics

**Synthetic-only training**
The pipeline was trained entirely on synthetic Blender-generated imagery. Results confirm the feasibility of synthetic data for real-time hazard classification without real-world training data.

**Feature learning over shortcuts**
Restricting belt colours to neutral variants forced the model to learn ore distribution and geometry as discriminative features — a deliberate design decision that significantly improved inference robustness.

**Transferability**
The pipeline is directly transferable to real conveyor video streams. Integration into the supervisory dashboard would enable automated alerts during material handling operations, complementing fibrous, dust plume, and toxic gas hazard detection.

**Lightweight architecture**
ResNet-18 was selected over ResNet-50 for stronger generalisation on a relatively small synthetic dataset, and for its computational efficiency in real-time deployment scenarios.

---

## Future Work

- Validation on real conveyor imagery
- Dataset augmentation with domain-specific variations (lighting changes, partial occlusions, background clutter)
- Integration into supervisory dashboard for automated alerting
- Deployment on edge hardware for real-time industrial monitoring
