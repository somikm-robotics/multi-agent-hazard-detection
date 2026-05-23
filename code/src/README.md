# Multi-Agent Mining Hazard Detection System

This repository contains the MSc project **Multi-Agent Robot System for Hazard Detection & Assessment in Ore Mining**.  
The system integrates aerial, ground, supervisory, and material-handling agents in simulation to detect and respond to multiple hazards relevant to mining environments.  


It is fully Dockerized, providing a reproducible environment with **ROS 2 Iron** and **Gazebo Fortress**.  
The Docker image includes all system, ROS, and Python dependencies required to reproduce the experiments.

---

## Project Overview

Mining environments present diverse operational hazards that threaten both safety and productivity.  
This system demonstrates a **multi-agent robotic framework** capable of detecting and responding to:

- **Fibrous hazards (asbestos-like materials)** – analysed with AI-based detection and asbestos index mapping.  
- **Dust plumes** – detected and quantified with AI pipelines for segmentation and density estimation.  
- **Toxic gases** – modelled as stochastic concentration fields, with drone-based sensing triggering conservative safety logic.  
- **Conveyor belt hazards** – including spillage, misalignment, overloading, and blockages, classified with a ResNet-18 model.

### Agents

- **Aerial Agent (Crazyflie drone)**: patrols, detects hazards (dust, fibrous, toxic gas), and reports results.  
- **Ground Agent (Agilex robot)**: performs inspections, orbiting, and data collection around hazards.  
- **Supervisory Agent (UI dashboard)**: provides real-time alerts, manual override, and mission approval/cancellation.  
- **Material Handling Agent (Conveyor system):** runs the conveyor belt classifier in a scrollable OpenCV window, detecting spillage, misalignment, blockages, and overloading in real time.  

### Key Features

- Full simulation in **Gazebo Fortress + ROS 2 Iron**, with reproducible Docker environment.  
- End-to-end missions: patrol → detect hazard → inspect/orbit → return-to-base.  
- AI pipelines for hazard detection, classification, and quantification.  
- Supervisor dashboard for situational awareness and manual interventions.  
- Independent conveyor belt classifier demo (scroll window).  

---

## Table of Contents
1. [Build the Docker Image](#1-build-the-docker-image)  
2. [Create the Docker Container](#2-create-the-docker-container)  
3. [Start and Enter the Container](#3-start-and-enter-the-container)  
4. [Build the ROS 2 Workspace](#4-build-the-ros-2-workspace)  
5. [Run the Applications](#5-run-the-applications)  
   - [Main Multi-Agent System + Supervisory Agent](#a-main-multi-agent-system--supervisory-agent)  
   - [Conveyor Belt Classifier Scroll (Independent Demo)](#b-conveyor-belt-classifier-scroll-independent-demo)  
6. [Exiting](#6-exiting)  
7. [AI Models and Datasets](#7-ai-models-and-datasets)
8. [Robot Models](#8-robot-models)
9. [Troubleshooting Notes](#9-troubleshooting-notes)
10. [Incident Times](#10-incident-times)

---

## 1. Build the Docker Image
From the folder containing the `Dockerfile` and `requirements.txt`:
```bash
docker build -t ros2_iron_fort_nav2_image .
```

---

## 2. Create the Docker Container
Allow GUI applications (Gazebo, RViz) to connect:
```bash
xhost +local:root
```

Then create the container (adjust the path if needed):
```bash
docker create   --name ros2_iron_fort_nav2_dev   --env DISPLAY=$DISPLAY   --env QT_X11_NO_MITSHM=1   --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR   --volume /tmp/.X11-unix:/tmp/.X11-unix:rw   --volume $HOME/.Xauthority:/root/.Xauthority:rw   --volume $HOME/comp0247_multi_agent_mining/ros2_ws:/root/ros2_ws   --net=host   --privileged   -it ros2_iron_fort_nav2_image
```

Optional (faster with GPU support): add  
```bash
  --gpus all   --runtime=nvidia ```

---

## 3. Start and Enter the Container
Start the container:
```bash
sudo docker start ros2_iron_fort_nav2_dev
```

Open a shell inside:
```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
```

---

## 4. Build the ROS 2 Workspace
Inside the container:
```bash
cd ~/ros2_ws
colcon build --merge-install --symlink-install
```

---

## 5. Run the Applications

### A. Main Multi-Agent System + Supervisory Agent
This demo requires three terminals.

**Terminal 1 – Multi-Agent System**

Before launching, set the desired hazard type in `shared_infrastructure/config/config.json`:

```json
"current_hazard_target": "Dust Plume"  // options: "Dust Plume", "Fibrous Hazard", "Toxic Gas"
```

```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch multi_agent multi_agent_launch.py
```
Expected output: Gazebo world launches and RViz visualization opens.

**Terminal 2 – Supervisory Agent**
```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch supervisory_agent supervisory_launch.py
```
Expected output: Supervisory dashboard window pops up.

**Terminal 3 – Nav2 (Navigation)**
```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ground_agent navigation_agilex.launch.py
```


---

### B. Conveyor Belt Classifier Scroll (Independent Demo)
This demo runs separately from the multi-agent system. Use a new terminal: 


#### Step 1: Generate Classified Conveyor Images

```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
cd ~/ros2_ws/src/material_handling_agent/scripts
python3 conveyor_classifier.py
```
Classified images with hazard labels and confidence values will be saved to: /tmp/annotated_images

#### Step 2: Run the Scrolling Window Demo

```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
cd ~/ros2_ws
source install/setup.bash
ros2 run material_handling_agent conveyor_classifier_slow_scroll_node
```

Expected output: Conveyor belt scroll window opens with classification results.

---

## 6. Exiting
When finished, type:
```bash
exit
```
in each container terminal.  

The container remains running in the background and can be re-entered with:
```bash
sudo docker exec -it ros2_iron_fort_nav2_dev bash
```

---


## 7. AI Models and Datasets

### AI Models
Pre-trained model weights are not included in this repository due to file size.
Download `ai_models.zip` from [Google Drive](https://drive.google.com/drive/folders/1F_m-Ortg4vahS6LnpLxdWxQkn8y391b4) and extract 
into the corresponding `ai_models/` folder within each agent package.

| Model | Agent | Purpose | File |
|-------|-------|---------|------|
| YOLOv8m-cls | Aerial | Hazard classification | `aerial_agent/yolo/classification/best.pt` |
| YOLOv8x | Aerial | Hazard detection | `aerial_agent/yolo/detection/yolov8x_best.pt` |
| ResNet18 | Ground | Dust plume density estimation | `ground_agent/density_regressor.pt` |
| ResNet18 | Material Handling | Conveyor belt classification | `material_handling_agent/best_loss.pth` |

### Datasets
Raw training datasets are not included in this repository.
Available on request

---

## 8. Robot Models

| Robot | Source | License |
|-------|--------|---------|
| Crazyflie | [crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation) (Bitcraze AB) | MIT |
| Agilex Scout | [scout_ros2](https://github.com/agilexrobotics/scout_ros2) (AgileX Robotics) | Apache 2.0 |

- Crazyflie model and meshes → `aerial_agent/models/crazyflie/` — unmodified
- Agilex meshes → `ground_agent/models/agilex/meshes/` — unmodified, see `NOTICE` and `LICENSE_AGILEX` for attribution
- Agilex URDF and model.sdf → `ground_agent/urdfs/agilex.urdf` — modified from original for this project

---

## 9. Troubleshooting Notes
All required ROS 2 and Gazebo packages are installed automatically in the Docker image.  
However, in case of version conflicts or missing dependencies, you may manually clone the upstream packages into `ros2_ws/src` and rebuild:

```bash
cd ~/ros2_ws/src
# Example: ros2_control
git clone https://github.com/ros-controls/ros2_control.git -b iron
git clone https://github.com/ros-controls/ros2_control_demos.git -b iron
git clone https://github.com/ros-controls/ros2_controllers.git -b iron

# Example: navigation2
git clone https://github.com/ros-planning/navigation2.git -b iron

# Example: ignition gazebo bridge
git clone https://github.com/gazebosim/ros_gz.git -b iron
```

## 10. Incident Times

- **Asbestos mission** — key events at approximately: 2, 4, 10 minutes, 
  and the final 1–2 minutes
- **Dust plume mission** — key events at approximately: 1–2, 5–6, and 8 minutes

Mission videos available from [Google Drive](your-link-here).

  

