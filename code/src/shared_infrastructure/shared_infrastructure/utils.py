from shared_infrastructure.hazard_types import HazardType
from shared_infrastructure.task_types import TaskType
from shared_infrastructure.ore_spawn_types import OreSpawnType
import json
import rclpy, sys
from rclpy.node import Node
from typing import List, Tuple
# from rclpy.context import get_default_context
from rclpy.utilities import ok
import  os, subprocess, signal, time
from std_msgs.msg import String
import math
from typing import Literal, Tuple
from geometry_msgs.msg import Pose

DEFAULT_Z = 0.05

HAZARD_POSES = {
    HazardType.FIBROUS_HAZARD: (1.5, 4.0, DEFAULT_Z),
    HazardType.DUST_PLUME:       (-2.0, -6.0, DEFAULT_Z),
    HazardType.TOXIC_GAS:      (8.0, 0.0, DEFAULT_Z),
    HazardType.MATERIAL_HANDLING: (8.0, 0.0, 0.0),
}

Edge = Literal["north", "east", "south", "west"]

CONFIG_PATH = "/root/ros2_ws/src/shared_infrastructure/config/config.json"

def load_service_timeout_from_config():
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return float(config.get('service_timeout', 10.0))  # default to 10 seconds
    except Exception as e:
        print(f"[Utils] Failed to load config: {e}")
        return 10.0

def load_stopping_distance_from_hazard_from_config():
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return float(config.get('stopping_distance_y_from_hazard', 0.5))  # default to 0.5 m distance in x & y
    except Exception as e:
        print(f"[Utils] Failed to load config: {e}")
        return 0.5

def load_navigation_override_timeout_from_config():
    try:
        with open('/root/ros2_ws/src/shared_infrastructure/config/config.json', 'r') as f:
            config = json.load(f)
        return float(config.get('navigation_override_timeout_in_minutes', 1.0))  # default to 1 minute
    except Exception as e:
        print(f"[Utils] Failed to load config: {e}")
        return 10.0

def load_agilex_camera_offset_from_config():
    try:
        with open('/root/ros2_ws/src/shared_infrastructure/config/config.json', 'r') as f:
            config = json.load(f)
        return float(config.get('agilex_camera_offset', 0.5))  # default to 0.5
    except Exception as e:
        print(f"[Utils] Failed to load camera offset config: {e}")
        return 0.5


def parse_hazard_type(value: str) -> HazardType:
    """Convert a string into the corresponding HazardType enum. Raises ValueError if invalid."""
    
    value = value.strip().upper().replace(" ", "_")

    try:
        return HazardType[value]
    except KeyError:
        raise ValueError(f"Invalid hazard type: '{value}'. Must be one of: {', '.join([e.name for e in HazardType])}")


def parse_task_type(value: str) -> TaskType:
    """Convert a string into the corresponding TaskType enum. Raises ValueError if invalid."""
    
    value = value.strip().upper().replace(" ", "_")

    try:
        return TaskType[value]
    except KeyError:
        raise ValueError(f"Invalid Task type: '{value}'. Must be one of: {', '.join([e.name for e in TaskType])}")

def parse_ore_spawn_type(value: str) -> OreSpawnType:
    """Convert a string into the corresponding OreSwpawnType enum. Raises ValueError if invalid."""
    
    value = value.strip().upper().replace(" ", "_")

    try:
        return OreSpawnType[value]
    except KeyError:
        raise ValueError(f"Invalid Ore Spawn type: '{value}'. Must be one of: {', '.join([e.name for e in OreSpawnType])}")


def hazard_type_to_str(hazard: HazardType) -> str:
    """Convert a HazardType enum into a readable string (e.g., 'Dust Plume')."""
    return hazard.name.replace("_", " ").title()

def task_type_to_str(task: TaskType) -> str:
    """Convert a TaskType enum into a readable string (e.g., 'Initial Inspection')."""
    return task.name.replace("_", " ").title()

def ore_spawn_type_to_str(spawnType: OreSpawnType) -> str:
    """Convert a OreSpawnType enum into a readable string (e.g., 'Normal')."""
    return spawnType.name.replace("_", " ").title()


def load_max_history_count():
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return int(config.get('max_ui_history', 5))
    except Exception as e:
        print(f"[Utils] Failed to load max_ui_history: {e}")
        return 5
    
def start_rosbag(topics, outdir, qos_file):
    os.makedirs(os.path.dirname(outdir), exist_ok=True)   # parent OK
    cmd = ["ros2", "bag", "record",
            "-o", outdir,
            "--qos-profile-overrides-path", qos_file,
            ] + topics
    
    # keep stderr so we can read the error text if it dies
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,      #  ← capture!
        text=True                    #  ← return strings not bytes
    )

    # give the child a moment: rosbag writes “Recording to ...” once it’s OK
    time.sleep(1.0)

    if proc.poll() is not None:                      # it already exited
        err = proc.stderr.read().strip()
        raise RuntimeError(f"ros2 bag failed: {err}")

    return proc

def stop_rosbag(bag_proc):
    if bag_proc.poll() is not None:         # died early
        err = bag_proc.stderr.read().strip()
        print(f"rosbag crashed: {err}")
    else:
        bag_proc.send_signal(signal.SIGINT)
        bag_proc.wait()
        print("Bag closed – loop done.")

def exit_gracefully(node):
    try:
        rclpy.spin(node)               # blocks until SIGINT
    except KeyboardInterrupt:
        node.get_logger().info("SIGINT received → exiting gracefully")
    finally:
        node.destroy_node()
        if ok():                      # or: if get_default_context().is_valid:
            rclpy.shutdown()

        sys.exit(0)     

def get_agilex_base_pose():
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return config.get('agilex_base_pose')
    except Exception as e:
        print(f"[Utils] Failed to get agilex_base_pose: {e}")
        return []
                
def get_current_hazard_type_and_pose():
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        hazard_str = config.get("current_hazard_target", "Fibrous Material").strip()
        hazard_enum = parse_hazard_type(hazard_str)
        return { 
                    "Hazard_Type": hazard_enum, 
                    "Hazard_Poses": HAZARD_POSES.get(hazard_enum, HAZARD_POSES[HazardType.FIBROUS_HAZARD])
               }
    except Exception as e:
        print(f"[Utils] Failed to load hazard pose from config: {e}")
        return { 
                    "Hazard_Type": HazardType.FIBROUS_HAZARD, 
                    "Hazard_Poses": HAZARD_POSES[HazardType.FIBROUS_HAZARD]
               }
   
    
def get_current_hazard_type_from_config() -> HazardType:
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        hazard_type_str = config.get("current_hazard_target", "Fibrous Material").strip()
        return parse_hazard_type(hazard_type_str)
    except Exception as e:
        print(f"[C++] ❌ Failed to load config: {e}")
    return HazardType.FIBROUS_HAZARD  # fallback

# def get_current_task_type_from_config() -> TaskType:
#     try:
#         with open(CONFIG_PATH, 'r') as f:
#             config = json.load(f)
#         task_type_str = config.get("current_task_type", "Initial Inspection").strip()
#         return parse_task_type(task_type_str)
#     except Exception as e:
#         print(f" ❌ Failed to load config: {e}")
#     return TaskType.INITIAL_INSPECTION  # fallback


def _get_stand_off_radius(hazard_type: HazardType) -> float:
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
    if hazard_type == HazardType.FIBROUS_HAZARD:
        r_h = config["fibrous_hazard_safety_radius"]
        m   = config["fibrous_hazard_safety_margin"]
    elif hazard_type == HazardType.DUST_PLUME:
        r_h = config["dust_plume_safety_radius"]
        m   = config["dust_plume_safety_margin"]
    else:
        raise ValueError(f"Unknown hazard_type: {hazard_type}")
    return r_h + m


def compute_edge(x_r: float, y_r: float,
                 x_h: float, y_h: float) -> Edge:
    """
    Decide which edge ('north', 'east', 'south', or 'west') of the
    standoff circle is quickest to reach *and* compatible with a
    clockwise orbit (camera on the right-hand side).

    Parameters
    ----------
    x_r, y_r : current robot position  (map frame, metres)
    x_h, y_h : hazard centre position (map frame, metres)

    Returns
    -------
    edge : str  -- one of 'north' | 'east' | 'south' | 'west'
    """
    dx = x_r - x_h          # robot offset in X (west = –ve)
    dy = y_r - y_h          # robot offset in Y (south = –ve)

    # Pick the axis with the *larger* absolute separation first.
    # That gives the shortest straight-line drive to the circle.
    if abs(dx) > abs(dy):
        return "west" if dx < 0 else "east"
    if abs(dy) > abs(dx):
        return "south" if dy < 0 else "north"

    # If they're equal (robot sits on a 45 ° line) choose the edge that
    # already puts the hazard on our *left* when heading straight in.
    # -  dx < 0  → robot is west  → drive east (edge = west)
    # -  dx ≥ 0  → robot is east  → drive west (edge = east)
    return "west" if dx < 0 else "east"

def find_orbit_safety_radius(hazard_type, hazard_diameter_m):
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
    fov_deg   = config["agilex_camera_fov_deg"]
    robot_l_m = config["agile_length_m"]
    robot_w_m = config["agile_width_m"]
    margin_m  = config["additional_safety_margin"]

    fov_rad = math.radians(fov_deg)
    # hazard_str = hazard_type_to_str(hazard_type)
    stand_off_radius = _get_stand_off_radius(hazard_type)
    r_camera = (hazard_diameter_m / 2) / math.tan(fov_rad / 2)
    r_robot  = math.hypot(robot_l_m/2, robot_w_m/2) + margin_m
    return max(stand_off_radius, r_camera, r_robot)

def get_current_ore_spawn_mode_from_config() -> OreSpawnType:
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        ore_spawn_mode = config.get("current_ore_spawn_mode", "Normal").strip()
        return parse_ore_spawn_type(ore_spawn_mode)
    except Exception as e:
        print(f" ❌ Failed to load config: {e}")
    return "normal"  # fallback

def is_orbit_clockwise():
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
    # Get key, default to "True" (string), and convert to bool
    return config.get("is_orbit_clockwise", "True") == "True"

def get_pose(x, y, z, qz, qw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = qz
    pose.orientation.w = qw

    return pose

def get_waypoints_from_config(hazard: HazardType) -> List[Tuple[float, float]]:
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        for entry in config.get("hazard_targets", []):
            if entry.get("hazard_type", "").strip().lower() == hazard.value.lower():
                return [tuple(wp) for wp in entry.get("waypoints", []) if len(wp) == 2]
    except Exception as e:
        print(f"[C++] ❌ Error parsing waypoints: {e}")
    return []
