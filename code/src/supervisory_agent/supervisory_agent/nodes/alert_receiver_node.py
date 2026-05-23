# Subscribes to /hazard_notification messages
# supervisory_agent/nodes/alert_receiver_node.py
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tkinter import messagebox
from shared_interfaces.msg import ToxicityResult, HazardPose, MissionStatus, BaseReturn, DustSensorResult, DensityEstimationResult
from shared_infrastructure.hazard_types import HazardType
from shared_infrastructure.utils import parse_hazard_type, parse_task_type
from shared_interfaces.msg import HazardPose, NavigationResult, BaseReturn, OnArrivalTask, MissionStatus
from rclpy.action import ActionClient
import math
from shared_infrastructure.task_types import TaskType




class AlertReceiverNode(Node):
    def __init__(self, ui_root, ui_node):
        super().__init__('alert_receiver_node')
        self.get_logger().info("Alert Receiver Node initialized")

        self.last_mission_hazard = None

        self.ui_root = ui_root  
        self.status_var = ui_node.status_var
        self.status_label = ui_node.status_label
        # self.ui_node = ui_node  # store a reference

        self.append_alert_history = ui_node.append_alert_history
        self.approve_button = ui_node.approve_button
        self.cancel_button = ui_node.cancel_button
        self.continue_button = ui_node.continue_button
        self.orbit_cancel_button = ui_node.orbit_cancel_button


        self.subscription = self.create_subscription(HazardPose, 'hazard_detected', self.hazard_callback, 10)
        # self.create_subscription(MissionStatus, 'mission_status', self.mission_status_callback, 10)
        self.create_subscription(BaseReturn, 'return_to_base_status', self.return_to_base_callback, 10)
        self.create_subscription(ToxicityResult, 'toxic_gas_reading', self.toxicity_callback, 10)
        self.create_subscription(DustSensorResult, '/dust_sensor_reading', self.dust_sensor_callback, 10)
        self.create_subscription(DensityEstimationResult, 'estimation_complete', self.estimation_complete_callback, 10)
        self.create_subscription(String, 'unsafe_returning_to_base', self.return_to_base_unsafe_callback, 10)
        self.create_subscription(MissionStatus, 'intermediary_mission_status', self.intermediary_mission_status_callback, 10)
        self.create_subscription(String, 'scan_now_response', self.return_to_base_unsafe_callback, 10)
        
        

    def hazard_callback(self, msg):
        text = msg.hazard_type
        self.get_logger().info(f"📥 Supervisory Agent received: {text}")
        # self.ui_root.after(0, lambda: self.status_var.set())
        # self.status_label.config(fg='red', bg='white', font=('Arial', 14, 'bold'))
        status_text = f"⚠️ {text} detected"
        self.update_status(status_text, "alert")
        self.ui_root.after(0, lambda: self.append_alert_history(status_text))
        

        hazard_type = parse_hazard_type(msg.hazard_type)
        if hazard_type in [HazardType.FIBROUS_HAZARD, HazardType.DUST_PLUME]:
            self.ui_root.after(0, lambda: self.approve_button.config(state='normal'))
            self.ui_root.after(0, lambda: self.cancel_button.config(state='normal'))
        else:
            self.ui_root.after(0, lambda: self.approve_button.config(state='disabled'))
            self.ui_root.after(0, lambda: self.cancel_button.config(state='disabled'))

    def mission_status_callback(self, msg):

        text = msg.message
        self.ui_root.after(0, lambda: self.append_alert_history(text))
        self.get_logger().info(f"📦 Mission Update: {text}")

        hazard_type = parse_hazard_type(msg.hazard_type)
        if hazard_type == HazardType.FIBROUS_HAZARD:
            task_type = parse_task_type(msg.task_type)
        else:
            task_type = TaskType.NONE
        
        if task_type == TaskType.INITIAL_INSPECTION:
            self.ui_root.after(0, lambda: self.continue_button.config(state='normal'))
            self.ui_root.after(0, lambda: self.orbit_cancel_button.config(state='normal'))
        else:
            self.ui_root.after(0, lambda: self.continue_button.config(state='disabled'))
            self.ui_root.after(0, lambda: self.orbit_cancel_button.config(state='disabled'))

    def intermediary_mission_status_callback(self, msg):
        text = msg.message
        self.ui_root.after(0, lambda: self.append_alert_history(text))
        self.get_logger().info(f"📦 Mission Update: {text}")

        if msg.toggle_command:
            self.toggle_approval_buttons(msg)
       

    def toggle_approval_buttons(self, msg):
        hazard_type = parse_hazard_type(msg.hazard_type)
        if hazard_type == HazardType.FIBROUS_HAZARD:
            task_type = parse_task_type(msg.task_type)
        else:
            task_type = TaskType.NONE

        if task_type == TaskType.INITIAL_INSPECTION:
            curr_state = self.approve_button['state']
            toggled_state = 'normal' if curr_state == 'disabled' else 'disabled'
            self.ui_root.after(0, lambda: self.approve_button.config(state=toggled_state))
            self.ui_root.after(0, lambda: self.cancel_button.config(state=toggled_state))
        elif task_type == TaskType.ASBESTOS_ANALYSIS:
            curr_state = self.continue_button['state']
            toggled_state = 'normal' if curr_state == 'disabled' else 'disabled'
            self.ui_root.after(0, lambda: self.continue_button.config(state=toggled_state))
            self.ui_root.after(0, lambda: self.orbit_cancel_button.config(state=toggled_state))
        else:
            self.ui_root.after(0, lambda: self.approve_button.config(state='disabled'))
            self.ui_root.after(0, lambda: self.cancel_button.config(state='disabled'))


        # if task_type == TaskType.INITIAL_INSPECTION:
        #     self.ui_root.after(0, lambda: self.approve_button.config(state='disabled'))
        #     self.ui_root.after(0, lambda: self.cancel_button.config(state='disabled'))
        # elif task_type == TaskType.ASBESTOS_ANALYSIS:
        #     self.ui_root.after(0, lambda: self.continue_button.config(state='disabled'))
        #     self.ui_root.after(0, lambda: self.orbit_cancel_button.config(state='disabled'))
        # else:
        #     self.ui_root.after(0, lambda: self.approve_button.config(state='disabled'))
        #     self.ui_root.after(0, lambda: self.cancel_button.config(state='disabled'))
        



    def return_to_base_unsafe_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"☣️ Return to Base Update: {text}")
        status_text = f"☣️ {text}"
        self.update_status(status_text, "alert")
        # self.ui_root.after(0, lambda: self.status_var.set())
        # self.status_label.config(fg='red', bg='white', font=('Arial', 14, 'bold'))
        self.ui_root.after(0, lambda: self.append_alert_history(text))
        
    def scan_now_response_callbase(self, msg):
        text = msg.data
        message = f"Scan Now response: {text}"
        self.ui_root.after(0, lambda: self.append_alert_history(message))

    def return_to_base_callback(self, msg):
        text = msg.message
        self.get_logger().info(f"🏁 Return to Base Update: {text}")
        status_text = f"🏁 {text}"
        # self.ui_root.after(0, lambda: self.status_var.set(f"🏁 {text}"))
        # self.status_label.config(fg='green', bg='white', font=('Arial', 14, 'bold'))
        self.update_status(status_text, "success")
        self.ui_root.after(0, lambda: self.append_alert_history(text))

    def toxicity_callback(self, msg: ToxicityResult):
        """
        Handle a ToxicityResult message coming from /toxic_gas_reading.
        • status_var shows the current gas‑hazard status only
        • full formatted string is appended to the alert history panel
        """
        # Build a concise but complete line for the history pane
        summary = (
            "☣️  Toxic Gas | "
            f"Status: {msg.status} | "
            f"CO {msg.co_ppm:.1f} ppm, "
            f"NH₃ {msg.nh3_ppm:.1f} ppm, "
            f"NO₂ {msg.no2_ppm:.1f} ppm, "
            f"VOC {msg.voc_ppb:.1f} ppb"
        )

        # Console log for good measure
        self.get_logger().info(summary)

        # Thread‑safe GUI updates
        status_text = f"☣️ Gas Toxicity:  {msg.status}"
        self.update_status(status_text, "alert")
        # self.ui_root.after(0, lambda: self.status_var.set(f"☣️ Gas Toxicity:  {msg.status}"))
        # self.status_label.config(fg='red', bg='white', font=('Arial', 14, 'bold'))

        self.ui_root.after(0, lambda: self.append_alert_history(summary))

    def dust_sensor_callback(self, msg):
        summary = (
            "💨 Dust Plume | "
            f"PM1.0 {msg.pm1_0:.1f} µg/m³, "
            f"PM2.5 {msg.pm2_5:.1f} µg/m³, "
            f"PM10 {msg.pm10:.1f} µg/m³, "
            f"TSP {msg.tsp:.1f} µg/m³, "
            f"Opacity {msg.opacity:.2f}, "
            f"Wind {msg.wind_speed:.1f} m/s @ {msg.wind_direction:.0f}°, "
            f"RH {msg.humidity:.0f} %, "
            f"Temp {msg.temperature:.1f} °C"
        )
        
        # Console log for good measure
        self.get_logger().info(summary)
        self.ui_root.after(0, lambda: self.append_alert_history(summary))

    def estimation_complete_callback(self, msg):
        summary = (
            "💨 Dust Plume Density Estimation Results | "
            f"Density {msg.density:.1f} µg/m³, "
            f"Severity Level {msg.level} !³, "
        )
    
        if any(keyword in msg.level for keyword in ["Severe", "Very High", "High"]):
            status_text = f"💨 Dust Plume Density Level:  {msg.level}"
            self.update_status(status_text, "alert")
            # self.ui_root.after(0, lambda: self.status_var.set(f"💨 Dust Plume Density Level:  {msg.level}"))
            # self.status_label.config(fg='red', bg='white', font=('Arial', 14, 'bold'))
        else:
            status_text = f"💨 Dust Plume Density Level:  {msg.level}"
            self.update_status(status_text)
            # self.ui_root.after(0, lambda: self.status_var.set(f"💨 Dust Plume Density Level:  {msg.level}"))
            # self.status_label.config(fg='blue', bg='white', font=('Arial', 14, 'bold'))

        self.ui_root.after(0, lambda: self.append_alert_history(summary))

    def update_status(self, text: str, level: str = "neutral"):
    
        def _apply_update():
            self.status_var.set(text)

            if level == "success":
                self.status_label.config(fg="green", bg="white", font=("Arial", 14, "bold"))
            elif level == "alert":
                self.status_label.config(fg="red", bg="white", font=("Arial", 14, "bold"))
            else:  # neutral/info
                self.status_label.config(fg="blue", bg="white", font=("Arial", 14, "bold"))

        self.ui_root.after(0, _apply_update)


# def main(args=None):
#     rclpy.init(args=args)
#     root = tk.Tk()
#     status_var = tk.StringVar()
#     status_var.set("🟢 No Hazard Detected")
#     node = AlertReceiverNode(root, status_var)
#     node.get_logger().info("✅ alert_receiver_node main() started")
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()