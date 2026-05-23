import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import tkinter as tk
import json, os
from tkinter import ttk
from shared_infrastructure.utils import load_max_history_count
from shared_interfaces.msg import UIOverrideMission
from supervisory_agent.data_loader import get_location_names


class UIInterfaceNode(Node):
    def __init__(self, root, status_var):
        super().__init__('ui_interface_node')

        self.max_history_entries = load_max_history_count()

        # ROS2 publishers
        self.override_pub = self.create_publisher(UIOverrideMission, 'ui_override_command', 10)
        self.scan_pub = self.create_publisher(String, 'ui_scan_incident', 10)
        self.shutdown_pub = self.create_publisher(Bool, 'shutdown_system', 10)

        # Listen for shutdown system broadcast
        self.shutdown_sub = self.create_subscription(
            Bool,
            'shutdown_system',
            self.handle_shutdown,
            10
        )

        location_names = get_location_names()

        self.root = root

        # === Setup UI ===

        # Status bar (bottom)
        self.status_var = status_var
        self.status_label = tk.Label(
            self.root, 
            textvariable=self.status_var, 
            fg='blue',       
            bg='white',     # ⚪ white background
            anchor='w',
            font=('Arial', 14, 'bold') # bigger, bold font
        )
        self.status_label.pack(fill='x', side='bottom', padx=5, pady=5)

        # History UI
        self.history = []
        self.history_text = tk.Text(root, height=5, width=50, state='disabled')
        self.history_text.pack(padx=10, pady=5)

        # --- Inspection Approval Frame ---
        inspection_frame = tk.LabelFrame(self.root, text="Inspection", bg='#d3d3d3', font=('Arial', 11, 'bold'))
        inspection_frame.pack(pady=8, fill='x', padx=20)

        self.approve_button = tk.Button(
            inspection_frame, text="Approve", bg='green', fg='white', width=18,
            command=lambda: self.send_initial_inspection_approval('approve'),
            state='disabled'
        )
        self.approve_button.pack(side='left', padx=10, pady=5)

        self.cancel_button = tk.Button(
            inspection_frame, text="Cancel", bg='red', fg='white', width=18,
            command=lambda: self.send_initial_inspection_approval('cancel'),
            state='disabled'
        )
        self.cancel_button.pack(side='left', padx=10, pady=5)

        # --- Orbit Approval Frame ---
        orbit_frame = tk.LabelFrame(self.root, text="Asbestos Analysis", bg='#d3d3d3', font=('Arial', 11, 'bold'))
        orbit_frame.pack(pady=10, fill='x', padx=20)

        self.continue_button = tk.Button(
            orbit_frame, text="Approve", bg='green', fg='white', width=18,
            command=self.send_continue_orbit,
            state='disabled'
        )
        self.continue_button.pack(side='left', padx=10, pady=5)

        self.orbit_cancel_button = tk.Button(
            orbit_frame, text="Cancel", bg='red', fg='white', width=18,
            command=self.send_cancel_orbit,
            state='disabled'
        )
        self.orbit_cancel_button.pack(side='left', padx=10, pady=5)


        # --- SCAN INCIDENT Frame (Separate) ---
        scan_frame = tk.LabelFrame(self.root, text="Scan Incident", bg='#d3d3d3', font=('Arial', 11, 'bold'))
        scan_frame.pack(pady=12, fill='x', padx=20)

        self.enable_scan_var = tk.IntVar(value=0)
        enable_scan_cb = tk.Checkbutton(
            scan_frame,
            text="Enable Scan Incident",
            variable=self.enable_scan_var,
            command=self.toggle_scan_incident
        )
        enable_scan_cb.pack(anchor='w', padx=10, pady=2)

        # Add label above dropdown
        tk.Label(scan_frame, text="Select location:", bg='#d3d3d3').pack(anchor='w', padx=10, pady=2)

        self.location_var = tk.StringVar()
        self.location_dropdown = ttk.Combobox(
            scan_frame,
            textvariable=self.location_var,
            values=location_names,   # <-- loaded from data_loader
            state="disabled",
            width=30
        )
        self.location_dropdown.pack(anchor='w', padx=10, pady=2)

        self.scan_button = tk.Button(
            scan_frame,
            text="Scan Incident",
            command=self.send_scan_command,
            state="disabled"
        )
        self.scan_button.pack(anchor='w', padx=10, pady=6)


        exit_button = tk.Button(self.root, text="Exit System", bg='red', fg='white', command=self.send_shutdown)
        exit_button.pack(padx=20, pady=10)


    # --- TOGGLE ENABLE/DISABLE FOR DROPDOWN/BUTTON ---
    def toggle_scan_incident(self):
        enabled = self.enable_scan_var.get() == 1
        self.location_dropdown.config(state="readonly" if enabled else "disabled")
        self.scan_button.config(state="normal" if enabled else "disabled")


    def load_location_names(self):
        with open(self.json_path, 'r') as f:
            loc_data = json.load(f)
        return list(loc_data["Location"].keys())


    def send_initial_inspection_approval(self, command):
        self.publish_override_mission(command, True)


    def send_scan_command(self):
        location = self.location_var.get()
        msg = String()
        msg.data = location
        self.scan_pub.publish(msg)
        self.get_logger().info(f"🛰️ Sent scan incident request: {location}")

    def publish_override_mission(self, command, is_initial_inspection):
        msg = UIOverrideMission()
        msg.command = command
        msg.is_initial_inspection = is_initial_inspection
        self.override_pub.publish(msg)
        self.get_logger().info(f"🛰️ Sent override command: {command}")

    def append_alert_history(self, msg):
        latest_status = msg

        # # Update status bar
        # self.root.after(0, lambda: self.status_var.set(latest_status))

        # Update history list and limit to last 5
        self.history.append(latest_status)
        if len(self.history) > self.max_history_entries:
            self.history.pop(0)

        # Update scrollable text box
        self.root.after(0, self.update_history_view)

    def update_history_view(self):
        self.history_text.config(state='normal')
        self.history_text.delete(1.0, tk.END)
        for entry in self.history:
            self.history_text.insert(tk.END, entry + "\n")
        self.history_text.config(state='disabled')

    def send_continue_orbit(self):
        self.publish_override_mission("approve" , False)
        self.get_logger().info("✅ Sent Continue Orbit request.")

    def send_cancel_orbit(self):
        self.publish_override_mission("cancel" , False)
        self.get_logger().info("❌ Sent Cancel Orbit request.")


    def send_shutdown(self):
        msg = Bool()
        msg.data = True
        self.shutdown_pub.publish(msg)
        self.get_logger().info("🟥 Sent shutdown signal to system")

    def handle_shutdown(self, msg):
        self.get_logger().info("🔴 Shutdown signal received. Closing UI...")
        self.root.quit()           # Close the Tkinter window
        rclpy.shutdown()           # Shutdown the ROS node



#     def run(self):
#         self.root.mainloop()


# def main(args=None):
#     rclpy.init(args=args)
#     node = UIInterfaceNode()
#     node.run()
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
