#!/usr/bin/env python3

import rclpy
import tkinter as tk


from supervisory_agent.nodes. ui_interface_node import UIInterfaceNode
from supervisory_agent.nodes.alert_receiver_node import AlertReceiverNode
from shared_infrastructure.utils import exit_gracefully

class UICoordinator:
    def __init__(self):
        # Setup UI
        self.root = tk.Tk()
        self.root.title("Supervisory Dashboard")
        self.status_var = tk.StringVar()
        self.status_var.set("🟢 No Hazard Detected")

        # Create subnodes with shared context
        self.ui_node = UIInterfaceNode(self.root, self.status_var)
        # self.alert_node = AlertReceiverNode(self.root, self.status_var, self.ui_node.append_alert_history)
        self.alert_node = AlertReceiverNode(self.root,  self.ui_node)


        # Add subnodes to an executor
        # self.executor = rclpy.executors.MultiThreadedExecutor()
        # self.executor.add_node(self.ui_node)
        # self.executor.add_node(self.alert_node)

        # Start ROS spinning in another thread
        # threading.Thread(target=self.executor.spin, daemon=True).start()

        # Start a polling-based spin via Tkinter's event loop
        self.root.after(100, self.spin_once)
        self.root.mainloop()

        # Cleanup on exit
        exit_gracefully(self.ui_node)
        exit_gracefully(self.alert_node)
        # self.ui_node.destroy_node()
        # self.alert_node.destroy_node()
        
    def spin_once(self):
        rclpy.spin_once(self.ui_node, timeout_sec=0)
        rclpy.spin_once(self.alert_node, timeout_sec=0)
        self.root.after(10, self.spin_once)  # keep polling ROS events


def main(args=None):
    rclpy.init(args=args)
    UICoordinator()
    # rclpy.shutdown()
    

if __name__ == '__main__':
    main()
