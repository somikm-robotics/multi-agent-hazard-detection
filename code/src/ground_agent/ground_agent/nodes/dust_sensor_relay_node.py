#!/usr/bin/env python3
"""
dust_relay_single_hit.py
Publishes the first non-zero DustSensorResult on /dust-sensor-reading,
then stays quiet.
"""

import rclpy
from rclpy.node import Node
from shared_interfaces.msg import DustSensorResult
from shared_infrastructure.utils import exit_gracefully


class DustSensorRelayNode(Node):
    def __init__(self):
        super().__init__('dust_sensor_relay_node')

        self.hit_sent = False   # flips after first publish

        # outgoing publisher
        self.pub_ = self.create_publisher(
            DustSensorResult,
            '/dust_sensor_reading',
            10)

        # incoming subscriber
        self.create_subscription(
            DustSensorResult,
            '/dust/agilex_diff_drive',
            self._cb,
            10)

        self.get_logger().info('Waiting for first non-zero dust reading…')

    def _cb(self, msg: DustSensorResult):
        if self.hit_sent:
            # already forwarded one packet – just optional minimal log
            if msg.pm10 == 0.0:
                pass
                # self.get_logger().debug('Zero again')
            return

        # still in “waiting” mode
        pm_values = [msg.pm1_0, msg.pm2_5, msg.pm10]

        # Count how many PM values are above threshold
        threshold = 0.1
        available = sum(1 for v in pm_values if v is not None and float(v) > threshold)


        if available >= 2:
            self.pub_.publish(msg)          # forward once
            self.hit_sent = True
            self.get_logger().info('Dust detected – published reading')
            self.get_logger().info(
                f"💨 Dust reading published: "
                f"PM1.0={msg.pm1_0:.1f} µg/m³, "
                f"PM2.5={msg.pm2_5:.1f} µg/m³, "
                f"PM10={msg.pm10:.1f} µg/m³, "
                f"TSP={msg.tsp:.1f} µg/m³, "
                f"Opacity={msg.opacity:.2f}, "
                f"Wind={msg.wind_speed:.1f} m/s @ {msg.wind_direction:.0f}°, "
                f"RH={msg.humidity:.0f} %, "
                f"Temp={msg.temperature:.1f} °C"
            )


        else:
            pass
            # self.get_logger().info('Still zero…')

def main(args=None):
    rclpy.init(args=args)
    node = DustSensorRelayNode()
    node.get_logger().info("✅ dust_sensor_relay_node main() started")
    exit_gracefully(node)

if __name__ == "__main__":
    main()
