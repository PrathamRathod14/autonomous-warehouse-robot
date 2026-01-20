#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class StationRecorder(Node):
    """
    Click points in RViz using 'Publish Point' tool (topic: /clicked_point).
    This prints YAML lines you can copy into warehouse.yaml stations.
    Yaw is not captured by clicked point; set yaw manually (0, 1.57, 3.14, -1.57).
    """

    def __init__(self):
        super().__init__("station_recorder")
        self.declare_parameter("name_prefix", "station")
        self.counter = 0
        self.prefix = str(self.get_parameter("name_prefix").value)

        self.sub = self.create_subscription(PointStamped, "/clicked_point", self.on_click, 10)
        self.get_logger().info("StationRecorder ready. Use RViz 'Publish Point' and click on map.")

    def on_click(self, msg: PointStamped):
        self.counter += 1
        name = f"{self.prefix}_{self.counter}"
        x = msg.point.x
        y = msg.point.y
        # yaw must be set manually
        yaw = 0.0
        print(f'  {name}: [{x:.3f}, {y:.3f}, {yaw:.2f}]')

def main():
    rclpy.init()
    node = StationRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
