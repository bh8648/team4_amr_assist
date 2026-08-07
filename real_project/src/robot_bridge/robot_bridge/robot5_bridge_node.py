#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

ROBOT_ID = 'robot5'


class Robot5BridgeNode(Node):
    def __init__(self):
        super().__init__('robot5_bridge_node')
        self.get_logger().info(f'{ROBOT_ID} 브릿지 노드 시작')


def main(args=None):
    rclpy.init(args=args)
    node = Robot5BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
