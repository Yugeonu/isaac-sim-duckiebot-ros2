#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from final_project.srv import SetColor 

class LEDServiceNode(Node):
    def __init__(self):
        super().__init__('led_service_node')
        # 아이작 심으로 쏘는 토픽
        self.publisher = self.create_publisher(Vector3, '/duckie/led_cmd', 10)
        # 사용자에게 받는 서비스 (요구사항: /duckie_led_control)
        self.srv = self.create_service(SetColor, '/duckie_led_control', self.handle_led_request)
        self.get_logger().info('LED Service Ready. Waiting for "color: \'red\'"...')

    def handle_led_request(self, request, response):
        req_color = request.color.lower()
        msg = Vector3()
        
        # 색상 변환 로직
        if req_color == 'red':
            msg.x, msg.y, msg.z = 100.0, 0.0, 0.0
        elif req_color == 'green':
            msg.x, msg.y, msg.z = 0.0, 100.0, 0.0
        elif req_color == 'blue':
            msg.x, msg.y, msg.z = 0.0, 0.0, 100.0
        elif req_color == 'white':
            msg.x, msg.y, msg.z = 100.0, 100.0, 100.0
        elif req_color == 'off':
            msg.x, msg.y, msg.z = 0.0, 0.0, 0.0
        else:
            self.get_logger().warn(f"Unknown color: {req_color}")
            response.success = False
            return response

        self.publisher.publish(msg)
        self.get_logger().info(f"Command Sent: {req_color}")
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LEDServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()