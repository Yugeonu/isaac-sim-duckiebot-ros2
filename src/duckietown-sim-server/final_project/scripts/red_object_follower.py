#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedObjectFollower(Node):
    def __init__(self):
        super().__init__('red_object_follower')
        
        # 1. 구독 (눈)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image/raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # 2. 발행 (발)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------------------------------------------------
        # [설정] 멈춤 거리 조절 (값이 클수록 멀리서 멈춤)
        # ---------------------------------------------------------
        self.stop_width_threshold = 150  # 물체 너비(픽셀)가 이보다 크면 정지
        
        self.get_logger().info('🦆 오리 로봇: 자율주행(거리 유지) 모드 시작!')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        # HSV 변환 및 마스크 생성
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 빨간색 범위 (두 구간 합치기)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        # 윤곽선 검출
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        twist = Twist()
        height, width, _ = cv_image.shape
        center_x = width // 2

        if len(contours) > 0:
            # 가장 큰 물체 찾기
            c = max(contours, key=cv2.contourArea)
            
            # 노이즈 제거 (면적 기준)
            if cv2.contourArea(c) > 10:
                # [필수 요구사항] Bounding Box 생성 및 그리기
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 255), 2)

                # 물체 중심점 계산
                cx = x + (w // 2)
                
                # ------------------------------------------------
                # [로직] 거리 제어 (가까우면 멈춤)
                # ------------------------------------------------
                if w > self.stop_width_threshold:
                    # 너무 가까움 -> 정지
                    twist.linear.x = 0.0
                    status = "STOP (Too Close)"
                    cv2.putText(cv_image, status, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    # 거리가 적당함 -> 전진
                    twist.linear.x = 0.2
                    status = "GO"
                    cv2.putText(cv_image, status, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # [로직] 회전 제어 (화면 중앙 맞추기)
                error_x = center_x - cx
                twist.angular.z = 0.005 * error_x
                
        else:
            # 빨간색 없으면 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)
        
        # 화면 출력
        cv2.imshow("Robot View", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()