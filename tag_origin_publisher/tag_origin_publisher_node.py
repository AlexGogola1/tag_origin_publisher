import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector

class TagOriginPublisher(Node):
    def __init__(self):
        super().__init__('tag_origin_publisher')

        # Parameters
        self.tag_id = 0
        self.corner_idx = 0
        self.marker_size = 0.1  # meters
        self.publish_rate = 10.0  # Hz

        # Camera intrinsics (assumed/fake, update if known)
        self.fx = 600
        self.fy = 600
        self.cx = 640 / 2
        self.cy = 480 / 2
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                       [0, self.fy, self.cy],
                                       [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1))

        # AprilTag detector
        self.detector = Detector(families='tag36h11')

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # Publisher
        self.pub = self.create_publisher(PoseStamped, '/tag_origin', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('Tag Origin Publisher node has started.')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)
        for det in detections:
            if det.tag_id == self.tag_id:
                corner = det.corners[self.corner_idx]
                cx, cy = int(corner[0]), int(corner[1])
                depth = depth_frame.get_distance(cx, cy)
                if depth <= 0:
                    continue

                point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'
                pose_msg.pose.position.x = point[0]
                pose_msg.pose.position.y = point[1]
                pose_msg.pose.position.z = point[2]
                pose_msg.pose.orientation.w = 1.0  # No orientation
                self.pub.publish(pose_msg)
                self.get_logger().info(f"Published origin at ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
                break  # Only one tag needed

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagOriginPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
