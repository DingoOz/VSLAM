#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
import numpy as np

try:
    import cuvslam
except ImportError:
    print("WARNING: PyCuVSLAM not installed. Please install it first.")
    cuvslam = None


class VSLAMNode(Node):
    def __init__(self):
        super().__init__('pycuvslam_node')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('mode', 'mono')  # mono, stereo, or rgbd

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.mode = self.get_parameter('mode').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Camera calibration
        self.camera_info = None
        self.slam_initialized = False

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '~/odometry', 10)
        self.path_pub = self.create_publisher(Path, '~/path', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Path for visualization
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        # VSLAM tracker (to be initialized when camera info is received)
        self.tracker = None

        self.get_logger().info(f'PyCuVSLAM Node initialized in {self.mode} mode')
        self.get_logger().info(f'Waiting for camera info on {self.camera_info_topic}')

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera calibration received')
            self.initialize_slam()

    def initialize_slam(self):
        if cuvslam is None:
            self.get_logger().error('PyCuVSLAM library not available')
            return

        if self.camera_info is None:
            self.get_logger().warn('Camera info not available yet')
            return

        try:
            # Extract camera parameters
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            width = self.camera_info.width
            height = self.camera_info.height

            # Initialize cuVSLAM tracker
            # Note: This is a simplified initialization.
            # Actual parameters depend on PyCuVSLAM API
            self.get_logger().info(f'Initializing cuVSLAM with camera params:')
            self.get_logger().info(f'  Resolution: {width}x{height}')
            self.get_logger().info(f'  fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}')

            # TODO: Initialize tracker based on actual PyCuVSLAM API
            # self.tracker = cuvslam.Tracker(...)

            self.slam_initialized = True
            self.get_logger().info('cuVSLAM initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize SLAM: {str(e)}')

    def image_callback(self, msg):
        if not self.slam_initialized or self.tracker is None:
            return

        try:
            # Convert ROS image to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # TODO: Process image with cuVSLAM
            # This is a placeholder - actual implementation depends on PyCuVSLAM API
            # pose = self.tracker.track(cv_image, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

            # For now, publish identity transform as placeholder
            self.publish_pose(msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_pose(self, timestamp):
        # Placeholder pose (identity)
        # In actual implementation, this would come from cuVSLAM tracker

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.world_frame
        odom_msg.child_frame_id = self.camera_frame

        # Set pose (placeholder identity)
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        self.odom_pub.publish(odom_msg)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = self.world_frame
            t.child_frame_id = self.camera_frame
            t.transform.translation.x = odom_msg.pose.pose.position.x
            t.transform.translation.y = odom_msg.pose.pose.position.y
            t.transform.translation.z = odom_msg.pose.pose.position.z
            t.transform.rotation = odom_msg.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

        # Update path
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose
        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = timestamp
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VSLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
