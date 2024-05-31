#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat
import cv2
import numpy as np
import time
from . import utils
from cv_bridge import CvBridge


ESC_KEY = 27
MIN_DEPTH = 500  # 0.5m
MAX_DEPTH = 80000  # 8.0m

class AstraPlusNode(Node):
    def __init__(self):
        super().__init__('astra_plus_node')
        self.ir_publisher_ = self.create_publisher(Image, 'ir_image', 10)
        self.depth_publisher_ = self.create_publisher(Image, 'depth_image', 10)
        self.color_publisher_ = self.create_publisher(Image, 'color_image', 10)
        self.bridge = CvBridge()
        self.pipeline = Pipeline()
        self.config = Config()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30Hz
        
        cameraparam = self.pipeline.get_camera_param()
        print(cameraparam)

        try:
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.IR_SENSOR)
            depth_profile = profile_list.get_default_video_stream_profile()
            self.config.enable_stream(depth_profile)
            self.get_logger().info(f"depth profile: {depth_profile}")

            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            ir_profile = profile_list.get_default_video_stream_profile()
            self.config.enable_stream(ir_profile)
            self.get_logger().info(f"ir profile: {ir_profile}")

            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = profile_list.get_default_video_stream_profile()
            self.config.enable_stream(color_profile)
            self.get_logger().info(f"color profile: {color_profile}")

            self.get_logger().warn(f"AstraPlusNode Initiallized successfully...")

        except Exception as e:
            self.get_logger().error(f"Failed to set up depth profile: {e}")
            return

        self.pipeline.start(self.config)

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                return
            
            # Get IR Image and convert to binary
            ir_frame = frames.get_ir_frame()
            ir_data = np.asanyarray(ir_frame.get_data())
            width = ir_frame.get_width()
            height = ir_frame.get_height()
            ir_format = ir_frame.get_format()
            if ir_format == OBFormat.Y8:
                ir_data = np.resize(ir_data, (height, width, 1))
                data_type = np.uint8
                image_dtype = cv2.CV_8UC1
                max_data = 255
            elif ir_format == OBFormat.MJPG:
                ir_data = cv2.imdecode(ir_data, cv2.IMREAD_UNCHANGED)
                data_type = np.uint8
                image_dtype = cv2.CV_8UC1
                max_data = 255
                if ir_data is None:
                    print("decode mjpeg failed")
                ir_data = np.resize(ir_data, (height, width, 1))
            else:
                ir_data = np.frombuffer(ir_data, dtype=np.uint16)
                data_type = np.uint16
                image_dtype = cv2.CV_16UC1
                max_data = 65535
                ir_data = np.resize(ir_data, (height, width, 1))
            cv2.normalize(ir_data, ir_data, 0, max_data, cv2.NORM_MINMAX, dtype=image_dtype)
            ir_data = ir_data.astype(data_type)
            ir_image_msg = self.bridge.cv2_to_imgmsg(ir_data, encoding="mono16")
            #cv2.imshow("Infrared Viewer", ir_image)
            #key = cv2.waitKey(1)

            # Get depth Image and convert to binary
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                return
            width_depth = depth_frame.get_width()
            height_depth = depth_frame.get_height()
            scale_depth = depth_frame.get_depth_scale()
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((height_depth, width_depth))
            depth_data = depth_data.astype(np.float32) * scale_depth
            depth_data = depth_data.astype(np.uint16)
            depth_data = np.where((depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0)
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_data, encoding="mono16")

            # Get color Image and convert to binary
            color_frame = frames.get_color_frame()
            if color_frame is None:
                return
            color_image = utils.frame_to_bgr_image(color_frame)
            color_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

            # Synchronize time stamp
            time_stamp = self.get_clock().now().to_msg()
            ir_image_msg.header.stamp = time_stamp
            depth_image_msg.header.stamp = time_stamp
            color_image_msg.header.stamp = time_stamp
            self.ir_publisher_.publish(ir_image_msg)
            self.depth_publisher_.publish(depth_image_msg)
            self.color_publisher_.publish(color_image_msg)

        except Exception as e:
            self.get_logger().error(f"Error in processing frames: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AstraPlusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()