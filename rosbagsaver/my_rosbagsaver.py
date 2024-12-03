import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
from datetime import datetime
import zenoh
import time
import json

class ProcessListenerNode(Node):
    def __init__(self):
        super().__init__('process_listener')
        # Track the subprocess for ros2 bag recording
        self.rosbag_process = None
        self.bag_name = "my_bag"  # Define the bag name
        config = zenoh.Config()
        config.insert_json5("mode", json.dumps("client"))
        router_url = "tcp/160.85.253.140:30447"
        config.insert_json5("connect/endpoints", json.dumps([router_url]))
        print("Opening Zenoh session...")
        zenoh_session = zenoh.open(config)
        zenoh.init_log_from_env_or("error")
    
        key = "process_trigger"
        print(f"Declaring Listener on '{key}'...")
        pub = zenoh_session.declare_subscriber(key, self.listener)
       

        
        print("Press CTRL-C to quit...")
        while True:
            time.sleep(1)

    def listener(self, sample: zenoh.Sample):
        print(
                f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.to_string()}')"
            )
        self.get_logger().info(f"Received message: {sample.payload.to_string()}")
        if sample.payload.to_string() == "start":
            self.start_rosbag_recording()
        elif sample.payload.to_string() == "stop":
            self.stop_rosbag_recording()
        else:
            self.get_logger().warn(f"Unknown command: {sample.payload.to_string()}")

    def start_rosbag_recording(self):
        """Start rosbag recording in MCAP format if not already running."""
        if self.rosbag_process is not None:
            self.get_logger().warn("Rosbag recording is already running.")
            return

    #    bag_name = f"rosbag_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mcap"
        bag_name = f"{self.bag_name}"
      #  bag_path = os.path.join(os.getcwd(), bag_name)
        bag_path = '/pod-data/my_rosbag'
        self.get_logger().info(f"Starting ros2 bag record to save as {bag_name}")

        try:
            self.rosbag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', '/pod-data/rosbag', '--storage', 'mcap', '-a'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.get_logger().info(f"Recording started. Bag file: {bag_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {str(e)}")
            self.rosbag_process = None

    def stop_rosbag_recording(self):
        """Stop rosbag recording if it's running."""
        if self.rosbag_process is None:
            self.get_logger().warn("No rosbag recording is currently running.")
            return

        self.get_logger().info("Stopping ros2 bag recording...")
        try:
            self.rosbag_process.terminate()  # Send SIGTERM to the process
            self.rosbag_process.wait()  # Wait for process to terminate
            self.get_logger().info("Recording stopped.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop recording: {str(e)}")
        finally:
            self.rosbag_process = None


def main(args=None):
    print('my rosbagsaver')
    rclpy.init(args=args)
    node = ProcessListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rosbag_recording()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

