import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import zenoh
import time
import json
import signal


class ProcessListenerNode(Node):
    def __init__(self, key, bag_name, router_url, regex_pattern, max_bag_size):
        super().__init__('process_listener')
        self.rosbag_process = None
        self.bag_name = bag_name
        self.regex_pattern =  regex_pattern
        self.max_bag_size = max_bag_size

        # Zenoh configuration
        config = zenoh.Config()
        config.insert_json5("mode", json.dumps("client"))
        config.insert_json5("connect/endpoints", json.dumps([router_url]))
        print("Opening Zenoh session...")
        zenoh_session = zenoh.open(config)
        zenoh.init_log_from_env_or("error")

        print(f"Declaring Listener on '{key}'...")
        zenoh_session.declare_subscriber(key, self.listener)

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

        
        # Dynamically set the bag path
        #bag_path = os.path.join('/pod-data/', self.bag_name)
       # self.get_logger().info(f"Starting ros2 bag record to save as {bag_path}")

        # Base path for the rosbag
        base_bag_path = os.path.join('/pod-data/', self.bag_name)
    
        # Ensure unique bag path
        bag_path = base_bag_path
        counter = 1
        while os.path.exists(bag_path):
            bag_path = f"{base_bag_path}_{counter}"
            counter += 1

        self.get_logger().info(f"Starting ros2 bag record to save as {bag_path}")

        try:
            self.rosbag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_path, '--storage', 'mcap', '-e', self.regex_pattern, '-b', str(self.max_bag_size)],
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
          #  self.rosbag_process.terminate()  # Send SIGTERM to the process
          #  self.rosbag_process.wait()  # Wait for process to terminate

            self.rosbag_process.send_signal(signal.SIGINT)  # Simulate Ctrl+C
            self.rosbag_process.wait(timeout=5)  # Give it time to shut down

            if self.rosbag_process.poll() is None:  # If still running
                self.get_logger().warn("Process did not exit with SIGINT, forcing termination...")
                self.rosbag_process.kill()  # Force kill
            self.get_logger().info("Recording stopped.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop recording: {str(e)}")
        finally:
            self.rosbag_process = None


def main(args=None):
    import sys
    if len(sys.argv) != 6:
        print("Usage: python script.py <key> <bag_name> <router_url> <topic_reg_expr> <max_bag_size>")
        return

    key = sys.argv[1]
    bag_name = sys.argv[2]
    router_url = sys.argv[3]
    regex_pattern = sys.argv[4]
    max_bag_size = sys.argv[5]

    print(f"Initializing with key='{key}', bag_name='{bag_name}',  router_url='{router_url}', max_bag_size='{max_bag_size}', and regex_pattern='{regex_pattern}'")
    rclpy.init(args=args)
    node = ProcessListenerNode(key, bag_name, router_url, regex_pattern, max_bag_size)
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
