#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import time

class MQTTBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        
        # MQTT Configuration
        self.broker = "localhost"
        self.port = 1883
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(self.broker, self.port, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            raise

        # QoS Profiles
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ROS 2 Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_rf2o', self.odom_callback, odom_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # ROS 2 Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)

        # MQTT Topics
        self.odom_topic = "factory/robot/odom"
        self.map_topic = "factory/robot/map"
        self.cmd_vel_topic = "factory/robot/cmd_vel"

        self.get_logger().info("MQTT Bridge Node Started")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker")
            self.client.subscribe(self.cmd_vel_topic, qos=1)
        else:
            self.get_logger().error(f"Failed to connect with code {rc}")
            time.sleep(5)
            self.client.reconnect()

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            twist = Twist()
            twist.linear.x = float(payload.get('linear_x', 0.0))
            twist.angular.z = float(payload.get('angular_z', 0.0))
            self.cmd_vel_pub.publish(twist)
            self.get_logger().debug(f"Published to /cmd_vel: {payload}")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def odom_callback(self, msg):
        odom_data = {
            'position': {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'z': float(msg.pose.pose.position.z)
            },
            'orientation': {
                'x': float(msg.pose.pose.orientation.x),
                'y': float(msg.pose.pose.orientation.y),
                'z': float(msg.pose.pose.orientation.z),
                'w': float(msg.pose.pose.orientation.w)
            }
        }
        try:
            self.client.publish(self.odom_topic, json.dumps(odom_data), qos=1)
            self.get_logger().debug("Published odom data to MQTT")
        except Exception as e:
            self.get_logger().error(f"Error publishing odom data: {e}")

    def map_callback(self, msg):
        map_data = {
            'info': {
                'width': int(msg.info.width),
                'height': int(msg.info.height),
                'resolution': float(msg.info.resolution),
                'origin': {
                    'position': {
                        'x': float(msg.info.origin.position.x),
                        'y': float(msg.info.origin.position.y),
                        'z': float(msg.info.origin.position.z)
                    }
                }
            },
            'data': [int(x) for x in msg.data]  # OccupancyGrid verisi
        }
        try:
            self.client.publish(self.map_topic, json.dumps(map_data), qos=1)
            self.get_logger().info("Published map data to MQTT")
        except Exception as e:
            self.get_logger().error(f"Error publishing map data: {e}")

    def destroy_node(self):
        self.client.loop_stop()
        self.client.disconnect()
        self.get_logger().info("MQTT Bridge Node Stopped")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()