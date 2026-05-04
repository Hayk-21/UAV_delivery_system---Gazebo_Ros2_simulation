#!/usr/bin/env python3
"""Control a specific PX4 drone instance: takeoff, hold, and land.

Usage:
    python3 drone_control.py           # controls default drone (instance 0)
    python3 drone_control.py --id 1    # controls px4_1 (x500_1)
    python3 drone_control.py --id 2    # controls px4_2 (x500_2)
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class DroneController(Node):
    def __init__(self, drone_id: int):
        super().__init__(f'drone_controller_{drone_id}')
        self.drone_id = drone_id

        # Topic prefix: default instance uses /fmu/..., others use /px4_{id}/fmu/...
        prefix = f'/px4_{drone_id}/fmu' if drone_id > 0 else '/fmu'
        self.get_logger().info(f'Controlling drone {drone_id} on {prefix}/...')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{prefix}/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{prefix}/in/vehicle_command', qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.landed = False

        # target_system: instance 0 → sysid 1, instance N → sysid N+1
        self.target_sys = drone_id + 1

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.target_system = self.target_sys
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def timer_callback(self):
        if self.landed:
            return

        if self.counter < 100:
            # Warmup 10s — stream setpoints so PX4 recognises the offboard source
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
            self.counter += 1
        elif self.counter == 100:
            # Switch to Offboard then arm
            self.get_logger().info('Engaging offboard mode and arming')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.counter += 1
        elif self.counter < 300:
            # Hold at 5 m for 20 s
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
            self.counter += 1
        elif self.counter == 300:
            # Land
            self.get_logger().info('Landing')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landed = True


def main():
    parser = argparse.ArgumentParser(description='Control a PX4 drone by instance ID')
    parser.add_argument('--id', type=int, default=0,
                        help='PX4 instance ID (0=default x500, 1=x500_1, 2=x500_2, ...)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DroneController(args.id)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
