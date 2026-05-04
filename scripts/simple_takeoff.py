import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_takeoff_node')

        # Configure QoS for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Timer for the control loop (PX4 requires at least 2Hz for Offboard mode)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.landed = False

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        if self.landed:
            return

        if self.offboard_setpoint_counter < 100:
            # 1. Warmup 10s — stream setpoints so PX4 recognises the offboard source
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
            self.offboard_setpoint_counter += 1
        elif self.offboard_setpoint_counter == 100:
            # 2. Switch to Offboard then arm
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.offboard_setpoint_counter += 1
        elif self.offboard_setpoint_counter < 300:
            # 3. Hold at 5 m for 20 s
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
            self.offboard_setpoint_counter += 1
        elif self.offboard_setpoint_counter == 300:
            # 4. Land
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landed = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57  # North
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
