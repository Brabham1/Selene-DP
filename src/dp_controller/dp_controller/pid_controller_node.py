import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from message_filters import Subscriber, ApproximateTimeSynchronizer

from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Wrench, Quaternion

import pymap3d as pm
import numpy as np
from dp_controller.pid_controller import Pid

class pid_controller(Node):
    def __init__(self):
        super().__init__("pid_controller")
        self.sub_enable_dp = self.create_subscription(
            Bool,
            "/dp_control/station_keeping",
            self.enable_callback,
            10
        )
        self.enable = False

        self.sub_gps = Subscriber(self, NavSatFix, "/selene/gps")
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        self.sub_imu = Subscriber(self, Imu, "/selene/imu")

        self.synched_pos = ApproximateTimeSynchronizer(
            [self.sub_gps, self.sub_imu],
            queue_size=10,
            slop=0.1
        )
        self.synched_pos.registerCallback(self.pos_callback)
    
        self.pub_wrench = self.create_publisher(
            Wrench,
            "/dp_control/desired_wrench",
            10
        )

        self.current_pos = np.full(3, None)
        self.desired_pos = np.full(3, None)
        
        self.declare_parameter("pid_x", [1.0, 0.0, 0.0])
        self.declare_parameter("pid_y", [1.0, 0.0, 0.0])
        self.declare_parameter("pid_torque", [1.0, 0.0, 0.0])
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.pid_x_param = self.get_parameter("pid_x").value
        self.pid_y_param = self.get_parameter("pid_y").value
        self.pid_torque_param = self.get_parameter("pid_torque").value
        
        self.pid_force_x = Pid(self.pid_x_param)
        self.pid_force_y = Pid(self.pid_y_param)
        self.pid_torque_z = Pid(self.pid_torque_param)
        self.time = 0.0
        self.prev_time = None
        

    def enable_callback(self, msg: Bool):
        self.enable = msg.data
        self.get_logger().info(f"Station keeping set to: {msg.data}")
        

    def pos_callback(self, msg_gps: NavSatFix, msg_imu: Imu):
        if not self.enable:
            self.current_pos = np.full(3, None)
            return
        
        #Time

        stamp = msg_gps.header.stamp
        self.time = stamp.sec + stamp.nanosec * 1e-9
        
        #GPS------------------
        lat = msg_gps.latitude
        lon = msg_gps.longitude
        alt = msg_gps.altitude

        if self.home_lat is None:
            self.home_lat = lat
            self.home_lon = lon
            self.home_alt = alt

        n,e,d = pm.geodetic2ned(
            lat, lon, alt,
            self.home_lat, self.home_lon, self.home_alt
        )

        self.current_pos[0] = n
        self.current_pos[1] = e
        #IMU------------------
        orientation: Quaternion = msg_imu.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        self.current_pos[2] = yaw

        self.get_logger().debug(f"Got position N: {self.current_pos[0]}, E: {self.current_pos[1]}, Yaw: {self.current_pos[2]}")
        self.pid_calc()

    def pid_calc(self):
        if None in self.current_pos or not self.enable: #Ensures that PID is reset and disabled when enable=False
            self.desired_pos = np.full(3, None)
            return
        if None in self.desired_pos: #Sets current 
                self.desired_pos = self.current_pos.copy()

        if self.prev_time is None:
            self.prev_time = self.time
            return
        
        
        dt = self.time - self.prev_time
        error = self.desired_pos - self.current_pos
        error[2]= np.arctan2(np.sin(error[2]), np.cos(error[2])) #Angle wrapping

        desired_force_x = self.pid_force_x.calc(error[0], dt)
        desired_force_y = self.pid_force_y.calc(error[1], dt)
        desired_torque_z = self.pid_torque_z.calc(error[2], dt)

        msg: Wrench = Wrench()
        msg.force.x = desired_force_x
        msg.force.y = desired_force_y
        msg.torque.z = desired_torque_z
        self.pub_wrench.publish(msg)
        self.get_logger().info(f" | Publishing F_x: {desired_force_x}, F_y: {desired_force_x}, T_z: {desired_torque_z} | Err_x: {error[0]}, Err_y: {error[1]}, Err_yaw: {error[2]}")

        self.prev_time = self.time

    def parameter_callback(self, params):
        """Callback to handle parameter updates."""
        for param in params:
            if param.name == 'pid_x':
                if len(param.value) == 3:
                    self.pid_force_x.setParams(param.value)
                    self.get_logger().info(f' {param.name}| reference was set: {param.value}')

                    return SetParametersResult(successful = True)
                return SetParametersResult(successful = False)
            
            if param.name == 'pid_y':
                if len(param.value) == 3:
                    self.pid_force_y.setParams(param.value)
                    self.get_logger().info(f' {param.name}| reference was set: {param.value}')
                    return SetParametersResult(successful = True)
                return SetParametersResult(successful = False)
            
            if param.name == 'pid_torque':
                if len(param.value) == 3:
                    self.pid_torque_z.setParams(param.value)
                    self.get_logger().info(f' {param.name}| reference was set: {param.value}')

                    return SetParametersResult(successful = True)
                return SetParametersResult(successful = False)
        
def main(args=None):
    rclpy.init(args=args)

    pid_controller_node = pid_controller()
    rclpy.spin(pid_controller_node)

    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
