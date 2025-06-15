#!/usr/bin/python3

import rclpy, math
from rclpy.node import Node
from rclpy.time import Time, Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy


from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped, TransformStamped, QuaternionStamped, PointStamped, Vector3Stamped, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage
from smarc_msgs.msg import Topics as SmarcTopics

class dji_controller(Node):
    def __init__(self):
        super().__init__("DjiControllerNode")
        self.logger = self.get_logger
        self.declare_node_parameters()
        self.robot_name = self.get_parameter("robot_name").value

        self.output = Float32()
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.kP = self.get_parameter("p_gain").value
        self.kD = self.get_parameter("d_gain").value
        self.kP_vert = self.get_parameter("p_gain_vert").value
        self.kD_vert = self.get_parameter("d_gain_vert").value
        self.output_limit = self.get_parameter("output_limit").value
        self.setpoint_max_age = self.get_parameter("setpoint_max_age")
        self.base_pose_max_age = self.get_parameter("base_pose_max_age")
        self.create_subscription(
            PoseStamped,
            self.get_parameter("setpoint_topic"),
            self._move_to_setpoint_callback,
            qos_profile=10)
        self.create_subscription(
            Odometry,
            SmarcTopics.ODOM_TOPIC,
            self.odom_callback,
            qos_profile=10)

        self.output_timer : Timer | None = None
        self.output_pub = self.create_publisher(Joy, self.get_parameter("output_topic").value, qos_profile = 10)
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.ODOM_FRAME = self.robot_name + "/odom"
        self.BASE_FLAT_FRAME = self.robot_name + "/base_flat_link"

        self.base_pose_in_odom : None | Odometry = None
        self.base_pose_flat_in_odom : None | PoseStamped = None


    def declare_node_parameters():
        self.declare_parameter("robot_name", "quadrotor")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("p_gain", 0.0)
        self.declare_parameter("d_gain", 0.0)
        self.declare_parameter("p_gain_vert", 0.0)
        self.declare_parameter("d_gain_vert", 0.0)
        self.declare_parameter("output_limit", 0.4)
        self.declare_parameter("setpoint_topic", "move_to_setpoint")
        self.declare_parameter("output_topic", "suggest_joy")
        self.declare_parameter("setpoint_max_age", .5)
        self.declare_parameter("base_pose_max_age", .5)

    @property
    def now_stamp(self):
        return self.get_clock().now().to_msg()

    def _move_to_setpoint_callback(self, msg: PoseStamped):
        # check if the message is too old
        if (self.now_stamp.sec - msg.header.stamp.sec) + \
           (self.now_stamp.nanosec - msg.header.stamp.nanosec) * 1e-9 > self.setpoint_max_age:
            self.log(f"Move to setpoint message is older than {self.setpoint_max_age} s, ignoring it.")
            self._move_to_setpoint = None
            return

        if msg.header.frame_id != self.ODOM_FRAME:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.ODOM_FRAME, 
                    msg.header.frame_id, 
                    Time(seconds=0),
                    timeout=Duration(seconds=1)
                )
                self._move_to_setpoint = do_transform_pose_stamped(msg, tf)
            except Exception as e:
                self.log(f"Failed to transform move to setpoint from {msg.header.frame_id} to {self.ODOM_FRAME}: {e}")
                self._move_to_setpoint = None
                return
        else:
            self._move_to_setpoint = msg

        self.log(f"Move to setpoint received: {format_pose_stamped(self._move_to_setpoint)}")
        
        if self.output_timer is None:
            self.output_timer = self.create_timer(0.1, self.suggest_move)
            self.log("Output timer started")
    
    def suggest_move(self):
        def cancel_output_timer():
            if self.output_timer is not None:
                self.output_timer.cancel()
                self.output_timer = None
                self.log("Output timer cancelled.")
        if self._move_to_setpoint is None:
            self.log("No move to setpoint set, cannot suggest move.")
            return
    
        if (self.now_stamp.sec - self._move_to_setpoint.header.stamp.sec) + \
        (self.now_stamp.nanosec - self._move_to_setpoint.header.stamp.nanosec) * 1e-9 > self.setpoint_max_age:
            self.log(f"Move to setpoint message is older than {self.setpoint_max_age} s, cancelling joy timer.")
            self._move_to_setpoint = None
            cancel_output_timer()
            return
        
        tf_diff = self.tf_buffer.lookup_transform(
            target_frame = self.BASE_FLAT_FRAME,
            source_frame = self.move_to_setpoint.header.frame_id,
            time=Time(seconds=0),
            timeout=Duration(seconds=1)
        )
        
        target_in_base = do_transform_pose_stamped(self.move_to_setpoint, tf_diff)
        e_forw = target_in_base.pose.position.x
        e_left = target_in_base.pose.position.y
        e_updn = target_in_base.pose.position.z


            

    def log(self, msg : String):
        self.get_logger.info(msg)
    
    def odom_callback(self, msg : Odometry):
        self.base_pose_in_odom = PoseStamped()
        self.base_pose_in_odom.header = odom.header
        self.base_pose_in_odom.pose = msg.pose.pose
        if(self.now_stamp.sec - self.base_in_odom.header.stamp.sec) + \
           (self.now_stamp.nanosec - self.base_in_odom.header.stamp.nanosec) * 1e-9 > self.base_pose_max_age:
           self.log(f"Base pose is older than {self.base_pose_max_age}s, ignoring it.")
           self.base_flat_in_odom = None
        if(self.base_pose_in_odom is not None):
            self.base_flat_in_odom = PoseStamped()
            self.base_flat_in_odom.header = self.base_pose_in_odom.header
            self.base_flat_in_odom.pose.position = self.base_pose_in_odom.pose.position
            rpy_enu = euler_from_quaternion([base_pose_in_odom.pose.quaternion.x, base_pose_in_odom.pose.quaternion.y, base_pose_in_odom.pose.quaternion.z, base_pose_in_odom.pose.quaternion.w])


        

def format_pose_stamped(pose: PoseStamped|None) -> str:
        if( pose is None):
            return "None"
        rpy = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ])
        return f"(x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}, " \
               f"roll={math.degrees(rpy[0]):.3f}, pitch={math.degrees(rpy[1]):.3f}, yaw={math.degrees(rpy[2]):.3f}, " \
               f"frame_id={pose.header.frame_id})"