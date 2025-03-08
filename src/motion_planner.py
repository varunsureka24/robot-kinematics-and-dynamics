import sys
from utils import _slerp, _quaternion_to_rotation, _rotation_to_quaternion
sys.path.append("../config")
import time

import numpy as np
from robot_config import RobotConfig
from frankapy import FrankaArm
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
import rospy

class TrajectoryPlanner:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.max_vel = RobotConfig.MAX_VELOCITY
        self.max_acc = RobotConfig.MAX_ACCELERATION
        self.duty_cycle = 0.5
        self.smaller_dt = self.dt - 0.00965
    
    def generate_straight_line(self, start_pose, end_pose, duration):
        """
        This function creates a smooth straight-line trajectory in Cartesian space.
        ------
        array_like
            Input to either interpolate_cartesian_trajectory() or convert_cartesian_to_joint()

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        - Need start pose (4x4 matrix) and end pose (4x4 matrix)
        - Use linear interpolation for position: p(t) = p0 + t*(p1-p0)
        - Use SLERP (spherical linear interpolation) for rotation
        - Number of points should give enough resolution for smooth motion
        - Each waypoint should be a 4x4 transformation matrix
        """
        n_points = int(duration / self.dt)
        n_points = 100
        # times = np.linspace(0, duration, n_points)

        #*translational
        start_translation = start_pose[:3, 3]
        end_translation = end_pose[:3, 3]
        positions = np.linspace(start_translation, end_translation, n_points)
        
        #*rotational
        start_rotation = start_pose[:3, :3]
        end_rotation = end_pose[:3, :3]
        start_quart, end_quart = _rotation_to_quaternion(start_rotation), _rotation_to_quaternion(end_rotation)

        orientations = []
        for t in np.linspace(0, 1, n_points):
            orientation = _slerp(start_quart, end_quart, t)
            orientations.append(_quaternion_to_rotation(orientation))
 
        waypoints = []
        for position, orientation in zip(positions, orientations):
            waypoint = np.eye(4)
            waypoint[:3, 3] = position
            waypoint[:3, :3] = orientation
            waypoints.append(waypoint)

        return waypoints
    
    def joint_space_lerp(self, start_pose, end_pose, duration):
        n_points = int(duration / self.dt)
        positions = np.linspace(start_pose, end_pose, n_points)
        return positions
        
    def generate_curve(self, center_pose, radius, sweep, flip_concavity = False):
        """
        This function creates a smooth curved trajectory in Cartesian space.

        Parameters
        ----------
        You can define any parameters you need for this function.
            
        Return
        ------(1 - self.duty_cycle)*self.max_vel)
        array_like
            Input to either interpolate_cartesian_trajectory() or convert_cartesian_to_joint()

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        - Need list of points defining the curve
        - Can break curve into segments and use linear interpolation for each
        - Each waypoint is a 4x4 transformation matrix
        - Keep orientation aligned with curve direction
        - PATH_RESOLUTION from TaskConfig helps determine point spacing
        - Line segments should be small enough for smooth motion
        """
        n_points = int(np.abs(sweep)/ (2 * np.pi) * 360) 
        theta_values = np.linspace(0, sweep, n_points)

        waypoints = []
        for theta in theta_values:
            x = -radius * np.cos(theta)
            y = radius * np.sin(theta) if not flip_concavity else -radius * np.sin(theta)
            waypoint = np.eye(4)
            waypoint[:3, 3] = np.array([x, y, 0])
            waypoint_pose = center_pose @ waypoint
            waypoints.append(waypoint_pose)
        # with open("whiteboard_pose.txt", "w") as f:
        #     for i, waypoint in enumerate(waypoints):
        #         np.savetxt(f, waypoint)

        return waypoints
    
    def respace_duration(self, dq, duration):
        #* ensures that duration meets maximum accel and vel constraints
        vm_dur = (dq / ((1 - self.duty_cycle)*self.max_vel))
        a_dur = np.sqrt((dq / (self.duty_cycle*(1 - self.duty_cycle)*self.max_acc)))
        return max(duration, max(vm_dur), max(a_dur))
        
    def respace_t(self, t, tf, tr):
        ftr = tf*tr / (2*(tf - tr))
        a = tf / (2*tr*(tf-tr))
        if 0 <= t < tr:
            return a * t**2
        elif tr <= t <= tf - tr:
            return 0.5*tf + ((tf - 2*ftr) / (tf - 2*tr)) * (t - 0.5*tf)
        elif tf-tr < t <= tf:
            return tf - a*(t - tf)**2
        elif t >= tf:
            raise Exception("whoops t >= tf")
        else:
            return tf
           
    def interpolate_joint_trajectory(self, q_start, q_end, duration):
        """
        Time-parameterize joint trajectory with trapezoidal velocity profile.

        Parameters
        ----------
        joint_trajectory : array_like 
            Array of joint angles

        Returns
        -------
        array_like
            Time-parameterized trajectory with 20ms spacing

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Hints
        -----
        Key Requirements:
        - Timing: Waypoints must be spaced exactly 20ms apart for controller
        - Safety: Stay within MAX_VELOCITY and MAX_ACCELERATION limits 
        - Smoothness: Use trapezoidal velocity profile for acceleration/deceleration

        Implementation:
        - Use max velocity and acceleration from RobotConfig
        - Ensure smooth acceleration and deceleration
        - Keep 20ms between waypoints as required by controller

        """
        frequency = 1 / self.smaller_dt
        q0, qf = q_start, q_end
        dq = qf - q0
        tf = self.respace_duration(dq, duration)
        n_points = int(tf * frequency)
        n_ramp_points = int(self.duty_cycle * n_points)
        trajectory = np.array([np.zeros(7) for _ in range(n_points)])
        tr = tf * self.duty_cycle
        vm = np.minimum(np.minimum(dq / (tf-tr), self.max_vel), self.max_acc*tr)
        # print("vm", vm)
        qa = q0 + (vm / (2 * tr)) * tr**2
        tb = tf - tr
        qb = qa + vm * (tb-tr)

        for i in range(n_points):
            t = self.respace_t(i / frequency, tf, tr)
            if i < n_ramp_points: 
                q = q0 + (vm / (2 * tr)) * t**2
            elif i >= n_points - n_ramp_points: 
                q = qb - (vm / (2 * tr)) * (tf**2 - 2*tf*t + t**2 - tr**2)
            else: 
                q = qa + vm * (t - tr)
            trajectory[i] = q
        return trajectory

class TrajectoryFollower:
    def __init__(self):
        self.dt = 0.02  # Required 20msdq / ( control loop
        self.fa = FrankaArm()
        
    def follow_joint_trajectory(self, joint_trajectory):
        """
        Follow a joint trajectory using dynamic control.
        
        From writeup: Must have 20ms between waypoints and maintain smooth motion
        
        Parameters
        ----------
        joint_trajectory : np.ndarray
            Array of shape (N, 7) containing joint angles for each timestep
        """
        rospy.loginfo('Initializing Sensor Publisher')
        pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        rate = rospy.Rate(1 / self.dt)

        rospy.loginfo('Publishing joints trajectory...')
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(joint_trajectory[0], duration=1000, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(1, joint_trajectory.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=joint_trajectory[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            
            rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            pub.publish(ros_msg)
            rate.sleep()

        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
        pub.publish(ros_msg)

        rospy.loginfo('Done')