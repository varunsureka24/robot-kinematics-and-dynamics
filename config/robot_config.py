from autolab_core import RigidTransform
import numpy as np

class RobotConfig:
    """Robot-specific configuration parameters"""
    
    # Robot joint configurations
    HOME_JOINTS = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    SAFE_JOINTS = [0, -0.4, 0, -2.0, 0, 1.57, 0.785]
    
    # Joint limits
    JOINT_LIMITS_MIN = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    JOINT_LIMITS_MAX = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
    
    # Tool parameters
    PEN_LENGTH = 0.15  # meters
    PEN_GRASP_WIDTH = 0.02
    GRIPPER_WIDTH_MAX = 0.08
    GRIPPER_WIDTH_MIN = 0.0
    GRIPPER_MAX_FORCE = 40.0  # N
    
    # Safety parameters
    MAX_VELOCITY = 0.5  # m/s
    MAX_ACCELERATION = 0.5  # m/s^2
    FORCE_THRESHOLD = 10.0  # N
    TORQUE_THRESHOLD = [10.0] * 7  # Nm
    MAX_CONDITION_NUMBER = 100.0
    
    # Motion parameters
    APPROACH_DISTANCE = 0.1  # m
    DRAWING_FORCE = 2.0  # N
    DEFAULT_DURATION = 3.0  # seconds
    CARTESIAN_IMPEDANCES = [3000.0, 3000.0, 1000.0, 300.0, 300.0, 300.0]
    
    @staticmethod
    def get_tool_delta_pose():
        """Returns RigidTransform for pen tip offset"""
        return RigidTransform(
            translation=[0, 0, RobotConfig.PEN_LENGTH],
            from_frame='franka_tool',
            to_frame='franka_tool_base'
        )
    
    @staticmethod
    def get_default_impedances():
        """Returns default impedance parameters"""
        return {
            'translational': [3000.0, 3000.0, 1000.0],
            'rotational': [300.0, 300.0, 300.0],
            'nullspace': [10.0] * 7
        }