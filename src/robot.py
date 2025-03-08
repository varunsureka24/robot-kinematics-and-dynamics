import sys
sys.path.append('../config')
import numpy as np
from robot_config import RobotConfig
from task_config import TaskConfig
from scipy.spatial.transform import Rotation as R


class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7

    def initialize_dh_parameters(self, thetas):
        return np.array([[0,        0,          0.333, thetas[0]], 
                         [0,       -np.pi / 2,      0, thetas[1]],  
                         [0,        np.pi / 2,  0.316, thetas[2]],
                         [0.0825,   np.pi / 2,      0, thetas[3]],
                         [-0.0825, -np.pi / 2,  0.384, thetas[4]],
                         [0,        np.pi / 2,      0, thetas[5]],
                         [0.088,    np.pi / 2,      0, thetas[6]],
                         [0,        0,          0.107,  -np.pi/4],  #flange
                         [0,        0,         0.1034,         0]]) #center of grip
                                                                    #tip of marker
    
    def axis_angle(self, R):
        if R.shape != (3, 3):
            raise ValueError("Input rotation matrix must be 3x3.")
       
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2,-1.0,1.0))
        
        #* handle cases where sin(angle) == 0
        if np.isclose(angle, 0):
            return np.zeros(3), 0
        elif np.isclose(angle, np.pi):
            axis = np.sqrt(np.diagonal(R) + 1) / 2
            axis[np.isnan(axis)] = 0
            return axis / np.linalg.norm(axis), angle
        
        axis = np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) / (2 * np.sin(angle))

        return axis, angle
            
    def forward_kinematics(self, thetas):
        """
        Compute foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        dh_parameters = self.initialize_dh_parameters(thetas)
        
        frames = np.zeros((len(dh_parameters)+1, 4, 4))
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        frames[0] = np.eye(4)
        current_transform = np.eye(4)
        for i in range(len(dh_parameters)):
            a, alpha, d, theta = dh_parameters[i]
            transform_i = np.array([[np.cos(theta),               -np.sin(theta),                0,               a              ],
                                    [np.sin(theta)*np.cos(alpha),  np.cos(theta)*np.cos(alpha), -np.sin(alpha),  -d*np.sin(alpha)],
                                    [np.sin(theta)*np.sin(alpha),  np.cos(theta)*np.sin(alpha),  np.cos(alpha),   d*np.cos(alpha)],
                                    [0,                            0,                            0,               1              ]])
            current_transform = np.matmul(current_transform, transform_i)
            frames[i + 1] = current_transform
        # --------------- END STUDENT SECTION --------------------------------------------------    
        return frames
    
    def jacobians(self, thetas):
        """
        Compute the Jacobians for each frame.start_position

        Parameters
        ----------
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            Jacobians
        """
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')
        
        frames = self.forward_kinematics(thetas)
        jacobians = np.zeros((len(frames) - 1, 6, len(frames-1)))
        z = np.zeros((len(frames), 3))
        o = np.zeros((len(frames), 3))
        
        for frame in range(len(frames)):
            z[frame] = frames[frame][:3, 2]
            o[frame] = frames[frame][:3, 3]

        for joint in range(1, self.dof+2):
            for frame in range(joint, len(frames)):
                J_v = np.cross(z[joint - 1], o[frame] - o[joint - 1])
                J_w = z[joint - 1]
                jacobians[frame - 1, :3, joint - 1] = J_v 
                jacobians[frame - 1, 3:, joint - 1] = J_w
            
        return jacobians[:,:,1:self.dof+1]
        # --------------- END STUDENT SECTION --------------------------------------------

    def inverse_kinematics(self, seed_joints, target_pose):
        """
        Compute inverse kinematics using Jacobian pseudo-inverse method.
        start_position
        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution
        
        Parameters
        ----------
        target_pose : 4x4 np.ndarray
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------seed_joints.shape
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() from robot arm
        - Implement a helper function to track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig, feel free to update them
        """

        if len(seed_joints) != self.dof:
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        thetas = seed_joints
        
        step_size = 0.335
        stopping_condition = 0.005
        
        max_iter = 2000
        num_iter = 0
        
        while num_iter < max_iter: 
            current_pose = self.forward_kinematics(thetas)[-1]
            
            translation_error = target_pose[:3, 3] - current_pose[:3, 3]
            R_current = current_pose[:3, :3]
            R_target = target_pose[:3, :3]
            R_error =  R_target @ R_current.T 
            axis, angle = self.axis_angle(R_error)
            rotation_error = angle * axis
            pose_error = np.hstack((translation_error, rotation_error))

            if np.linalg.norm(pose_error) < stopping_condition:
                return thetas

            end_effector_jacobian = self.jacobians(thetas)[-1]
            cost_gradient = end_effector_jacobian.T @ pose_error
            thetas += step_size * cost_gradient
            num_iter += 1
            
        return None