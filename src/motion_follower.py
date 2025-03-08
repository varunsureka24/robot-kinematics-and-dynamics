from robot import Robot
import numpy as np
import time

class MotionFollower():
    def __init__(self, trajectory_planner, trajectory_follower):
        self.HOME_JOINTS = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
        self.PEN_LENGTH = 0.15
        self.robot = Robot()
        self.trajectory_planner = trajectory_planner
        self.trajectory_follower = trajectory_follower
        self.pen_holder_pose = np.eye(4)
        self.whiteboard_pose = np.eye(4)
        self.drop_bin_pose = np.eye(4)
        self.start_cartesian = self.robot.forward_kinematics(self.HOME_JOINTS)[-1]
        self.prev_js = np.copy(self.HOME_JOINTS)
        self.drop_bin_theta = 2/3 * np.pi
        self.drop_bin_rotation_joints = np.array([-0.01435332,  0.05754631, -0.00734021, -2.2443875,  -0.00362317,  1.35387358, 0.83542864])
        self.drop_bin_cartesian = np.eye(4)
        
    def load_calibration(self):
        pen_holder_pose = np.load("pen_holder_pose.npy")
        self.whiteboard_pose = np.load("whiteboard_pose.npy")
        drop_bin_pose = np.load("drop_bin_pose.npy")
        return pen_holder_pose, drop_bin_pose
    
    def drop_bin_rotation(self):
        self.drop_bin_cartesian = self.robot.forward_kinematics(self.drop_bin_rotation_joints)[-1]
        self.drop_bin_pose[:3,:3] = self.drop_bin_cartesian[:3,:3]
        return self.drop_bin_pose

    def create_calibration_poses(self, pen_holder_translation, drop_bin_translation):
        self.pen_holder_pose[:3, :3] = self.start_cartesian[:3,:3]
        self.pen_holder_pose[:3, 3] = pen_holder_translation
        self.drop_bin_pose[:3, :3] = self.drop_bin_rotation()[:3,:3]
        self.drop_bin_pose[:3, 3] = drop_bin_translation
        
    def calibrate_poses(self):
        pen_holder_translation, drop_bin_translation = self.load_calibration()
        self.create_calibration_poses(pen_holder_translation, drop_bin_translation)
        
    def rotate_y_axis(self, start_pose):
        R_y = np.array([[ np.cos(self.drop_bin_theta), 0, np.sin(self.drop_bin_theta)],
                        [ 0,                           1,                          0 ],
                        [-np.sin(self.drop_bin_theta), 0, np.cos(self.drop_bin_theta)]])
        
        rotated_pose = np.eye(4)
        rotated_pose[:3, :3] = R_y @ start_pose[:3, :3]
        rotated_pose[:3, 3] = start_pose[:3, 3]
        return rotated_pose
    
    def apply_z_offset(self, start_pose, distance):
        translation = np.array([0, 0, distance])
        return self.apply_translation_offset(start_pose, translation)
    
    def apply_translation_offset(self, start_pose, translation_vector):
        offset = np.eye(4)
        offset[:3, 3] = translation_vector
        offset_pose = start_pose @ offset
        return offset_pose
    
    def generate_target_in_straight_line(self, start_pose, distance, angle):
        dx = distance * np.cos(angle)
        dy = distance * np.sin(angle)
        target_translation = np.array([dx, dy, 0])
        return self.apply_translation_offset(start_pose, target_translation)

    def lift_and_lower(self, start_pose, distance, duration = 5):
        lifted_pose = self.apply_lift(start_pose, distance)
        self.move(lifted_pose, duration)
        self.move(start_pose, duration)
        
    def cartesian_to_js_ik(self, waypoints):
        current_pose = np.copy(self.prev_js)
        for i, waypoint in enumerate(waypoints[1:]):
            # print("waypoint", i)
            current_pose = self.robot.inverse_kinematics(current_pose, waypoint)
            if current_pose is None:
                print("failed on :", i, "out of",len(waypoints))
                raise ValueError("crashout error")
        return current_pose
    
    def cartesian_to_ik_list(self, waypoints):
        js = []
        current_pose = np.copy(self.prev_js)
        for i, waypoint in enumerate(waypoints[1:]):
            current_pose = self.robot.inverse_kinematics(current_pose, waypoint)
            if current_pose is None:
                print("failed on :", i, "out of",len(waypoints))
                raise ValueError("crashout error")
            js.append(np.copy(current_pose))
        self.prev_js = current_pose
        return np.array(js)

    def get_trajectory_end_js(self, target, duration = 10):
        start_position = self.robot.forward_kinematics(self.prev_js)[-1]
        cartesian_trajectory = self.trajectory_planner.generate_straight_line(start_position, target, duration)
        return self.cartesian_to_js_ik(cartesian_trajectory)

    def follow_js_trajectory(self, qf, duration = 5):
        js_trajectory = self.trajectory_planner.interpolate_joint_trajectory(self.prev_js, qf, duration)
        self.trajectory_follower.follow_joint_trajectory(js_trajectory)
        self.prev_js = qf
        
    def move(self, target, duration = 5):
        target_js = self.get_trajectory_end_js(target, duration)
        self.follow_js_trajectory(target_js, duration)
        time.sleep(0.5)

    def draw_line(self, distance, duration = 5, angle = 0):
        start_pose = self.robot.forward_kinematics(self.prev_js)[-1]
        target_pose = self.generate_target_in_straight_line(start_pose, distance, angle)
        self.move(target_pose, duration)
        return start_pose, target_pose
        
    def draw_curve(self, radius, sweep, angle = 0, flip_concavity = False):
        start_pose = self.robot.forward_kinematics(self.prev_js)[-1]
        center_pose = self.generate_target_in_straight_line(start_pose, radius, angle)
        waypoints = self.trajectory_planner.generate_curve(center_pose, radius, sweep, flip_concavity)
        curve_js = self.cartesian_to_ik_list(waypoints)
        self.trajectory_follower.follow_joint_trajectory(curve_js)
        time.sleep(0.5)
        return waypoints[0], waypoints[-1]