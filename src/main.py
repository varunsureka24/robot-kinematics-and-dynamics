import argparse
from motion_planner import TrajectoryPlanner, TrajectoryFollower
from motion_follower import MotionFollower
import numpy as np

# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

PEN_HOLDER_OFFSET = 0.25
PEN_WHITEBOARD_OFFSET = -0.525
SENSOR_DISTANCE_INPUT = 0.00125

RED_FISH_TAIL_ANGLE = np.pi/6
BLUE_FISH_TAIL_ANGLE = np.pi/6
FISH_TAIL_VERTICAL_DISTANCE = 0.05
FISH_HEAD_RADIUS = 0.05
FISH_VERTICAL_DISTANCE = 0.075
FISH_TAIL_DISTANCE = FISH_TAIL_VERTICAL_DISTANCE * (np.cos(RED_FISH_TAIL_ANGLE)/ np.sin(RED_FISH_TAIL_ANGLE))

def do_while_offset(target_pose, distance, motion_follower, duration = 5):
    while(True):
        distance = abs(distance)
        print(distance)
        user_input = input("pen good? ")
        if user_input == 'y':
            break
        elif user_input == '-':
            distance = - distance
        print(target_pose)
        target_pose = motion_follower.apply_z_offset(target_pose, distance)
        print(target_pose)
        motion_follower.move(target_pose, duration)

def undo_line(start_pose, target_pose, line_length, offset_distance, motion_follower, duration = 5, angle = 0):
    user_input = input("line good? ")
    if user_input == 'y':
        return
    do_while_offset(target_pose, 5 * offset_distance, motion_follower, duration = 5)
    motion_follower.move(start_pose, duration)
    do_while_offset(start_pose, offset_distance, motion_follower, duration = 5)
    motion_follower.draw_line(line_length, duration, angle)

def undo_curve(start_pose, target_pose, radius, sweep, offset_distance, motion_follower, flip_concavity= False, duration = 5):
    user_input = input("curve good? ")
    if user_input == 'y':
        return
    do_while_offset(target_pose, 5 * offset_distance, motion_follower, duration = 5)
    motion_follower.move(start_pose, duration)
    do_while_offset(start_pose, offset_distance, motion_follower, duration = 5)
    motion_follower.draw_curve(radius, sweep, flip_concavity)
    
def full_movement(pen_location, motion_follower, trajectory_follower):
    #* home to pen holder
    motion_follower.move(pen_location, duration = 5)

    #* lower pen to pre-pick
    pre_pick_pen_1 = motion_follower.apply_z_offset(pen_location, PEN_HOLDER_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(pre_pick_pen_1, duration = 4)
    do_while_offset(pre_pick_pen_1, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    
    #* pick up pen
    trajectory_follower.fa.close_gripper()
    
    #* lift pen
    post_pick_pen_1 = motion_follower.apply_z_offset(pre_pick_pen_1, -2 * motion_follower.PEN_LENGTH)
    motion_follower.move(post_pick_pen_1, duration = 3)
    
    #* go to whiteboard (with pen offset):
    whiteboard_pose = motion_follower.apply_z_offset(motion_follower.whiteboard_pose, PEN_WHITEBOARD_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(whiteboard_pose, duration=5)
    do_while_offset(whiteboard_pose, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    
    #* draw line
    line_start, line_end = motion_follower.draw_line(-0.1)
    undo_line(line_start, line_end, -0.1, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    
    #* lift pen from whiteboard
    lifted_whiteboard_pose = motion_follower.apply_z_offset(line_end, 0.5 * PEN_WHITEBOARD_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(lifted_whiteboard_pose, duration = 5)
    
    #* lower pen to whiteboard
    motion_follower.move(line_end, duration = 5)
    do_while_offset(line_end, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)

    #* draw curve
    curve_start, curve_end = motion_follower.draw_curve(radius = -0.05, sweep = np.pi, flip_concavity= False)
    undo_curve(curve_start, curve_end, radius = -0.05, sweep = np.pi, offset_distance = SENSOR_DISTANCE_INPUT, motion_follower = motion_follower, flip_concavity = False, duration = 5)
    
    #* lift pen from whiteboard
    lifted_whiteboard_pose = motion_follower.apply_z_offset(curve_end, 0.5* PEN_WHITEBOARD_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(lifted_whiteboard_pose, duration = 3)
    
    #* board to drop location
    motion_follower.move(motion_follower.drop_bin_pose, duration = 5)
    
    #* drop pen
    trajectory_follower.fa.open_gripper()
    
    #* go back to home! 
    trajectory_follower.fa.reset_joints()
    motion_follower.prev_js = np.copy(motion_follower.HOME_JOINTS)
    
def pen_to_whiteboard(pen_location, whiteboard_pose, motion_follower, trajectory_follower):
    #* home to pen holder
    motion_follower.move(pen_location, duration = 5)

    #* lower pen to pre-pick
    pre_pick_pen_1 = motion_follower.apply_z_offset(pen_location, PEN_HOLDER_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(pre_pick_pen_1, duration = 5)
    do_while_offset(pre_pick_pen_1, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    
    #* pick up pen
    trajectory_follower.fa.close_gripper()
    
    #* lift pen
    post_pick_pen_1 = motion_follower.apply_z_offset(pre_pick_pen_1, -2 * motion_follower.PEN_LENGTH)
    motion_follower.move(post_pick_pen_1, duration = 5)
    
    #* go to whiteboard (with pen offset):
    offset_whiteboard_pose = motion_follower.apply_z_offset(whiteboard_pose, PEN_WHITEBOARD_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(offset_whiteboard_pose, duration=5)
    do_while_offset(offset_whiteboard_pose, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)

def drop_pen(motion_follower, trajectory_follower):
    #* board to drop location
    motion_follower.move(motion_follower.drop_bin_pose, duration = 5)
    
    #* drop pen
    trajectory_follower.fa.open_gripper()

def go_home(motion_follower, trajectory_follower):
    trajectory_follower.fa.reset_joints()
    motion_follower.prev_js = np.copy(motion_follower.HOME_JOINTS)

def one_fish(pen_location, whiteboard_pose, motion_follower, trajectory_follower):
    pen_to_whiteboard(pen_location, whiteboard_pose, motion_follower, trajectory_follower)
    red_fish(RED_FISH_TAIL_ANGLE, FISH_TAIL_VERTICAL_DISTANCE, FISH_HEAD_RADIUS, FISH_TAIL_DISTANCE, motion_follower)
    drop_pen(motion_follower, trajectory_follower)
    go_home(motion_follower, trajectory_follower)

def two_fish(pen_location, whiteboard_pose, motion_follower, trajectory_follower):
    pen_to_whiteboard(pen_location, whiteboard_pose, motion_follower, trajectory_follower)
    blue_fish(BLUE_FISH_TAIL_ANGLE, FISH_TAIL_VERTICAL_DISTANCE, FISH_HEAD_RADIUS, FISH_TAIL_DISTANCE, motion_follower)
    drop_pen(motion_follower, trajectory_follower)
    go_home(motion_follower, trajectory_follower)

#* lower fish
def red_fish(angle, vertical_distance, radius, tail_distance, motion_follower):
    line_start, line_end = motion_follower.draw_line(vertical_distance, duration = 5, angle = np.pi / 2)
    undo_line(line_start, line_end, vertical_distance, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    line_start, line_end = motion_follower.draw_line(-tail_distance, duration = 5, angle = angle)
    undo_line(line_start, line_end, -tail_distance, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    start_pose, target_pose = motion_follower.draw_curve(-radius, np.pi, flip_concavity = False)
    lifted_whiteboard_pose = motion_follower.apply_z_offset(target_pose, 0.5* PEN_WHITEBOARD_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(lifted_whiteboard_pose, duration = 5)

#* upper fish
def blue_fish(angle, vertical_distance, radius, tail_distance, motion_follower):
    line_start, line_end = motion_follower.draw_line(-vertical_distance, duration = 5, angle = np.pi / 2)
    undo_line(line_start, line_end, vertical_distance, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5,)
    line_start, line_end = motion_follower.draw_line(-tail_distance, duration = 5, angle = -angle) #? maybe 45°
    undo_line(line_start, line_end, vertical_distance, SENSOR_DISTANCE_INPUT, motion_follower, duration = 5)
    start_pose, target_pose = motion_follower.draw_curve(-radius, np.pi, flip_concavity = True)
    lifted_whiteboard_pose = motion_follower.apply_z_offset(target_pose, 0.5* PEN_WHITEBOARD_OFFSET * motion_follower.PEN_LENGTH)
    motion_follower.move(lifted_whiteboard_pose, duration = 5)
    
def offset_pen_start(prev_pen_pose):
    pen_pose = np.copy(prev_pen_pose)
    pen_pose[0, 3] += 0.029
    return pen_pose

def main():
    trajectory_planner = TrajectoryPlanner()
    trajectory_follower = TrajectoryFollower()
    motion_follower = MotionFollower(trajectory_planner, trajectory_follower)
    trajectory_follower.fa.open_gripper()
    trajectory_follower.fa.reset_joints()
    
    #* calibrate
    motion_follower.calibrate_poses()

    #? fishy
    #* pen 1
    pen_1_pose = motion_follower.pen_holder_pose
    pen_1_whiteboard_pose = motion_follower.whiteboard_pose
    one_fish(pen_1_pose, pen_1_whiteboard_pose, motion_follower, trajectory_follower)
    
    #* pen 2
    pen_2_pose = offset_pen_start(pen_1_pose)
    pen_2_whiteboard_pose = motion_follower.whiteboard_pose
    two_fish(pen_2_pose, pen_2_whiteboard_pose, motion_follower, trajectory_follower)
    
    #* pen 3
    pen_3_pose = offset_pen_start(pen_2_pose)
    pen_3_whiteboard_pose = motion_follower.apply_translation_offset(motion_follower.whiteboard_pose, np.array([0, -FISH_VERTICAL_DISTANCE, 0]))
    one_fish(pen_3_pose, pen_3_whiteboard_pose, motion_follower, trajectory_follower)
    
    #* pen 4
    pen_4_pose = offset_pen_start(pen_3_pose)
    pen_4_whiteboard_pose = np.copy(pen_3_whiteboard_pose)
    two_fish(pen_4_pose, pen_4_whiteboard_pose, motion_follower, trajectory_follower)

if __name__ == '__main__':
    main()