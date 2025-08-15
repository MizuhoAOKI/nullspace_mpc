#!/usr/bin/env python3
import rospy
from mpc_eval_msgs.msg import MPCEval
from geometry_msgs.msg import PoseStamped
import os
import sys
import yaml
import csv
import subprocess

class MPCEvaluator:
    def __init__(self):

        # load param
        ## load initial_pose_x
        self.initial_pose_x = rospy.get_param('~initial_pose_x', 0.0)
        ## load initial_pose_y
        self.initial_pose_y = rospy.get_param('~initial_pose_y', 0.0)

        # initialize subscriber
        mpc_eval_topic = rospy.get_param('~mpc_eval_topic', '')
        self.sub = rospy.Subscriber(mpc_eval_topic, MPCEval, self.callback)

        # initialize publisher
        goal_pose_topic = rospy.get_param('~goal_pose_topic', '')
        self.pub = rospy.Publisher(goal_pose_topic, PoseStamped, queue_size=10)

        # initialize timer
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        # load rosparam
        self.scenario_config_path = rospy.get_param('~scenario_config_path', './default.yaml')
        self.eval_result_dir = rospy.get_param('~eval_result_dir', './default')
        self.csv_filepath = os.path.join(os.path.expanduser(self.eval_result_dir), os.path.splitext(os.path.basename(self.scenario_config_path))[0] + '.csv')

        # load scenario config
        if not os.path.exists(os.path.expanduser(self.scenario_config_path)):
            rospy.logerr(f"[mpc_nav_evaluator] {self.scenario_config_path} does not exist")
            sys.exit(1)
        with open(os.path.expanduser(self.scenario_config_path), 'r') as file:
            self.config = yaml.safe_load(file)

        # write header to csv file
        with open(self.csv_filepath, "w") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "time_stamp",
                    "state_cost",
                    "global_x",
                    "global_y",
                    "global_yaw",
                    "cmd_vx",
                    "cmd_vy",
                    "cmd_yawrate",
                    "cmd_steer_fl",
                    "cmd_steer_fr",
                    "cmd_steer_rl",
                    "cmd_steer_rr",
                    "cmd_rotor_fl",
                    "cmd_rotor_fr",
                    "cmd_rotor_rl",
                    "cmd_rotor_rr",
                    "calc_time_ms",
                    "goal_reached",
                ]
            )

        # initialize variables
        self.calc_time_ms = 0.0 #[ms]
        self.node_launch_time = rospy.Time.now() # get roslaunch time
        self.launch_waiting_time = 5.0 # [s] after launching this node
        self.cooling_time = 0.1 # [s] after publishing goal pose
        self.goal_publishing_time = rospy.Time.now() # get goal publishing time
        self.published_first_goal = False
        self.reached_goal_count = 0
        self.total_goal_count = len(self.config['goals'])
        self.latest_goal_reached_pos_x = self.initial_pose_x
        self.latest_goal_reached_pos_y = self.initial_pose_y

    def kill_simulation(self):
        # kill simulation
        WORKSPACE_DIR = '~/nullspace_mpc'
        KILL_ALL_CMD = 'make killall'
        subprocess.Popen(KILL_ALL_CMD, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, cwd=os.path.expanduser(WORKSPACE_DIR))

    def callback(self, mpc_eval_msg):
        # parse and save mpc evaluation message
        goal_reached = mpc_eval_msg.goal_reached
        DISTANCE_THRESHOLD = 0.5
        is_remaining_last_goal = True if (mpc_eval_msg.global_x - self.latest_goal_reached_pos_x) ** 2 + (mpc_eval_msg.global_y - self.latest_goal_reached_pos_y) ** 2 < DISTANCE_THRESHOLD ** 2 else False
        is_final_goal_reached = False

        # at the timing of reaching the goal
        if goal_reached and not is_remaining_last_goal:
            rospy.logwarn("[mpc_nav_evaluator] publish next goal pose.")
            if (rospy.Time.now() - self.goal_publishing_time).to_sec() > self.cooling_time:

                # add goal reached count
                self.reached_goal_count += 1
                self.latest_goal_reached_pos_x = mpc_eval_msg.global_x
                self.latest_goal_reached_pos_y = mpc_eval_msg.global_y
                rospy.loginfo(f"[mpc_nav_evaluator] goal reached count: {self.reached_goal_count}")
                rospy.loginfo(f"[mpc_nav_evaluator] latest goal reached position: ({self.latest_goal_reached_pos_x}, {self.latest_goal_reached_pos_y})")

                # check if it is the last goal in this episode scenario
                if self.reached_goal_count < self.total_goal_count:
                    next_goal_x = self.config['goals'][self.reached_goal_count]['goal_x']
                    next_goal_y = self.config['goals'][self.reached_goal_count]['goal_y']
                    self.publish_goal_pose(next_goal_x, next_goal_y)
                else:
                    is_final_goal_reached = True

        # save data to csv file
        with open(self.csv_filepath, "a") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    mpc_eval_msg.header.stamp.to_sec(),
                    mpc_eval_msg.state_cost,
                    mpc_eval_msg.global_x,
                    mpc_eval_msg.global_y,
                    mpc_eval_msg.global_yaw,
                    mpc_eval_msg.cmd_vx,
                    mpc_eval_msg.cmd_vy,
                    mpc_eval_msg.cmd_yawrate,
                    mpc_eval_msg.cmd_steer_fl,
                    mpc_eval_msg.cmd_steer_fr,
                    mpc_eval_msg.cmd_steer_rl,
                    mpc_eval_msg.cmd_steer_rr,
                    mpc_eval_msg.cmd_rotor_fl,
                    mpc_eval_msg.cmd_rotor_fr,
                    mpc_eval_msg.cmd_rotor_rl,
                    mpc_eval_msg.cmd_rotor_rr,
                    mpc_eval_msg.calc_time_ms,
                    self.reached_goal_count,
                ]
            )

        # kill simulation if all goals are reached
        if is_final_goal_reached:
            rospy.logwarn("[mpc_nav_evaluator] all goals are reached.")
            self.kill_simulation()
            return

    def timer_callback(self, event):
        # get elapsed time since this node is launched.
        elapsed_time = (rospy.Time.now() - self.node_launch_time).to_sec()

        # announce elapsed time as rosinfo. 
        rospy.loginfo(f"[mpc_nav_evaluator] elapsed time: {elapsed_time} [s]")

        # publish first goal pose after waiting for launch_waiting_time [s]
        if elapsed_time > self.launch_waiting_time and not self.published_first_goal:
            rospy.logwarn("[mpc_nav_evaluator] publish first goal pose.")
            self.publish_goal_pose(self.config['goals'][0]['goal_x'], self.config['goals'][0]['goal_y'])
            self.published_first_goal = True

    def publish_goal_pose(self, goal_x: float, goal_y: float):
        """publish goal pose to (goal_x, goal_y)
        Args:
            goal_x (float): x position of goal pose
            goal_y (float): y position of goal pose
        """
        # publish goal pose as PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.seq = 0
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.pub.publish(goal_pose)
        self.goal_publishing_time = rospy.Time.now()

        # update previous goal position
        rospy.loginfo(f"[mpc_nav_evaluator] publish goal pose: ({goal_x}, {goal_y})")

if __name__ == "__main__":
    # initialize node
    rospy.init_node('mpc_nav_evaluator')

    # create object
    evaluator = MPCEvaluator()

    # spin
    rospy.spin()
