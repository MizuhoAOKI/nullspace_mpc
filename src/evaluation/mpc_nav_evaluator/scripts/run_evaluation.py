#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import yaml
import rospkg
import subprocess
import argparse
import datetime
import numpy as np
import pandas as pd

# global params
WORKSPACE_DIR = '~/nullspace_mpc'
DEFAULT_RESULT_DIR = '~/nullspace_mpc/result'
KILL_ALL_CMD = 'make killall'

# run a single episode of the evaluation
def run_single_episode(
    episode: int,
    max_episode_time: float,
    config_path: str,
    result_dir: str,
    roslaunch_cmd: str,
    total_num_episodes: int = 0,
    show_output_to_terminal: bool = True,
    ):

    # get current time in seconds
    start_time = time.time()

    # run the ros launch command
    cmd = f'{roslaunch_cmd} scenario_config_path:={config_path} eval_result_dir:={result_dir}'
    print(f'[{os.path.basename(config_path)}] [{episode: 4} / {total_num_episodes: 4} episode] [Launching] {cmd}')
    p = subprocess.Popen(cmd, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, cwd=os.path.expanduser(WORKSPACE_DIR))

    # wait for the process to finish
    while p.poll() is None:

        # get elapsed time
        elapsed_time = time.time() - start_time

        # print the output to the terminal
        if show_output_to_terminal:
            # show config file name
            print(f'[{os.path.basename(config_path)}] [{episode: 4} / {total_num_episodes: 4} episode] [Running] [{elapsed_time: 5.0f} / {max_episode_time: 5.0f} sec]', p.stdout.readline().decode().strip())

        # check if the process is taking too long, and kill ros processes it if time is up
        if time.time() - start_time > max_episode_time:
            print(f'\033[31m[{os.path.basename(config_path)}] [{episode: 4} / {total_num_episodes: 4} episode] [Killing] kill the process as it took too long to finish.\033[0m')
            subprocess.Popen(KILL_ALL_CMD, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, cwd=os.path.expanduser(WORKSPACE_DIR))
            print('\033[31m'+f'### Episode {episode} finished because time is up ###'+'\033[0m')
            return

    # announce that the episode is finished.
    print('\033[32m'+f'### Episode {episode} finished normally ###'+'\033[0m')
    return

# main function
def run_evaluation(agenda_yaml_path: str, controller: str = '', world_name: str = ''):

    # register controller name
    roslaunch_cmd = f'roslaunch launch/navigation.launch local_planner:={controller}  gazebo_world_name:={world_name}'

    # load config if it exists
    if not os.path.exists(os.path.expanduser(agenda_yaml_path)):
        print(f'[ERROR]  {agenda_yaml_path} does not exist')
        sys.exit(1)
    with open(os.path.expanduser(agenda_yaml_path), 'r') as file:
        agenda = yaml.safe_load(file)
    
    # get result dir
    if not os.path.exists(os.path.expanduser(agenda['result_dir'])):
        print(f'[ERROR]  {os.path.join(os.path.expanduser(agenda_yaml_path), "result_dir")} does not exist')
        sys.exit(1)
    RESULT_DIR = os.path.join(os.path.expanduser(agenda['result_dir']), "eval_" + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.expanduser(RESULT_DIR), exist_ok=True)
    os.system(f'cp {agenda_yaml_path} {os.path.join(RESULT_DIR, f"agenda_{controller}.yaml")}') # copy agenda file to the result dir

    # get general params
    MAX_EPISODE_TIME = agenda['maximum_time_per_single_episode'] # [s] if the process does not finish in this time, it is killed forcibly
    COOLING_TIME_BETWEEN_EPISODES = agenda['cooling_time_between_episodes']  # [s] wait for a while before starting the next episode
    TOTAL_NUM_EPISODES = agenda['total_number_of_episodes']  # total number of episodes to run

    # check if the total number of episodes is valid
    if TOTAL_NUM_EPISODES > len(agenda["scenarios"]):
        print(f'[ERROR]  total number of episodes ({TOTAL_NUM_EPISODES}) is greater than the number of scenarios ({len(agenda["scenarios"])})')
        sys.exit(1)

    # check if the all scenarios exist
    for scenario in agenda["scenarios"]:
        if not os.path.exists(os.path.expanduser(scenario)):
            print(f'[ERROR]  {scenario} does not exist')
            sys.exit(1)
    print(f'[INFO] All {len(agenda["scenarios"])} scenarios exist. Start running the evaluation.')

    # make a csv to save summary of the evaluation
    summary_csv_filename = f'summary_{controller}.csv'
    summary_df = pd.DataFrame(columns=[
        'episode',
        'mean_state_cost',
        'min_calc_time',
        'mean_calc_time',
        'max_calc_time',
        'steer_cmd_diff',
        'rotor_cmd_diff',
        'vehicle_cmd_diff',
        'trajectory_length',
        'max_goal_reached',
        'episode_time',
        'all_goals_reached',
    ])
    summary_df.to_csv(os.path.join(RESULT_DIR, summary_csv_filename), index=False)

    # initialize variables
    episode = 0 # current episode number (0 ~ TOTAL_NUM_EPISODES-1)

    # run all episodes
    for episode in range(TOTAL_NUM_EPISODES):

        # load config file for this episode
        scenario_config_path = agenda["scenarios"][episode]
        if not os.path.exists(os.path.expanduser(scenario_config_path)):
            print(f'[ERROR]  {scenario_config_path} does not exist')
            sys.exit(1)

        # keep how many goals are set 
        with open(os.path.expanduser(scenario_config_path), 'r') as file:
            config = yaml.safe_load(file)
        total_goal_count = len(config['goals'])

        # run the episode
        run_single_episode(
            episode=episode, 
            max_episode_time=MAX_EPISODE_TIME, 
            config_path=scenario_config_path,
            result_dir=RESULT_DIR,
            roslaunch_cmd=roslaunch_cmd,
            total_num_episodes=TOTAL_NUM_EPISODES
        )

        # wait for a while before starting the next episode
        time.sleep(COOLING_TIME_BETWEEN_EPISODES)

        # load the result csv
        episode_result_csv_path = os.path.join(RESULT_DIR, f'{os.path.basename(scenario_config_path).split(".")[0]}.csv')
        df = pd.read_csv(episode_result_csv_path)

        # add new column and initialize it with 0
        df['rotor_cmd_diff'] = 0.0
        df['steer_cmd_diff'] = 0.0
        df['vehicle_cmd_diff'] = 0.0

        # add column of control input change
        for i in range(1, len(df)):
            # calculate the difference of control input
            rotor_diff = np.abs(df.at[i, 'cmd_rotor_fl'] - df.at[i-1, 'cmd_rotor_fl']) + \
                         np.abs(df.at[i, 'cmd_rotor_fr'] - df.at[i-1, 'cmd_rotor_fr']) + \
                         np.abs(df.at[i, 'cmd_rotor_rl'] - df.at[i-1, 'cmd_rotor_rl']) + \
                         np.abs(df.at[i, 'cmd_rotor_rr'] - df.at[i-1, 'cmd_rotor_rr'])

            # note: steering angle is [-pi ~ pi]
            diff_steer_fl = np.fmod(np.abs(df.at[i, 'cmd_steer_fl'] - df.at[i-1, 'cmd_steer_fl']), (2.0 * np.pi))
            diff_steer_fl = np.min([diff_steer_fl, 2.0*np.pi - diff_steer_fl])
            diff_steer_fr = np.fmod(np.abs(df.at[i, 'cmd_steer_fr'] - df.at[i-1, 'cmd_steer_fr']), (2.0 * np.pi))
            diff_steer_fr = np.min([diff_steer_fr, 2.0*np.pi - diff_steer_fr])
            diff_steer_rl = np.fmod(np.abs(df.at[i, 'cmd_steer_rl'] - df.at[i-1, 'cmd_steer_rl']), (2.0 * np.pi))
            diff_steer_rl = np.min([diff_steer_rl, 2.0*np.pi - diff_steer_rl])
            diff_steer_rr = np.fmod(np.abs(df.at[i, 'cmd_steer_rr'] - df.at[i-1, 'cmd_steer_rr']), (2.0 * np.pi))
            diff_steer_rr = np.min([diff_steer_rr, 2.0*np.pi - diff_steer_rr])
            steer_diff = diff_steer_fl + diff_steer_fr + diff_steer_rl + diff_steer_rr

            df.at[i, 'rotor_cmd_diff'] = rotor_diff
            df.at[i, 'steer_cmd_diff'] = steer_diff
            df.at[i, 'vehicle_cmd_diff'] =  rotor_diff + steer_diff

        # get summary of the episode
        mean_state_cost = df["state_cost"].mean()
        min_calc_time = df["calc_time_ms"].min()
        mean_calc_time = df["calc_time_ms"].mean()
        max_calc_time = df["calc_time_ms"].max()
        max_goal_reached = df["goal_reached"].max()
        episode_time = df["time_stamp"].max() - df["time_stamp"].min()
        trajectory_length = np.sum(np.sqrt(np.diff(df["global_x"])**2 + np.diff(df["global_y"])**2))
        all_goals_reached = 1 if max_goal_reached == total_goal_count else 0
        sum_steer_cmd_diff = df["steer_cmd_diff"].sum()
        sum_rotor_cmd_diff = df["rotor_cmd_diff"].sum()
        sum_vehicle_cmd_diff = df["vehicle_cmd_diff"].sum()

        # update the summary csv
        summary_df = summary_df.append({
            'episode': episode,
            'mean_state_cost': mean_state_cost,
            'min_calc_time': min_calc_time,
            'mean_calc_time': mean_calc_time,
            'max_calc_time': max_calc_time,
            'steer_cmd_diff': sum_steer_cmd_diff,
            'rotor_cmd_diff': sum_rotor_cmd_diff,
            'vehicle_cmd_diff': sum_vehicle_cmd_diff,
            'trajectory_length': trajectory_length,
            'max_goal_reached': max_goal_reached,
            'episode_time': episode_time,
            'all_goals_reached': all_goals_reached,
        }, ignore_index=True)
        summary_df.to_csv(os.path.join(RESULT_DIR, summary_csv_filename), index=False)

    # Here, all episodes are finished. Now, calculate the success rate and the summary of the whole evaluation

    # calculate the success rate
    success_rate = summary_df['all_goals_reached'].sum() / len(summary_df)

    # remove failed episode from the summary_df
    successful_episodes_df = summary_df[summary_df['all_goals_reached'] == 1]

    # get summary of the evaluation
    mean_state_cost        = successful_episodes_df["mean_state_cost"].mean()
    min_calc_time          = successful_episodes_df["min_calc_time"].min()  # get minimum
    mean_calc_time         = successful_episodes_df["mean_calc_time"].mean()
    max_calc_time          = successful_episodes_df["max_calc_time"].max()  # get maximum
    mean_steer_cmd_diff    = successful_episodes_df["steer_cmd_diff"].mean()
    mean_rotor_cmd_diff    = successful_episodes_df["rotor_cmd_diff"].mean()
    mean_vehicle_cmd_diff  = successful_episodes_df["vehicle_cmd_diff"].mean()
    mean_trajectory_length = successful_episodes_df["trajectory_length"].mean()
    mean_goal_reached      = successful_episodes_df["max_goal_reached"].mean()
    mean_episode_time      = successful_episodes_df["episode_time"].mean()

    # append row to the summary csv, that shows total summary of the evaluation
    summary_df = summary_df.append({
        'episode': 'summary',
        'mean_state_cost': mean_state_cost,
        'min_calc_time': min_calc_time,
        'mean_calc_time': mean_calc_time,
        'max_calc_time': max_calc_time,
        'steer_cmd_diff': mean_steer_cmd_diff,
        'rotor_cmd_diff': mean_rotor_cmd_diff,
        'vehicle_cmd_diff': mean_vehicle_cmd_diff,
        'trajectory_length': mean_trajectory_length,
        'max_goal_reached': mean_goal_reached,
        'episode_time': mean_episode_time,
        'all_goals_reached': success_rate,
    }, ignore_index=True)
    summary_df.to_csv(os.path.join(RESULT_DIR, summary_csv_filename), index=False)

    # announce the evaluation is finished
    print("\n\n")
    print("#"*30)
    print(f'[INFO] All {TOTAL_NUM_EPISODES} episodes finished. The result is saved in {RESULT_DIR}')
    print("#"*30)
    print("\n\n")

if __name__ == '__main__':
    """ How to run this script
    `cd ~/nullspace_mpc/src/evaluation/mpc_nav_evaluator/scripts`
    `python3 run_evaluation.py`
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--agenda_yaml_path', type=str, default='agenda yaml file', help='config file for the evaluation')
    parser.add_argument('--controller', type=str, default='', help='controller name')
    parser.add_argument('--world_name', type=str, default='', help='world name to be used in the evaluation')
    args = parser.parse_args()

    # run evaluation
    run_evaluation(args.agenda_yaml_path, args.controller, args.world_name)
