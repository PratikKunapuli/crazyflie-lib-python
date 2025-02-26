import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def plot_rates(experiment_name, gyro_df, state_dot_df, controller_setpoint_rate_df, controller_cmd_att_df, command_df):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(gyro_df["timestamp"], gyro_df["x"], label="Gyro x")
    plt.plot(state_dot_df["timestamp"], state_dot_df["roll_rate"], label="State Estimate Roll Rate")
    plt.plot(controller_setpoint_rate_df["timestamp"], controller_setpoint_rate_df["roll"], label="Controller Setpoint Roll Rate")
    if "ctbr" in experiment_name:
        plt.scatter(command_df["timestamp"], command_df["roll"], label="Cmd Roll Rate", marker=".", color="green")
        # plt.scatter(controller_cmd_att_df["timestamp"], controller_cmd_att_df["roll"], label="Controller Cmd Roll Rate", marker=".", color="red")
    plt.legend()
    plt.ylabel("Roll Rate [deg/s]")
    plt.subplot(3, 1, 2)
    plt.plot(gyro_df["timestamp"], gyro_df["y"], label="Gyro y")
    plt.plot(state_dot_df["timestamp"], state_dot_df["pitch_rate"], label="State Estimate Pitch Rate")
    plt.plot(controller_setpoint_rate_df["timestamp"], controller_setpoint_rate_df["pitch"], label="Controller Setpoint Pitch Rate")
    if "ctbr" in experiment_name:
        plt.scatter(command_df["timestamp"], command_df["pitch"], label="Cmd Pitch Rate", marker=".", color="green")
        # plt.scatter(controller_cmd_att_df["timestamp"], controller_cmd_att_df["pitch"], label="Controller Cmd Pitch Rate", marker=".", color="red")
    plt.legend()
    plt.ylabel("Pitch Rate [deg/s]")
    plt.subplot(3, 1, 3)
    plt.plot(gyro_df["timestamp"], gyro_df["z"], label="Gyro z")
    plt.plot(state_dot_df["timestamp"], state_dot_df["yaw_rate"], label="State Estimate Yaw Rate")
    plt.plot(controller_setpoint_rate_df["timestamp"], controller_setpoint_rate_df["yaw"], label="Controller Setpoint Yaw Rate")
    if "ctbr" in experiment_name:
        plt.scatter(command_df["timestamp"], command_df["yaw"], label="Cmd Yaw Rate", marker=".", color="green")
        # plt.scatter(controller_cmd_att_df["timestamp"], controller_cmd_att_df["yaw"], label="Controller Cmd Yaw Rate", marker=".", color="red")
    plt.legend()
    plt.ylabel("Yaw Rate [deg/s]")
    plt.xlabel("Time (s)")
    plt.savefig(os.path.join(experiment_name, "rate_data.png"))

def plot_attitudes(experiment_name, state_estimate_df, control_target_att_df, controller_setpoint_att_df, controller_cmd_att_df, commander_roll_df, command_df):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(state_estimate_df["timestamp"], state_estimate_df["roll"], label="State Estimate Roll")
    plt.plot(control_target_att_df["timestamp"], control_target_att_df["roll"], label="Control Target Roll")
    plt.plot(controller_setpoint_att_df["timestamp"], controller_setpoint_att_df["roll"], label="Controller Setpoint Roll")
    # if "ctbr" not in experiment_name:
    #     plt.scatter(command_df["timestamp"], command_df["roll"], label="Cmd Roll", marker=".", color="green")
    #     plt.scatter(control_target_att_df["timestamp"], control_target_att_df["commander_roll"], label="Commander Roll", marker=".", color="blue")
        # plt.scatter(commander_roll_data["timestamp"], commander_roll_data["received_roll"], label="Commander Roll Received", marker=".", color="red")
        # plt.scatter(commander_roll_data["timestamp"], commander_roll_data["output_roll"], label="Commander Roll Output", marker=".", color="green")
        # plt.scatter(controller_cmd_att_df["timestamp"], controller_cmd_att_df["roll"], label="Controller Cmd Roll", marker=".", color="red")
    plt.legend()
    plt.ylabel("Roll [deg]")
    plt.subplot(3, 1, 2)
    plt.plot(state_estimate_df["timestamp"], state_estimate_df["pitch"], label="State Estimate Pitch")
    plt.plot(control_target_att_df["timestamp"], control_target_att_df["pitch"], label="Control Target Pitch")
    plt.plot(controller_setpoint_att_df["timestamp"], controller_setpoint_att_df["pitch"], label="Controller Setpoint Pitch")
    if "ctbr" not in experiment_name:
        plt.scatter(command_df["timestamp"], command_df["pitch"], label="Cmd Pitch", marker=".", color="green")
        # plt.scatter(controller_cmd_att_df["timestamp"], controller_cmd_att_df["pitch"], label="Controller Cmd Pitch", marker=".", color="red")
    plt.legend()
    plt.ylabel("Pitch [deg]")
    plt.subplot(3, 1, 3)
    plt.plot(state_estimate_df["timestamp"], state_estimate_df["yaw"], label="State Estimate Yaw")
    plt.plot(control_target_att_df["timestamp"], control_target_att_df["yaw"], label="Control Target Yaw")
    plt.plot(controller_setpoint_att_df["timestamp"], controller_setpoint_att_df["yaw"], label="Controller Setpoint Yaw")
    if "ctbr" not in experiment_name:
        plt.scatter(command_df["timestamp"], command_df["yaw"], label="Cmd Yaw", marker=".", color="green")
        # plt.scatter(controller_cmd_att_df["timestamp"], controller_cmd_att_df["yaw"], label="Controller Cmd Yaw", marker=".", color="red")
    plt.legend()
    plt.ylabel("Yaw [deg]")
    plt.xlabel("Time (s)")
    plt.savefig(os.path.join(experiment_name, "attitude_data.png"))
    
def plot_motors(experiment_name, motor_df, command_df):
    plt.figure()
    plt.plot(motor_df["timestamp"], motor_df["m1"], label="Motor 1")
    plt.plot(motor_df["timestamp"], motor_df["m2"], label="Motor 2")
    plt.plot(motor_df["timestamp"], motor_df["m3"], label="Motor 3")
    plt.plot(motor_df["timestamp"], motor_df["m4"], label="Motor 4")
    plt.plot(command_df["timestamp"], command_df["thrust"], label="Cmd Thrust", linestyle="--")
    plt.ylabel("PWM")
    plt.legend(loc="best")
    plt.xlabel("Time (s)")
    plt.savefig(os.path.join(experiment_name, "motor_data.png"))

def plot_roll_setpoints(experiment_name, command_df, controller_setpoint_att_df, controller_cmd_att_df, commander_roll_df):
    plt.figure()
    plt.plot(command_df["timestamp"], command_df["roll"], label="Cmd Roll")
    plt.plot(controller_setpoint_att_df["timestamp"], controller_setpoint_att_df["roll"], label="Controller Setpoint Roll")
    plt.plot(controller_cmd_att_df["timestamp"], controller_cmd_att_df["roll"]/65535.0, label="Controller Cmd Roll")
    plt.plot(commander_roll_df["timestamp"], commander_roll_df["received_roll"], label="Commander Roll Received")
    # plt.plot(commander_roll_df["timestamp"], commander_roll_df["output_roll"], label="Commander Roll Output")
    plt.legend()
    plt.ylabel("Roll [deg]")
    plt.xlabel("Time (s)")
    plt.savefig(os.path.join(experiment_name, "roll_setpoint_data.png"))

def plot_rate_PID_outputs(experiment_name, pid_rate_output_df, gyro_df, state_dot_df, controller_setpoint_rate_df, command_df):
    pid_rate_output_data["roll_output"] = pid_rate_output_data["roll_kp"] + pid_rate_output_data["roll_ki"] + pid_rate_output_data["roll_kd"] + pid_rate_output_data["roll_kff"]
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(gyro_df["timestamp"], gyro_df["x"], label="Gyro x")
    plt.plot(state_dot_df["timestamp"], state_dot_df["roll_rate"], label="State Estimate Roll Rate")
    plt.plot(controller_setpoint_rate_df["timestamp"], controller_setpoint_rate_df["roll"], label="Controller Setpoint Roll Rate")
    # if "ctbr" in experiment_name:
    #     plt.scatter(command_df["timestamp"], command_df["roll"], label="Cmd Roll Rate", marker=".", color="green")
    plt.legend(loc="best")
    plt.ylabel("Roll Rate [deg/s]")

    plt.subplot(2,1,2)
    plt.plot(pid_rate_output_df["timestamp"], pid_rate_output_df["roll_kp"], label="kP Out")
    plt.plot(pid_rate_output_df["timestamp"], pid_rate_output_df["roll_ki"], label="kI Out")
    plt.plot(pid_rate_output_df["timestamp"], pid_rate_output_df["roll_kd"], label="kD Out")
    plt.plot(pid_rate_output_df["timestamp"], pid_rate_output_df["roll_kff"], label="kFF Out")
    plt.plot(pid_rate_output_data["timestamp"], pid_rate_output_data["roll_output"], label="PID Output")
    plt.legend(loc="best")
    plt.ylabel("PID Outputs")
    plt.xlabel("Time (s)")
    plt.savefig(os.path.join(experiment_name, "rate_pid_outputs.png"))



# First argument is the experiment name
if __name__ == "__main__":
    experiment_name = sys.argv[1]

    if len(sys.argv) > 2:
        experiment_datetime = sys.argv[2]
    else:
        # get the latest experiment in the experiement name folder
        experiment_datetime = sorted(os.listdir(experiment_name))[-1]

    experiment_name = os.path.join(experiment_name, experiment_datetime)
    # Load the data
    gyro_data = pd.read_csv(os.path.join(experiment_name, "gyro_data.csv"))
    state_estimate_data = pd.read_csv(os.path.join(experiment_name, "state_estimate_data.csv"))
    state_estimate_dot_data = pd.read_csv(os.path.join(experiment_name, "state_estimate_dot_data.csv"))
    motor_data = pd.read_csv(os.path.join(experiment_name, "motor_data.csv"))
    # motor_rpm_data = pd.read_csv(os.path.join(experiment_name, "motor_rpm_data.csv")) # always 0
    control_target_att_data = pd.read_csv(os.path.join(experiment_name, "control_target_att_data.csv"))
    command_data = pd.read_csv(os.path.join(experiment_name, "command_data.csv"))
    controller_setpoint_att_data = pd.read_csv(os.path.join(experiment_name, "controller_setpoint_att_data.csv"))
    controller_setpoint_rate_data = pd.read_csv(os.path.join(experiment_name, "controller_setpoint_rate_data.csv"))
    controller_cmd_att_data = pd.read_csv(os.path.join(experiment_name, "controller_cmd_att_data.csv"))
    commander_roll_data = pd.read_csv(os.path.join(experiment_name, "commander_data.csv"))
    pid_rate_output_data = pd.read_csv(os.path.join(experiment_name, "rate_pid_outputs.csv"))

    # Normalize timestamps to start at 0 which is the earliest timestamp logged
    earliest_timestamp = min(
        gyro_data["timestamp"].min(), 
        state_estimate_dot_data["timestamp"].min(), 
        motor_data["timestamp"].min(), 
        state_estimate_data["timestamp"].min(), 
        control_target_att_data["timestamp"].min(),
        controller_setpoint_att_data["timestamp"].min(),
        controller_setpoint_rate_data["timestamp"].min(),
        command_data["timestamp"].min(),
        commander_roll_data["timestamp"].min(),
        pid_rate_output_data["timestamp"].min(),
    )
    gyro_data["timestamp"] -= earliest_timestamp
    gyro_data["timestamp"] /= 1000.
    state_estimate_dot_data["timestamp"] -= earliest_timestamp
    state_estimate_dot_data["timestamp"] /= 1000.
    state_estimate_data["timestamp"] -= earliest_timestamp
    state_estimate_data["timestamp"] /= 1000.
    motor_data["timestamp"] -= earliest_timestamp
    motor_data["timestamp"] /= 1000
    control_target_att_data["timestamp"] -= earliest_timestamp
    control_target_att_data["timestamp"] /= 1000.
    # motor_rpm_data["timestamp"] -= earliest_timestamp
    # motor_rpm_data["timestamp"] /= 1000.
    controller_setpoint_att_data["timestamp"] -= earliest_timestamp
    controller_setpoint_att_data["timestamp"] /= 1000.
    controller_setpoint_rate_data["timestamp"] -= earliest_timestamp
    controller_setpoint_rate_data["timestamp"] /= 1000.
    controller_cmd_att_data["timestamp"] -= earliest_timestamp
    controller_cmd_att_data["timestamp"] /= 1000.
    commander_roll_data["timestamp"] -= earliest_timestamp
    commander_roll_data["timestamp"] /= 1000.
    pid_rate_output_data["timestamp"] -= earliest_timestamp
    pid_rate_output_data["timestamp"] /= 1000.

    command_data["timestamp"] -= min(command_data["timestamp"])

    # Plot the data
    plot_rates(experiment_name, gyro_data, state_estimate_dot_data, controller_setpoint_rate_data, controller_cmd_att_data, command_data)
    plot_attitudes(experiment_name, state_estimate_data, control_target_att_data, controller_setpoint_att_data, controller_cmd_att_data, commander_roll_data, command_data)
    plot_motors(experiment_name, motor_data, command_data)
    plot_roll_setpoints(experiment_name, command_data, controller_setpoint_att_data, controller_cmd_att_data, commander_roll_data)
    plot_rate_PID_outputs(experiment_name, pid_rate_output_data, gyro_data, state_estimate_dot_data, controller_setpoint_rate_data, command_data)
    plt.show()
