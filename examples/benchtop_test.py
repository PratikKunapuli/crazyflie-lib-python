import time

from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

import pandas as pd
import numpy as np
import os


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7B3')
experiment_name = "benchtop_test_ctbr_setpoint_800"


command_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": [], "thrust": []}
motor_data = {"timestamp": [], "m1": [], "m2": [], "m3": [], "m4": []}
motor_rpm_data = {"timestamp": [], "m1": [], "m2": [], "m3": [], "m4": []}
control_target_att_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": [], "commander_roll": []}
gyro_data = {"timestamp": [], "x": [], "y": [], "z": []}
state_estimate_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": []}
state_estimate_dot_data = {"timestamp": [], "roll_rate": [], "pitch_rate": [], "yaw_rate": []}
controller_setpoint_att_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": []}
controller_setpoint_rate_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": []}
controller_cmd_att_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": []}
commander_data = {"timestamp": [], "received_roll": [], "output_roll": []}

rate_pid_outputs = {"timestamp": [], "roll_kp": [], "roll_kd": [], "roll_ki": [], "roll_kff": []}
# att_pid_outputs = {"timestamp": [], "roll": [], "pitch": [], "yaw": []}
# stabilizer_commander_setpoint_data = {"timestamp": [], "roll": [], "pitch": [], "yaw": []}


def send_setpoint(cf, duration, thrust, roll, pitch, yaw):
    """
    Duration - How long to repeat this command for
    Thrust - Thrust value to send to the Crazyflie (7000-60000) (clipped to 0xFFFF)
    Roll, Pitch, Yaw - Angles or Rate to send depending on firmware settings
    """
    # Set points must be sent continuously to the Crazyflie, if not it will think that connection is lost
    end_time = time.time() + duration
    while time.time() < end_time:
        command_data["timestamp"].append(time.time())
        command_data["roll"].append(roll)
        command_data["pitch"].append(pitch)
        command_data["yaw"].append(yaw)
        command_data["thrust"].append(thrust)
        cf.commander.send_setpoint(roll, pitch, yaw, int(thrust))
        time.sleep(0.01) # 100 Hz


def run_motor_step_sequence(scf):
    cf = scf.cf

    # Set to mellinger controller (2) 
    # cf.param.set_value('stabilizer.controller', '2')

    # Arm the Crazyflie
    print("Arming")
    cf.platform.send_arming_request(True)
    send_setpoint(cf, 0.2, 0, 0.0, 0.0, 0.0)
    time.sleep(0.5)

    print('Min Thrust')
    send_setpoint(cf, 2.0, 7000, 0.0, 0.0, 0.0)

    print('Max Thrust')
    send_setpoint(cf, 6.0, 60000, 0.0, 0.0, 0.0)

    print('Min Thrust')
    send_setpoint(cf, 2.0, 7000, 0.0, 0.0, 0.0)

    print("Stopping")
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    cf.platform.send_arming_request(False)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def test_hover(scf):
    cf = scf.cf

    # Set to mellinger controller (2) 
    # cf.param.set_value('stabilizer.controller', '2')

    # Arm the Crazyflie
    print("Arming (2 seconds)")
    cf.platform.send_arming_request(True)
    send_setpoint(cf, 0.01, 0, 0.0, 0.0, 0.0)
    send_setpoint(cf, 1.99, 30000, 0.0, 0.0, 0.0)

    print('Hover Thrust')
    send_setpoint(cf, 10.0, 30000, 0.0, 0.0, 0.0)

    print("Stopping")
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    cf.platform.send_arming_request(False)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def test_setpoint_ctbr(scf, roll_degrees=20.0):
    cf = scf.cf

    # Set to mellinger controller (2) 
    # cf.param.set_value('stabilizer.controller', '2')

    # Arm the Crazyflie
    print("Arming (instant)")
    cf.platform.send_arming_request(True)
    send_setpoint(cf, 0.01, 0, 0.0, 0.0, 0.0)
    send_setpoint(cf, 1.99, 30000, 0.0, 0.0, 0.0)

    print('Roll: ', roll_degrees)
    send_setpoint(cf, 5.0, 30000, roll_degrees, 0.0, 0.0)

    print('Roll: ', -roll_degrees)
    send_setpoint(cf, 5.0, 30000, -roll_degrees, 0.0, 0.0)

    # print('Stop', roll_degrees)
    # send_setpoint(cf, 2.0, 30000, 0.0, 0.0, 0.0)

    # print('Roll: -', roll_degrees)
    # send_setpoint(cf, 2.0, 30000, -roll_degrees, 0.0, 0.0)

    # print('Stop', roll_degrees)
    # send_setpoint(cf, 2.0, 30000, 0.0, 0.0, 0.0)

    print("Stopping")
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    cf.platform.send_arming_request(False)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def test_setpoint_ctatt(scf, roll_degrees=20.0):
    cf = scf.cf

    # Set to mellinger controller (2) 
    # cf.param.set_value('stabilizer.controller', '2')

    # Arm the Crazyflie
    print("Arming (2 seconds)")
    cf.platform.send_arming_request(True)
    send_setpoint(cf, 0.01, 0, 0.0, 0.0, 0.0)
    send_setpoint(cf, 1.99, 30000, 0.0, 0.0, 0.0)
    # time.sleep(2.0)

    print('Roll: ', roll_degrees)
    send_setpoint(cf, 5.0, 30000, roll_degrees, 0.0, 0.0)

    print('Roll: ', -roll_degrees)
    send_setpoint(cf, 5.0, 30000, -roll_degrees, 0.0, 0.0)

    print('Roll: ', roll_degrees)
    send_setpoint(cf, 5.0, 30000, roll_degrees, 0.0, 0.0)

    print('Roll: ', -roll_degrees)
    send_setpoint(cf, 5.0, 30000, -roll_degrees, 0.0, 0.0)

    print("Stopping")
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    cf.platform.send_arming_request(False)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def test_data_only(scf):
    cf = scf.cf

    # Set to mellinger controller (2) 
    # cf.param.set_value('stabilizer.controller', '2')

    # Arm the Crazyflie
    print("Arming")
    cf.platform.send_arming_request(True)
    send_setpoint(cf, 0.01, 0, 0.0, 0.0, 0.0)
    time.sleep(10.0)

    print("Stopping")
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    cf.platform.send_arming_request(False)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def test_sinusoid(scf):
    cf = scf.cf

    # Set to mellinger controller (2) 
    # cf.param.set_value('stabilizer.controller', '2')

    # Arm the Crazyflie
    print("Arming")
    cf.platform.send_arming_request(True)
    send_setpoint(cf, 0.01, 0, 0.0, 0.0, 0.0)
    send_setpoint(cf, 0.09, 30000, 0.0, 0.0, 0.0)
    time.sleep(0.1)

    # start_time = time.time()
    # duration = 10.0
    # while time.time() - start_time < duration:
    #     time_elapsed = time.time() - start_time
    #     roll_degrees = 1.0 * np.sin(2 * np.pi * time_elapsed * 10.0)
    #     print('Roll: ', roll_degrees)
    #     send_setpoint(cf, 0.01, 30000, roll_degrees, 0.0, 0.0)

    send_setpoint(cf, 10.0, 7000, 0.0, 0.0, 0.0)

    print("Stopping")
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    cf.platform.send_arming_request(False)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def _log_motor_data(timestamp, data, logconf):
    # print("time: ", timestamp, "m1: ", data["motor.m1"], "m2: ", data["motor.m2"], "m3: ", data["motor.m3"], "m4: ", data["motor.m4"])
    motor_data["timestamp"].append(timestamp)
    motor_data["m1"].append(data["motor.m1"])
    motor_data["m2"].append(data["motor.m2"])
    motor_data["m3"].append(data["motor.m3"])
    motor_data["m4"].append(data["motor.m4"])

def _log_motor_rpm_data(timestamp, data, logconf):
    motor_rpm_data["timestamp"].append(timestamp)
    motor_rpm_data["m1"].append(data["rpm.m1"])
    motor_rpm_data["m2"].append(data["rpm.m2"])
    motor_rpm_data["m3"].append(data["rpm.m3"])
    motor_rpm_data["m4"].append(data["rpm.m4"])

def _log_control_target_att_data(timestamp, data, logconf):
    control_target_att_data["timestamp"].append(timestamp)
    control_target_att_data["roll"].append(data["ctrltarget.roll"])
    control_target_att_data["pitch"].append(data["ctrltarget.pitch"])
    control_target_att_data["yaw"].append(data["ctrltarget.yaw"])
    control_target_att_data["commander_roll"].append(data["ctrltarget.commander_roll"])

def _log_gyro_data(timestamp, data, logconf):
    gyro_data["timestamp"].append(timestamp)
    gyro_data["x"].append(data["gyro.x"])
    gyro_data["y"].append(data["gyro.y"])
    gyro_data["z"].append(data["gyro.z"])

def _log_state_estimate_data(timestamp, data, logconf):
    state_estimate_data["timestamp"].append(timestamp)
    state_estimate_data["roll"].append(data["stateEstimate.roll"])
    state_estimate_data["pitch"].append(-data["stateEstimate.pitch"]) # pitch is inverted on CF
    state_estimate_data["yaw"].append(data["stateEstimate.yaw"])

def _log_state_estimate_dot_data(timestamp, data, logconf):
    # Data is a int16_t corresponding to millirads/s. We want degree/s as a float. 
    state_estimate_dot_data["timestamp"].append(timestamp)
    scaling_factor = (1/1000.0) * (180.0 / np.pi)
    state_estimate_dot_data["roll_rate"].append(data["stateEstimateZ.rateRoll"] * scaling_factor)
    state_estimate_dot_data["pitch_rate"].append(-data["stateEstimateZ.ratePitch"] * scaling_factor) # pitch is inverted on CF
    state_estimate_dot_data["yaw_rate"].append(data["stateEstimateZ.rateYaw"] * scaling_factor)

def _log_controller_setpoint_att_data(timestamp, data, logconf):
    controller_setpoint_att_data["timestamp"].append(timestamp)
    controller_setpoint_att_data["roll"].append(data["controller.roll"])
    controller_setpoint_att_data["pitch"].append(data["controller.pitch"])
    controller_setpoint_att_data["yaw"].append(data["controller.yaw"])

def _log_controller_setpoint_rate_data(timestamp, data, logconf):
    controller_setpoint_rate_data["timestamp"].append(timestamp)
    controller_setpoint_rate_data["roll"].append(data["controller.rollRate"])
    controller_setpoint_rate_data["pitch"].append(data["controller.pitchRate"])
    controller_setpoint_rate_data["yaw"].append(data["controller.yawRate"])

def _log_controller_cmd_att_data(timestamp, data, logconf):
    controller_cmd_att_data["timestamp"].append(timestamp)
    controller_cmd_att_data["roll"].append(data["controller.cmd_roll"])
    controller_cmd_att_data["pitch"].append(data["controller.cmd_pitch"])
    controller_cmd_att_data["yaw"].append(data["controller.cmd_yaw"])

def _log_commander_data(timestamp, data, logconf):
    commander_data["timestamp"].append(timestamp)
    commander_data["received_roll"].append(data["commander.received_roll"])
    commander_data["output_roll"].append(data["commander.output_roll"])

def _log_rate_pid_outputs(timestamp, data, logconf):
    rate_pid_outputs["timestamp"].append(timestamp)
    rate_pid_outputs["roll_kp"].append(data["pid_rate.roll_outP"])
    rate_pid_outputs["roll_kd"].append(data["pid_rate.roll_outD"])
    rate_pid_outputs["roll_ki"].append(data["pid_rate.roll_outI"])
    rate_pid_outputs["roll_kff"].append(data["pid_rate.roll_outFF"])

# def _log_stabilizer_commander_setpoint_data(timestamp, data, logconf):
#     stabilizer_commander_setpoint_data["timestamp"].append(timestamp)
#     stabilizer_commander_setpoint_data["roll"].append(data["ctrltarget.commander_roll"])
#     stabilizer_commander_setpoint_data["pitch"].append(data["ctrltarget.commander_pitch"])
#     stabilizer_commander_setpoint_data["yaw"].append(data["ctrltarget.commander_yaw"])

def set_up_logging(scf):
    # Set up logging
    _lg_motors = LogConfig(name='Motors', period_in_ms=10)
    _lg_motors.add_variable('motor.m1', 'uint16_t')
    _lg_motors.add_variable('motor.m2', 'uint16_t')
    _lg_motors.add_variable('motor.m3', 'uint16_t')
    _lg_motors.add_variable('motor.m4', 'uint16_t')

    # _lg_motors_rpm = LogConfig(name='Motors RPM', period_in_ms=10)
    # _lg_motors_rpm.add_variable('rpm.m1', 'uint16_t')
    # _lg_motors_rpm.add_variable('rpm.m2', 'uint16_t')
    # _lg_motors_rpm.add_variable('rpm.m3', 'uint16_t')
    # _lg_motors_rpm.add_variable('rpm.m4', 'uint16_t')

    _lg_ctrl_targets = LogConfig(name='Control Targets', period_in_ms=10)
    _lg_ctrl_targets.add_variable('ctrltarget.roll', 'float')
    _lg_ctrl_targets.add_variable('ctrltarget.pitch', 'float')
    _lg_ctrl_targets.add_variable('ctrltarget.yaw', 'float')
    _lg_ctrl_targets.add_variable('ctrltarget.commander_roll', 'float')

    _lg_gyro = LogConfig(name='Gyro', period_in_ms=10)
    _lg_gyro.add_variable('gyro.x', 'float')
    _lg_gyro.add_variable('gyro.y', 'float')
    _lg_gyro.add_variable('gyro.z', 'float')

    _lg_state_estimate = LogConfig(name='State Estimate', period_in_ms=10)
    _lg_state_estimate.add_variable('stateEstimate.roll', 'float')
    _lg_state_estimate.add_variable('stateEstimate.pitch', 'float')
    _lg_state_estimate.add_variable('stateEstimate.yaw', 'float')

    _lg_state_estimate_dot = LogConfig(name='State Estimate Dot', period_in_ms=10)
    _lg_state_estimate_dot.add_variable('stateEstimateZ.rateRoll', 'int16_t')
    _lg_state_estimate_dot.add_variable('stateEstimateZ.ratePitch', 'int16_t')
    _lg_state_estimate_dot.add_variable('stateEstimateZ.rateYaw', 'int16_t')

    _lg_controller_setpoint_att = LogConfig(name='Controller Setpoint Attitude', period_in_ms=10)
    _lg_controller_setpoint_att.add_variable('controller.roll', 'float')
    _lg_controller_setpoint_att.add_variable('controller.pitch', 'float')
    _lg_controller_setpoint_att.add_variable('controller.yaw', 'float')

    _lg_controller_setpoint_rate = LogConfig(name='Controller Setpoint Rate', period_in_ms=10)
    _lg_controller_setpoint_rate.add_variable('controller.rollRate', 'float')
    _lg_controller_setpoint_rate.add_variable('controller.pitchRate', 'float')
    _lg_controller_setpoint_rate.add_variable('controller.yawRate', 'float')

    _lg_controller_cmd_att = LogConfig(name='Controller Command Attitude', period_in_ms=10)
    _lg_controller_cmd_att.add_variable('controller.cmd_roll', 'float')
    _lg_controller_cmd_att.add_variable('controller.cmd_pitch', 'float')
    _lg_controller_cmd_att.add_variable('controller.cmd_yaw', 'float')

    _lg_commander_roll = LogConfig(name='Commander Roll', period_in_ms=10)
    _lg_commander_roll.add_variable('commander.received_roll', 'float')
    _lg_commander_roll.add_variable('commander.output_roll', 'float')

    _lg_rate_pid_outputs = LogConfig(name='Rate PID Outputs', period_in_ms=10)
    _lg_rate_pid_outputs.add_variable('pid_rate.roll_outP', 'float')
    _lg_rate_pid_outputs.add_variable('pid_rate.roll_outD', 'float')
    _lg_rate_pid_outputs.add_variable('pid_rate.roll_outI', 'float')
    _lg_rate_pid_outputs.add_variable('pid_rate.roll_outFF', 'float')


    # _lg_stabilizer_commander_setpoint = LogConfig(name='Stabilizer Commander Setpoint', period_in_ms=10)
    # _lg_stabilizer_commander_setpoint.add_variable('ctrltarget.commander_roll', 'float')
    # _lg_stabilizer_commander_setpoint.add_variable('ctrltarget.commander_pitch', 'float')
    # _lg_stabilizer_commander_setpoint.add_variable('ctrltarget.commander_yaw', 'float')


    # Add the callback to the log config
    scf.cf.log.add_config(_lg_motors)
    _lg_motors.data_received_cb.add_callback(_log_motor_data)
    # scf.cf.log.add_config(_lg_motors_rpm)
    # _lg_motors_rpm.data_received_cb.add_callback(_log_motor_rpm_data)
    scf.cf.log.add_config(_lg_ctrl_targets)
    _lg_ctrl_targets.data_received_cb.add_callback(_log_control_target_att_data)
    scf.cf.log.add_config(_lg_gyro)
    _lg_gyro.data_received_cb.add_callback(_log_gyro_data)
    scf.cf.log.add_config(_lg_state_estimate)
    _lg_state_estimate.data_received_cb.add_callback(_log_state_estimate_data)
    scf.cf.log.add_config(_lg_state_estimate_dot)
    _lg_state_estimate_dot.data_received_cb.add_callback(_log_state_estimate_dot_data)
    scf.cf.log.add_config(_lg_controller_setpoint_att)
    _lg_controller_setpoint_att.data_received_cb.add_callback(_log_controller_setpoint_att_data)
    scf.cf.log.add_config(_lg_controller_setpoint_rate)
    _lg_controller_setpoint_rate.data_received_cb.add_callback(_log_controller_setpoint_rate_data)
    scf.cf.log.add_config(_lg_controller_cmd_att)
    _lg_controller_cmd_att.data_received_cb.add_callback(_log_controller_cmd_att_data)
    # scf.cf.log.add_config(_lg_stabilizer_commander_setpoint)
    # _lg_stabilizer_commander_setpoint.data_received_cb.add_callback(_log_stabilizer_commander_setpoint_data)
    scf.cf.log.add_config(_lg_commander_roll)
    _lg_commander_roll.data_received_cb.add_callback(_log_commander_data)
    scf.cf.log.add_config(_lg_rate_pid_outputs)
    _lg_rate_pid_outputs.data_received_cb.add_callback(_log_rate_pid_outputs)



    # Start logging
    _lg_motors.start()
    # _lg_motors_rpm.start() # This is always 0
    _lg_ctrl_targets.start()
    _lg_gyro.start()
    _lg_state_estimate.start()
    _lg_state_estimate_dot.start()
    _lg_controller_setpoint_att.start()
    _lg_controller_setpoint_rate.start()
    _lg_controller_cmd_att.start()
    # _lg_stabilizer_commander_setpoint.start()
    _lg_commander_roll.start()
    _lg_rate_pid_outputs.start()

def save_all_data():
    current_datetime = time.strftime("%Y-%m-%d_%H-%M-%S")
    save_folder = os.path.join(experiment_name, current_datetime)
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    df = pd.DataFrame(motor_data)
    df.to_csv(os.path.join(save_folder, "motor_data.csv"), index=False)
    df = pd.DataFrame(motor_rpm_data)
    df.to_csv(os.path.join(save_folder, "motor_rpm_data.csv"), index=False)
    df = pd.DataFrame(control_target_att_data)
    df.to_csv(os.path.join(save_folder, "control_target_att_data.csv"), index=False)
    df = pd.DataFrame(gyro_data)
    df.to_csv(os.path.join(save_folder, "gyro_data.csv"), index=False)
    df = pd.DataFrame(state_estimate_data)
    df.to_csv(os.path.join(save_folder, "state_estimate_data.csv"), index=False)
    df = pd.DataFrame(state_estimate_dot_data)
    df.to_csv(os.path.join(save_folder, "state_estimate_dot_data.csv"), index=False)
    df = pd.DataFrame(command_data)
    df.to_csv(os.path.join(save_folder, "command_data.csv"), index=False)
    df = pd.DataFrame(controller_setpoint_att_data)
    df.to_csv(os.path.join(save_folder, "controller_setpoint_att_data.csv"), index=False)
    df = pd.DataFrame(controller_setpoint_rate_data)
    df.to_csv(os.path.join(save_folder, "controller_setpoint_rate_data.csv"), index=False)
    df = pd.DataFrame(controller_cmd_att_data)
    df.to_csv(os.path.join(save_folder, "controller_cmd_att_data.csv"), index=False)
    # df = pd.DataFrame(stabilizer_commander_setpoint_data)
    # df.to_csv(os.path.join(save_folder, "stabilizer_commander_setpoint_data.csv"), index=False)
    df = pd.DataFrame(commander_data)
    df.to_csv(os.path.join(save_folder, "commander_data.csv"), index=False)
    df = pd.DataFrame(rate_pid_outputs)
    df.to_csv(os.path.join(save_folder, "rate_pid_outputs.csv"), index=False)

def set_cf_params(scf):
    cf = scf.cf
    cf.commander.set_client_xmode(False)

    cf.param.set_value('stabilizer.controller', '1') # PID Controller - 1, Mellinger Controller - 2
    cf.param.set_value('supervisor.tmblChckEn', '0') # Disable tumble check
    # cf.param.set_value('imu_sensors.imuPsi', '45') # IMU Correction

    # Set PID Gains (CTBR MODE)
    cf.param.set_value('pid_rate.roll_kp', '-800')
    cf.param.set_value('pid_rate.roll_kd', '-20.0')
    cf.param.set_value('pid_rate.roll_ki', '0.0')
    cf.param.set_value('pid_rate.roll_kff', '0.0')

    yaw_mode = cf.param.get_value('flightmode.yawMode')
    print("Yaw Mode: ", yaw_mode)



if __name__ == '__main__':
    print("Initializing drivers...")
    cflib.crtp.init_drivers()

    print("Starting sequence...")    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        set_cf_params(scf)
        set_up_logging(scf)

        # run_motor_step_sequence(scf)
        # test_hover(scf)
        test_setpoint_ctbr(scf, 800.0)
        # test_setpoint_ctatt(scf, 60.0)
        # test_data_only(scf)
        # test_sinusoid(scf)
    
    save_all_data()