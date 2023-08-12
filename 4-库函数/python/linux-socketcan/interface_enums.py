# -*- coding=utf-8 -*-
# Axis.AxisState
AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_STARTUP_SEQUENCE = 2
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_SENSORLESS_CONTROL = 5
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_LOCKIN_SPIN = 9
AXIS_STATE_ENCODER_DIR_FIND = 10
AXIS_STATE_HOMING = 11

# Controller.ControlMode
CONTROL_MODE_VOLTAGE_CONTROL = 0
CONTROL_MODE_TORQUE_CONTROL = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
CONTROL_MODE_IMPEDANCE_CONTROL = 4

# Controller.InputMode
INPUT_MODE_INACTIVE = 0
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2
INPUT_MODE_POS_FILTER = 3
INPUT_MODE_MIX_CHANNELS = 4
INPUT_MODE_TRAP_TRAJ = 5
INPUT_MODE_TORQUE_RAMP = 6
INPUT_MODE_MIRROR = 7

# Motor.MotorType
MOTOR_TYPE_HIGH_CURRENT = 0
MOTOR_TYPE_GIMBAL = 2
MOTOR_TYPE_ACIM = 3

# Encoder.Mode
ENCODER_MODE_INCREMENTAL = 0
ENCODER_MODE_HALL = 1
ENCODER_MODE_SINCOS = 2
ENCODER_MODE_SPI_ABS_CUI = 256
ENCODER_MODE_SPI_ABS_AMS = 257
ENCODER_MODE_SPI_ABS_AEAT = 258
ENCODER_MODE_SPI_ABS_RLS = 259
ENCODER_MODE_SPI_ABS_EDR = 271

# Axis.Error
AXIS_ERROR_NONE = 0x00000000
AXIS_ERROR_INVALID_STATE = 0x00000001
AXIS_ERROR_DC_BUS_UNDER_VOLTAGE = 0x00000002
AXIS_ERROR_DC_BUS_OVER_VOLTAGE = 0x00000004
AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x00000008
AXIS_ERROR_BRAKE_RESISTOR_DISARMED = 0x00000010
AXIS_ERROR_MOTOR_DISARMED = 0x00000020
AXIS_ERROR_MOTOR_FAILED = 0x00000040
AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080
AXIS_ERROR_ENCODER_FAILED = 0x00000100
AXIS_ERROR_CONTROLLER_FAILED = 0x00000200
AXIS_ERROR_POS_CTRL_DURING_SENSORLESS = 0x00000400
AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000800
AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x00001000
AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x00002000
AXIS_ERROR_ESTOP_REQUESTED = 0x00004000
AXIS_ERROR_UNKOWN_ERROR_CODE1 = 0x00008000
AXIS_ERROR_UNKOWN_ERROR_CODE2 = 0x00010000
AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x00020000
AXIS_ERROR_OVER_TEMP = 0x00040000

# Motor.Error
MOTOR_ERROR_NONE = 0x00000000
MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001
MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002
MOTOR_ERROR_ADC_FAILED = 0x00000004
MOTOR_ERROR_DRV_FAULT = 0x00000008
MOTOR_ERROR_CONTROL_DEADLINE_MISSED = 0x00000010
MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x00000020
MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x00000040
MOTOR_ERROR_MODULATION_MAGNITUDE = 0x00000080
MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION = 0x00000100
MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK = 0x00000200
MOTOR_ERROR_CURRENT_SENSE_SATURATION = 0x00000400
MOTOR_ERROR_UNKOWN_ERROR_CODE = 0x00000800
MOTOR_ERROR_CURRENT_LIMIT_VIOLATION = 0x00001000
MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN = 0x00002000
MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT = 0x00004000
MOTOR_ERROR_DC_BUS_OVER_CURRENT = 0x00008000
MOTOR_ERROR_STALL_LIMIT_VIOLATION = 0x00010000

# Encoder.Error
ENCODER_ERROR_NONE = 0x00000000
ENCODER_ERROR_UNSTABLE_GAIN = 0x00000001
ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH = 0x00000002
ENCODER_ERROR_NO_RESPONSE = 0x00000004
ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE = 0x00000008
ENCODER_ERROR_ILLEGAL_HALL_STATE = 0x00000010
ENCODER_ERROR_INDEX_NOT_FOUND_YET = 0x00000020
ENCODER_ERROR_ABS_SPI_TIMEOUT = 0x00000040
ENCODER_ERROR_ABS_SPI_COM_FAIL = 0x00000080
ENCODER_ERROR_ABS_SPI_NOT_READY = 0x00000100
ENCODER_ERROR_EDR_OVER_RANGE = 0x00000200
ENCODER_ERROR_EDR_POWER_FAILURE = 0x00000400
ERROR_EDR_BATTERY_LOW = 0x00000800
ERROR_OFFSET_CALIBRATE_FAILURE = 0x00001000
ERROR_INCORRECT_OFFSET = 0x00002000
ERROR_TURNS_COUNTER_WARNING = 0x00004000

# Controller.Error
CONTROLLER_ERROR_NONE = 0x00000000
CONTROLLER_ERROR_OVERSPEED = 0x00000001
CONTROLLER_ERROR_INVALID_INPUT_MODE = 0x00000002
CONTROLLER_ERROR_UNSTABLE_GAIN = 0x00000004
CONTROLLER_ERROR_INVALID_MIRROR_AXIS = 0x00000008
CONTROLLER_ERROR_INVALID_LOAD_ENCODER = 0x00000010
CONTROLLER_ERROR_INVALID_ESTIMATE = 0x00000020

# Can.Error
CAN_ERROR_NONE = 0x00000000
CAN_ERROR_DUPLICATE_CAN_IDS = 0x00000001

# Fet_thermistor.Error
THERMISTOR_CURRENT_LIMITER_ERROR_NONE = 0x00000000
THERMISTOR_CURRENT_LIMITER_ERROR_OVER_TEMP = 0x00000001

# Motor_thermistor.Error
MOTOR_CURRENT_LIMITER_ERROR_NONE = 0x00000000
MOTOR_CURRENT_LIMITER_ERROR_OVER_TEMP = 0x00000001

N = 1


def plus_plus(n=0):
    global N
    if n == 0:
        N = 1
    else:
        N = N + 1
    return N


property_addresss = {
    #     #
    'vbus_voltage': 00000 + plus_plus(0),
    'ibus': 00000 + plus_plus(1),
    'serial_number_l': 00000 + plus_plus(1),
    'serial_number_h': 00000 + plus_plus(1),
    'hw_version': 00000 + plus_plus(1),
    'fw_version': 00000 + plus_plus(1),
    # config. #
    'config.uart_baudrate': 10000 + plus_plus(0),
    'config.dc_bus_undervoltage_trip_level': 10000 + plus_plus(1),
    'config.dc_bus_overvoltage_trip_level': 10000 + plus_plus(1),
    'config.dc_max_positive_current': 10000 + plus_plus(1),
    'config.dc_max_negative_current': 10000 + plus_plus(1),
    'config.max_regen_current': 10000 + plus_plus(1),
    'config.brake_resistance': 10000 + plus_plus(1),
    'config.enable_uart': 10000 + plus_plus(1),
    # can. #
    'can.error': 20000 + plus_plus(0),
    # can.config. #
    'can.config.baud_rate': 21000 + plus_plus(0),
    'can.config.protocol': 21000 + plus_plus(1),
    # axis0. #
    'axis0.error': 30000 + plus_plus(0),
    'axis0.current_state': 30000 + plus_plus(1),
    'axis0.requested_state': 30000 + plus_plus(1),
    # axis0.config. #
    'axis0.config.can_node_id': 31000 + plus_plus(0),
    'axis0.config.startup_motor_calibration': 31000 + plus_plus(1),
    'axis0.config.startup_encoder_index_search': 31000 + plus_plus(1),
    'axis0.config.startup_encoder_offset_calibration': 31000 + plus_plus(1),
    'axis0.config.startup_closed_loop_control': 31000 + plus_plus(1),
    'axis0.config.enable_step_dir': 31000 + plus_plus(1),
    'axis0.config.turns_per_step': 31000 + plus_plus(1),
    # axis0.config.calibration_lockin. #
    'axis0.config.calibration_lockin.current': 31100 + plus_plus(0),
    'axis0.config.calibration_lockin.ramp_time': 31100 + plus_plus(1),
    'axis0.config.calibration_lockin.ramp_distance': 31100 + plus_plus(1),
    'axis0.config.calibration_lockin.accel': 31100 + plus_plus(1),
    'axis0.config.calibration_lockin.vel': 31100 + plus_plus(1),
    # axis0.config.extra_setting. #
    'axis0.config.extra_setting.enable_120r': 31200 + plus_plus(0),
    'axis0.config.extra_setting.enable_circular_setpoint_limit': 31200 + plus_plus(1),
    'axis0.config.extra_setting.circular_setpoint_min': 31200 + plus_plus(1),
    'axis0.config.extra_setting.circular_setpoint_max': 31200 + plus_plus(1),
    'axis0.config.extra_setting.zero_offset_counts': 31200 + plus_plus(1),
    'axis0.config.extra_setting.stored_turns_counts': 31200 + plus_plus(1),
    'axis0.config.extra_setting.enable_multi_circle': 31200 + plus_plus(1),
    'axis0.config.extra_setting.gear_ratio': 31200 + plus_plus(1),
    'axis0.config.extra_setting.stall_current_limit': 31200 + plus_plus(1),
    'axis0.config.extra_setting.stall_time_limit': 31200 + plus_plus(1),
    'axis0.config.extra_setting.enable_stall_limit': 31200 + plus_plus(1),
    'axis0.config.extra_setting.enable_adaptive_gain': 31200 + plus_plus(1),
    'axis0.config.extra_setting.encoder_offset_calibrated': 31200 + plus_plus(1),
    'axis0.config.extra_setting.enable_reply_state': 31200 + plus_plus(1),
    'axis0.config.extra_setting.enable_damp_idle': 31200 + plus_plus(1),
    # axis0.controller. #
    'axis0.controller.error': 32000 + plus_plus(0),
    'axis0.controller.input_pos': 32000 + plus_plus(1),
    'axis0.controller.input_vel': 32000 + plus_plus(1),
    'axis0.controller.input_torque': 32000 + plus_plus(1),
    'axis0.controller.pos_setpoint': 32000 + plus_plus(1),
    'axis0.controller.vel_setpoint': 32000 + plus_plus(1),
    'axis0.controller.torque_setpoint': 32000 + plus_plus(1),
    'axis0.controller.trajectory_done': 32000 + plus_plus(1),
    # axis0.controller.config. #
    'axis0.controller.config.enable_gain_scheduling': 32100 + plus_plus(0),
    'axis0.controller.config.enable_vel_limit': 32100 + plus_plus(1),
    'axis0.controller.config.enable_current_mode_vel_limit': 32100 + plus_plus(1),
    'axis0.controller.config.enable_overspeed_error': 32100 + plus_plus(1),
    'axis0.controller.config.control_mode': 32100 + plus_plus(1),
    'axis0.controller.config.input_mode': 32100 + plus_plus(1),
    'axis0.controller.config.gain_scheduling_width': 32100 + plus_plus(1),
    'axis0.controller.config.pos_gain': 32100 + plus_plus(1),
    'axis0.controller.config.vel_gain': 32100 + plus_plus(1),
    'axis0.controller.config.vel_integrator_gain': 32100 + plus_plus(1),
    'axis0.controller.config.vel_limit': 32100 + plus_plus(1),
    'axis0.controller.config.vel_limit_tolerance': 32100 + plus_plus(1),
    'axis0.controller.config.vel_ramp_rate': 32100 + plus_plus(1),
    'axis0.controller.config.torque_ramp_rate': 32100 + plus_plus(1),
    'axis0.controller.config.circular_setpoints': 32100 + plus_plus(1),
    'axis0.controller.config.circular_setpoint_range': 32100 + plus_plus(1),
    'axis0.controller.config.inertia': 32100 + plus_plus(1),
    'axis0.controller.config.input_filter_bandwidth': 32100 + plus_plus(1),
    # axis0.motor. #
    'axis0.motor.error': 33000 + plus_plus(0),
    'axis0.motor.is_calibrated': 33000 + plus_plus(1),
    'axis0.motor.effective_current_lim': 33000 + plus_plus(1),
    # axis0.motor.config. #
    'axis0.motor.config.pre_calibrated': 33100 + plus_plus(0),
    'axis0.motor.config.pole_pairs': 33100 + plus_plus(1),
    'axis0.motor.config.calibration_current': 33100 + plus_plus(1),
    'axis0.motor.config.resistance_calib_max_voltage': 33100 + plus_plus(1),
    'axis0.motor.config.phase_inductance': 33100 + plus_plus(1),
    'axis0.motor.config.phase_resistance': 33100 + plus_plus(1),
    'axis0.motor.config.torque_constant': 33100 + plus_plus(1),
    'axis0.motor.config.direction': 33100 + plus_plus(1),
    'axis0.motor.config.motor_type': 33100 + plus_plus(1),
    'axis0.motor.config.current_lim': 33100 + plus_plus(1),
    'axis0.motor.config.current_lim_margin': 33100 + plus_plus(1),
    'axis0.motor.config.torque_lim': 33100 + plus_plus(1),
    'axis0.motor.config.requested_current_range': 33100 + plus_plus(1),
    'axis0.motor.config.current_control_bandwidth': 33100 + plus_plus(1),
    # axis.motor.current_control. #
    'axis0.motor.current_control.Ibus': 33200 + plus_plus(0),
    'axis0.motor.current_control.final_v_alpha': 33200 + plus_plus(1),
    'axis0.motor.current_control.final_v_beta': 33200 + plus_plus(1),
    'axis0.motor.current_control.Id_setpoint': 33200 + plus_plus(1),
    'axis0.motor.current_control.Iq_setpoint': 33200 + plus_plus(1),
    'axis0.motor.current_control.Iq_measured': 33200 + plus_plus(1),
    'axis0.motor.current_control.Id_measured': 33200 + plus_plus(1),
    # axis0.encoder. #
    'axis0.encoder.error': 34000 + plus_plus(0),
    'axis0.encoder.is_ready': 34000 + plus_plus(1),
    'axis0.encoder.index_found': 34000 + plus_plus(1),
    'axis0.encoder.pos_estimate': 34000 + plus_plus(1),
    'axis0.encoder.pos_circular': 34000 + plus_plus(1),
    'axis0.encoder.vel_estimate': 34000 + plus_plus(1),
    'axis0.encoder.spi_error_rate': 34000 + plus_plus(1),
    'axis0.encoder.battery_voltage': 34000 + plus_plus(1),
    'axis0.encoder.pos_abs': 34000 + plus_plus(1),
    'axis0.encoder.turn_abs': 34000 + plus_plus(1),
    'axis0.encoder.spi_buff_0': 34000 + plus_plus(1),
    'axis0.encoder.spi_buff_1': 34000 + plus_plus(1),
    'axis0.encoder.spi_buff_2': 34000 + plus_plus(1),
    'axis0.encoder.encoder_voltage': 34000 + plus_plus(1),
    'axis0.encoder.charge_state': 34000 + plus_plus(1),
    # axis0.encoder.config. #
    'axis0.encoder.config.mode': 34100 + plus_plus(0),
    'axis0.encoder.config.use_index': 34100 + plus_plus(1),
    'axis0.encoder.config.cpr': 34100 + plus_plus(1),
    'axis0.encoder.config.pre_calibrated': 34100 + plus_plus(1),
    'axis0.encoder.config.bandwidth': 34100 + plus_plus(1),
    'axis0.encoder.config.abs_spi_cs_gpio_pin': 34100 + plus_plus(1),
    # axis0.encode.config.  #
    'axis0.trap_traj.config.vel_limit': 35100 + plus_plus(0),
    'axis0.trap_traj.config.accel_limit': 35100 + plus_plus(1),
    'axis0.trap_traj.config.decel_limit': 35100 + plus_plus(1),
    'axis0.trap_traj.config.traj_mode': 35100 + plus_plus(1),
    # axis0.fet_thermistor. #
    'axis0.fet_thermistor.error': 36000 + plus_plus(0),
    'axis0.fet_thermistor.temperature': 36000 + plus_plus(1),
    # axis0.fet_thermistor.config. #
    'axis0.fet_thermistor.config.enabled': 36100 + plus_plus(0),
    'axis0.fet_thermistor.config.temp_limit_lower': 36100 + plus_plus(1),
    'axis0.fet_thermistor.config.temp_limit_upper': 36100 + plus_plus(1),
    # axis0.motor_thermistor. #
    'axis0.motor_thermistor.error': 37000 + plus_plus(0),
    'axis0.motor_thermistor.temperature': 37000 + plus_plus(1),
    # axis0.motor_thermistor.config. #
    'axis0.motor_thermistor.config.enabled': 37100 + plus_plus(0),
    'axis0.motor_thermistor.config.temp_limit_lower': 37100 + plus_plus(1),
    'axis0.motor_thermistor.config.temp_limit_upper': 37100 + plus_plus(1),
    # axis0.output_shaft. #
    'axis0.output_shaft.input_pos': 38000 + plus_plus(0),
    'axis0.output_shaft.input_vel': 38000 + plus_plus(1),
    'axis0.output_shaft.input_torque': 38000 + plus_plus(1),
    'axis0.output_shaft.pos_setpoint': 38000 + plus_plus(1),
    'axis0.output_shaft.vel_setpoint': 38000 + plus_plus(1),
    'axis0.output_shaft.torque_setpoint': 38000 + plus_plus(1),
    'axis0.output_shaft.pos_estimate': 38000 + plus_plus(1),
    'axis0.output_shaft.vel_estimate': 38000 + plus_plus(1),
    'axis0.output_shaft.torque_estimate': 38000 + plus_plus(1),
    'axis0.output_shaft.circular_setpoint_min': 38000 + plus_plus(1),
    'axis0.output_shaft.circular_setpoint_max': 38000 + plus_plus(1),
    'axis0.output_shaft.vel_limit': 38000 + plus_plus(1),
    'axis0.output_shaft.torque_lim': 38000 + plus_plus(1),
    'axis0.output_shaft.trap_traj_vel_limit': 38000 + plus_plus(1),
    'axis0.output_shaft.trap_traj_accel_limit': 38000 + plus_plus(1),
    'axis0.output_shaft.trap_traj_decel_limit': 38000 + plus_plus(1),
    'axis0.output_shaft.vel_ramp_rate': 38000 + plus_plus(1),
    'axis0.output_shaft.torque_ramp_rate': 38000 + plus_plus(1),
    'axis0.output_shaft.steps_per_turn': 38000 + plus_plus(1),
    'axis0.output_shaft.torque_constant': 38000 + plus_plus(1),
    'axis0.output_shaft.load_inertia': 38000 + plus_plus(1),
    'axis0.output_shaft.homing_torque_lim':38000 + plus_plus(1),
    'axis0.output_shaft.homing_vel_limit': 38000 + plus_plus(1),
    'axis0.output_shaft.homing_pos_setpoint': 38000 + plus_plus(1),
    'axis0.output_shaft.homing_move_range': 38000 + plus_plus(1),
}

property_type = {
    #     #
    'vbus_voltage': 'f',
    'ibus': 'f',
    'serial_number_l': 'u32',
    'serial_number_h': 'u32',
    'hw_version': 'u32',
    'fw_version': 'u32',
    # config. #
    'config.uart_baudrate': 'u32',
    'config.dc_bus_undervoltage_trip_level': 'f',
    'config.dc_bus_overvoltage_trip_level': 'f',
    'config.dc_max_positive_current': 'f',
    'config.dc_max_negative_current': 'f',
    'config.max_regen_current': 'f',
    'config.brake_resistance': 'f',
    'config.enable_uart': 'u32',
    # can. #
    'can.error': 'u32',
    # can.config. #
    'can.config.baud_rate': 'u32',
    'can.config.protocol': 'u32',
    # axis0. #
    'axis0.error': 'u32',
    'axis0.current_state': 'u32',
    'axis0.requested_state': 'u32',
    # axis0.config. #
    'axis0.config.can_node_id': 'u32',
    'axis0.config.startup_motor_calibration': 'u32',
    'axis0.config.startup_encoder_index_search': 'u32',
    'axis0.config.startup_encoder_offset_calibration': 'u32',
    'axis0.config.startup_closed_loop_control': 'u32',
    'axis0.config.enable_step_dir': 'u32',
    'axis0.config.turns_per_step': 'f',
    # axis0.config.calibration_lockin. #
    'axis0.config.calibration_lockin.current': 'f',
    'axis0.config.calibration_lockin.ramp_time': 'f',
    'axis0.config.calibration_lockin.ramp_distance': 'f',
    'axis0.config.calibration_lockin.accel': 'f',
    'axis0.config.calibration_lockin.vel': 'f',
    # axis0.config.extra_setting. #
    'axis0.config.extra_setting.enable_120r': 'u32',
    'axis0.config.extra_setting.enable_circular_setpoint_limit': 'u32',
    'axis0.config.extra_setting.circular_setpoint_min': 'f',
    'axis0.config.extra_setting.circular_setpoint_max': 'f',
    'axis0.config.extra_setting.zero_offset_counts': 'f',
    'axis0.config.extra_setting.stored_turns_counts': 's32',
    'axis0.config.extra_setting.enable_multi_circle': 'u32',
    'axis0.config.extra_setting.gear_ratio': 'f',
    'axis0.config.extra_setting.stall_current_limit': 'f',
    'axis0.config.extra_setting.stall_time_limit': 'f',
    'axis0.config.extra_setting.enable_stall_limit': 'u32',
    'axis0.config.extra_setting.enable_adaptive_gain': 'u32',
    'axis0.config.extra_setting.encoder_offset_calibrated': 'u32',
    'axis0.config.extra_setting.enable_reply_state': 'u32',
    'axis0.config.extra_setting.enable_damp_idle': 'u32',
    # axis0.controller. #
    'axis0.controller.error': 'u32',
    'axis0.controller.input_pos': 'f',
    'axis0.controller.input_vel': 'f',
    'axis0.controller.input_torque': 'f',
    'axis0.controller.pos_setpoint': 'f',
    'axis0.controller.vel_setpoint': 'f',
    'axis0.controller.torque_setpoint': 'f',
    'axis0.controller.trajectory_done': 'u32',
    # axis0.controller.config. #
    'axis0.controller.config.enable_gain_scheduling': 'u32',
    'axis0.controller.config.enable_vel_limit': 'u32',
    'axis0.controller.config.enable_current_mode_vel_limit': 'u32',
    'axis0.controller.config.enable_overspeed_error': 'u32',
    'axis0.controller.config.control_mode': 'u32',
    'axis0.controller.config.input_mode': 'u32',
    'axis0.controller.config.gain_scheduling_width': 'f',
    'axis0.controller.config.pos_gain': 'f',
    'axis0.controller.config.vel_gain': 'f',
    'axis0.controller.config.vel_integrator_gain': 'f',
    'axis0.controller.config.vel_limit': 'f',
    'axis0.controller.config.vel_limit_tolerance': 'f',
    'axis0.controller.config.vel_ramp_rate': 'f',
    'axis0.controller.config.torque_ramp_rate': 'f',
    'axis0.controller.config.circular_setpoints': 'u32',
    'axis0.controller.config.circular_setpoint_range': 'f',
    'axis0.controller.config.inertia': 'f',
    'axis0.controller.config.input_filter_bandwidth': 'f',
    # axis0.motor. #
    'axis0.motor.error': 'u32',
    'axis0.motor.is_calibrated': 'u32',
    'axis0.motor.effective_current_lim': 'f',
    # axis0.motor.config. #
    'axis0.motor.config.pre_calibrated': 'u32',
    'axis0.motor.config.pole_pairs': 's32',
    'axis0.motor.config.calibration_current': 'f',
    'axis0.motor.config.resistance_calib_max_voltage': 'f',
    'axis0.motor.config.phase_inductance': 'f',
    'axis0.motor.config.phase_resistance': 'f',
    'axis0.motor.config.torque_constant': 'f',
    'axis0.motor.config.direction': 's32',
    'axis0.motor.config.motor_type': 'u32',
    'axis0.motor.config.current_lim': 'f',
    'axis0.motor.config.current_lim_margin': 'f',
    'axis0.motor.config.torque_lim': 'f',
    'axis0.motor.config.requested_current_range': 'f',
    'axis0.motor.config.current_control_bandwidth': 'f',
    # axis.motor.current_control. #
    'axis0.motor.current_control.Ibus': 'f',
    'axis0.motor.current_control.final_v_alpha': 'f',
    'axis0.motor.current_control.final_v_beta': 'f',
    'axis0.motor.current_control.Id_setpoint': 'f',
    'axis0.motor.current_control.Iq_setpoint': 'f',
    'axis0.motor.current_control.Iq_measured': 'f',
    'axis0.motor.current_control.Id_measured': 'f',
    # axis0.encoder. #
    'axis0.encoder.error': 'u32',
    'axis0.encoder.is_ready': 'u32',
    'axis0.encoder.index_found': 'u32',
    'axis0.encoder.pos_estimate': 'f',
    'axis0.encoder.pos_circular': 'f',
    'axis0.encoder.vel_estimate': 'f',
    'axis0.encoder.spi_error_rate': 'f',
    'axis0.encoder.battery_voltage': 'f',
    'axis0.encoder.pos_abs': 's32',
    'axis0.encoder.turn_abs': 's32',
    'axis0.encoder.spi_buff_0': 'u32',
    'axis0.encoder.spi_buff_1': 'u32',
    'axis0.encoder.spi_buff_2': 'u32',
    'axis0.encoder.encoder_voltage': 'f',
    'axis0.encoder.charge_state': 'u32',
    # axis0.encoder.config. #
    'axis0.encoder.config.mode': 'u32',
    'axis0.encoder.config.use_index': 'u32',
    'axis0.encoder.config.cpr': 's32',
    'axis0.encoder.config.pre_calibrated': 'u32',
    'axis0.encoder.config.bandwidth': 'f',
    'axis0.encoder.config.abs_spi_cs_gpio_pin': 'u32',
    # axis0.trap_traj.config  #
    'axis0.trap_traj.config.vel_limit': 'f',
    'axis0.trap_traj.config.accel_limit': 'f',
    'axis0.trap_traj.config.decel_limit': 'f',
    'axis0.trap_traj.config.traj_mode': 'u32',
    # axis0.fet_thermistor. #
    'axis0.fet_thermistor.error': 'u32',
    'axis0.fet_thermistor.temperature': 'f',
    # axis0.fet_thermistor.config. #
    'axis0.fet_thermistor.config.enabled': 'u32',
    'axis0.fet_thermistor.config.temp_limit_lower': 'f',
    'axis0.fet_thermistor.config.temp_limit_upper': 'f',
    # axis0.motor_thermistor. #
    'axis0.motor_thermistor.error': 'u32',
    'axis0.motor_thermistor.temperature': 'f',
    # axis0.motor_thermistor.config. #
    'axis0.motor_thermistor.config.enabled': 'u32',
    'axis0.motor_thermistor.config.temp_limit_lower': 'f',
    'axis0.motor_thermistor.config.temp_limit_upper': 'f',
    # axis0.output_shaft. #
    'axis0.output_shaft.input_pos': 'f',
    'axis0.output_shaft.input_vel': 'f',
    'axis0.output_shaft.input_torque': 'f',
    'axis0.output_shaft.pos_setpoint': 'f',
    'axis0.output_shaft.vel_setpoint': 'f',
    'axis0.output_shaft.torque_setpoint': 'f',
    'axis0.output_shaft.pos_estimate': 'f',
    'axis0.output_shaft.vel_estimate': 'f',
    'axis0.output_shaft.torque_estimate': 'f',
    'axis0.output_shaft.circular_setpoint_min': 'f',
    'axis0.output_shaft.circular_setpoint_max': 'f',
    'axis0.output_shaft.vel_limit': 'f',
    'axis0.output_shaft.torque_lim': 'f',
    'axis0.output_shaft.trap_traj_vel_limit': 'f',
    'axis0.output_shaft.trap_traj_accel_limit': 'f',
    'axis0.output_shaft.trap_traj_decel_limit': 'f',
    'axis0.output_shaft.vel_ramp_rate': 'f',
    'axis0.output_shaft.torque_ramp_rate': 'f',
    'axis0.output_shaft.steps_per_turn': 'f',
    'axis0.output_shaft.torque_constant': 'f',
    'axis0.output_shaft.load_inertia': 'f',
    'axis0.output_shaft.homing_torque_lim': 'f',
    'axis0.output_shaft.homing_vel_limit': 'f',
    'axis0.output_shaft.homing_pos_setpoint': 'f',
    'axis0.output_shaft.homing_move_range': 'f',
}


# 通过属性编码找到名称
def value_find_key(value=1):
    dict_temp = property_addresss
    if value in dict_temp.values():
        return list(dict_temp.keys())[list(dict_temp.values()).index(value)]
    else:
        return 0


# 通过名称找到属性编码
def key_find_value(key=''):
    if 'axis' in key:
        temp = key.split('.', 1)[-1]
        key = 'axis0.' + temp
    dict_temp = property_addresss
    if key in dict_temp.keys():
        return dict_temp[key]
    else:
        return 0


error_codes = {
    'axis_error': [
        'AXIS_ERROR_NONE',
        'AXIS_ERROR_INVALID_STATE',
        'AXIS_ERROR_DC_BUS_UNDER_VOLTAGE',
        'AXIS_ERROR_DC_BUS_OVER_VOLTAGE',
        'AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT',
        'AXIS_ERROR_BRAKE_RESISTOR_DISARMED',
        'AXIS_ERROR_MOTOR_DISARMED',
        'AXIS_ERROR_MOTOR_FAILED',
        'AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED',
        'AXIS_ERROR_ENCODER_FAILED',
        'AXIS_ERROR_CONTROLLER_FAILED',
        'AXIS_ERROR_POS_CTRL_DURING_SENSORLESS',
        'AXIS_ERROR_WATCHDOG_TIMER_EXPIRED',
        'AXIS_ERROR_MIN_ENDSTOP_PRESSED',
        'AXIS_ERROR_MAX_ENDSTOP_PRESSED',
        'AXIS_ERROR_ESTOP_REQUESTED',
        'AXIS_ERROR_UNKOWN_ERROR_CODE1',
        'AXIS_ERROR_UNKOWN_ERROR_CODE2',
        'AXIS_ERROR_HOMING_WITHOUT_ENDSTOP',
        'AXIS_ERROR_OVER_TEMP'],

    'motor_error': [
        'MOTOR_ERROR_NONE',
        'MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE',
        'MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE',
        'MOTOR_ERROR_ADC_FAILED',
        'MOTOR_ERROR_DRV_FAULT',
        'MOTOR_ERROR_CONTROL_DEADLINE_MISSED',
        'MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE',
        'MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE',
        'MOTOR_ERROR_MODULATION_MAGNITUDE',
        'MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION',
        'MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK',
        'MOTOR_ERROR_CURRENT_SENSE_SATURATION',
        'MOTOR_ERROR_UNKOWN_ERROR_CODE',
        'MOTOR_ERROR_CURRENT_LIMIT_VIOLATION',
        'MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN',
        'MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT',
        'MOTOR_ERROR_DC_BUS_OVER_CURRENT',
        'MOTOR_ERROR_STALL_LIMIT_VIOLATION'],

    'thermistor_error': [
        'THERMISTOR_CURRENT_LIMITER_ERROR_NONE',
        'THERMISTOR_CURRENT_LIMITER_ERROR_OVER_TEMP'],

    'encoder_error': [
        'ENCODER_ERROR_NONE',
        'ENCODER_ERROR_UNSTABLE_GAIN',
        'ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH',
        'ENCODER_ERROR_NO_RESPONSE',
        'ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE',
        'ENCODER_ERROR_ILLEGAL_HALL_STATE',
        'ENCODER_ERROR_INDEX_NOT_FOUND_YET',
        'ENCODER_ERROR_ABS_SPI_TIMEOUT',
        'ENCODER_ERROR_ABS_SPI_COM_FAIL',
        'ENCODER_ERROR_ABS_SPI_NOT_READY',
        'ENCODER_ERROR_EDR_OVER_RANGE',
        'ENCODER_ERROR_EDR_POWER_FAILURE',
        'ERROR_EDR_BATTERY_LOW ',
        'ERROR_OFFSET_CALIBRATE_FAILURE',
        'ERROR_INCORRECT_OFFSET',
        'ERROR_TURNS_COUNTER_WARNING'],

    'controller_error': [
        'CONTROLLER_ERROR_NONE',
        'CONTROLLER_ERROR_OVERSPEED',
        'CONTROLLER_ERROR_INVALID_INPUT_MODE',
        'CONTROLLER_ERROR_UNSTABLE_GAIN',
        'CONTROLLER_ERROR_INVALID_MIRROR_AXIS',
        'CONTROLLER_ERROR_INVALID_LOAD_ENCODER',
        'CONTROLLER_ERROR_INVALID_ESTIMATE'],

    'can_error': [
        'CAN_ERROR_NONE',
        'CAN_ERROR_DUPLICATE_CAN_IDS'],

    'fet_thermistor_error': [
        'FET_THERMISTOR_CURRENT_LIMITER_ERROR_NONE',
        'FET_THERMISTOR_CURRENT_LIMITER_ERROR_OVER_TEMP'],

    'motor_thermistor_error': [
        'MOTOR_THERMISTOR_CURRENT_LIMITER_ERROR_NONE',
        'MOTOR_THERMISTOR_CURRENT_LIMITER_ERROR_OVER_TEMP']

}


def error_decode(data=''):
    line_list = data.split('\r\n')
    for line in line_list:
        if line != '':
            print(line)
            error_name = line.split(': ')[0] + '_error'
            code_num = int(line.split()[1])
            error_list = []
            for i in range(len(error_codes[error_name]) - 1):
                if code_num & (0x01 << i):
                    print(error_codes[error_name][i + 1])
