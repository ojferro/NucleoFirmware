# AXIS 0
odrv0.axis0.motor.config.current_lim = 10
odrv0.axis0.controller.config.vel_limit = 10
odrv0.config.enable_brake_resistor = True
odrv0.axis0.motor.config.calibration_current = 10
odrv0.config.dc_max_negative_current = -0.010
odrv0.config.dc_max_positive_current = 10



odrv0.axis0.motor.config.pole_pairs = 11
odrv0.axis0.motor.config.torque_constant = 8.27/380
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
odrv0.axis0.encoder.config.cpr = 2**14

odrv0.save_configuration()

# Set CAN baud rate
# odrv0.can.config.baud_rate = 500000
odrv0.axis0.config.can.node_id = 3
odrv0.axis1.config.can.node_id = 4

# odrv0.axis0.config.can.bus_vi_rate_ms = 0
# odrv0.axis1.config.can.bus_vi_rate_ms = 0

# odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis0.controller.config.pos_gain = 20.0
odrv0.axis0.controller.config.vel_gain = 0.06
odrv0.axis0.controller.config.vel_integrator_gain = 0.0

odrv0.save_configuration()

odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL

# Start closed loop position control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_IDLE



odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv0.axis0.controller.input_vel = 1.0

# Start closed loop velocity control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_IDLE


######## AXIS 1 ########
odrv0.axis1.motor.config.current_lim = 10
odrv0.axis1.controller.config.vel_limit = 10
odrv0.config.enable_brake_resistor = True
odrv0.axis1.motor.config.calibration_current = 10
odrv0.config.dc_max_negative_current = -0.010
odrv0.config.dc_max_positive_current = 10



odrv0.axis1.motor.config.pole_pairs = 11
odrv0.axis1.motor.config.torque_constant = 8.27/380
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 3
odrv0.axis1.encoder.config.cpr = 2**14

odrv0.save_configuration()