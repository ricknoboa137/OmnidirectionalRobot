# OmnidirectionalRobot
The repository contains the current state of the development of an omnidirectional mobile robot platform using ROS 

## Contents

- `ESP32_MotorDriver/` - firmware for the three-wheel motor driver: encoder reading, one PID per wheel,
  MQTT setpoints on `motor_velocities` and velocity telemetry on `motor_telemetry`.
- `ESP32_MotorDriver_original/` - untouched backup of the driver firmware as it was before the
  telemetry section was added. Kept as a separate sketch so it still compiles on its own.
- `velocity_logger.py` - subscribes to the telemetry topic, stores the wheel velocities in a CSV log
  file and plots setpoint vs. measured velocity and PID output for each wheel.
- `remote_control_MQTT.py` - publishes gamepad axes to the robot over MQTT.
- `ARM_connection.py`, `armband_TestModel.py`, `LSTM_model_04.keras` - sEMG armband interface and gesture model.

### Logging the wheel velocities

```
python velocity_logger.py --broker <broker-ip> --plot
```

The firmware streams one CSV row per control cycle (200 Hz):
`seq,t_ms,setA,velA,pwmA,setB,velB,pwmB,setC,velC,pwmC`, batched 8 rows per MQTT message.
Publishing `1` / `0` on `motor_telemetry_ctrl` starts and stops the stream, `s1` / `s0` mirrors it
on the USB serial port.
