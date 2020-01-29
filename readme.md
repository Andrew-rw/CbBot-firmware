## Firmware for CbBot 4.0 Base Controller

**Deprecated. Not maintained anymore**

Based on Linorobot firmware

**Responsible for:**
 - Motor PID regulation;
 - Velocity calculation;
 - IMU (MPU-6150);
 - Battery voltage monitoring;
 - Piezo buzzer support

**Subscribed ROS topics:**
 - cmd_vel (geometry_msgs/Twist) - velocity commands;
 - pid (cbbot_msgs/cfgPID) - Obsolete. Used for passing PID controller coefficients in realtime;
 - srv_cmd (cbbot_msgs/CBService) - Service command topic. Used for sending service commands to base controller. For commands list refer to cbbot_msgs/doc/commands.txt

**Published ROS topics:**
 - raw_imu (ros_arduino_msgs/RawImu) - IMU sensor data: gyroscope, accelerometer, compass;
 - raw_vel (geometry_msgs/Vector3Stamped) - velocity data;
 - battery_state (sensor_msgs/BatteryState) - battery information
