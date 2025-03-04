# minibot_pc_interface

Testing ros2 implementation of a turtlebot3-burger based robot.

## Todo
- [x] Obtain robot description
- [x] Implement basic hardware control with ESP32
- [x] Test basic hardware functionality
- [x] Implement PID Control and fine tune its constants
- [x] Integrate ros2_control with hardware
- [x] Test basic functionality with teleop_keyboard
- [ ] Attach IMU with PI and make it properly publish IMU msgs
- [ ] Apply sensor fusion to fuse IMU and Odometry
- [x] Test basic navigation without LIDAR by using static tf between odom and map
- [x] Attach LIDAR with PI and make it publish LaserScan msgs
- [x] Test final functionality
- [ ] Debug and resolve any leftover issues

## References
- https://github.com/Slamtec/sllidar_ros2
- https://github.com/kimsniper/ros2_mpu6050
- https://www.sparkfun.com/hobby-motor-with-encoder-metal-gear-dg01d-e.html
- https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
