
### Dependencies

`ros-<distro>-serial`

### RoboteqDemo

`rosrun burro_base_cpp RoboteqDemo`

Connects to two roboteq devices on `/dev/ttyUSB0` and `/dev/ttyUSB1`

Send it string commands on the `/burro_base/command` ROS topic, eg `?C_` to read the encoders
