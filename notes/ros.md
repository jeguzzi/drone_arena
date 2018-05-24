# ROS Architecture

## Interfaces

Topic and messages provide an API for ROS nodes.

### Bebop driver

[bebop_autonomy](https://bebop-autonomy.readthedocs.io) is a ROS driver for the Parrot Bebop drone.

Among other topics, it exposes the following API.

#### Input (topic subscribers)

- `takeoff: std_msgs/Empty`
- `land: std_msgs/Empty`
- `cmd_vel: geometry_msgs/Twist`

  + `cmd_vel.linear.x` in [0, 1] maps to (desired) pitch (i.e, acceleration)
  + `cmd_vel.linear.y` in [0, 1] maps to (desired) roll (i.e, acceleration)
  + `cmd_vel.linear.z` in [0, 1] maps to (desired) vertical speed
  + `cmd_vel.angular.z` in [0, 1] maps to (desired) angular speed

  > The linear.x and linear.y parts of this message set the pitch and roll angles of the Bebop, respectively, hence control its forward and lateral accelerations. The resulting pitch/roll angles depend on the value of ~PilotingSettingsMaxTiltCurrent parameter, which is specified in degrees and is dynamically reconfigurable (Dynamically Reconfigurable Parameters for Bebop).

  >The linear.z part of this message controls the vertical velocity of the Bebop. The resulting velocity in m/s depends on the value of ~SpeedSettingsMaxVerticalSpeedCurrent parameter, which is specified in meters per second and is also dynamically reconfigurable (Dynamically Reconfigurable Parameters for Bebop). Similarly, the angular.z component of this message controls the rotational velocity of the Bebop (around the z-axis). The corresponding scaling parameter is SpeedSettingsMaxRotationSpeedCurrent (in degrees per sec).

  ```roll_degree     = linear.y  * max_tilt_angle
  pitch_degree      = linear.x  * max_tilt_angle
  ver_vel_m_per_s   = linear.z  * max_vert_speed
  rot_vel_deg_per_s = angular.z * max_rot_speed
  ```

#### Output (topic publishers)

- `odom: nav_msgs/Odometry`
- `/tf: tfMessage`
- `states/ARDrone3/PilotingState/FlyingStateChanged: bebop_msgs/Ardrone3PilotingStateFlyingStateChanged`
  ```
  Header header

  # Drone flying state
  uint8 state_landed=0  # Landed state
  uint8 state_takingoff=1  # Taking off state
  uint8 state_hovering=2  # Hovering / Circling (for fixed wings) state
  uint8 state_flying=3  # Flying state
  uint8 state_landing=4  # Landing state
  uint8 state_emergency=5  # Emergency state
  uint8 state_usertakeoff=6  # User take off state. Waiting for user action to take off.
  uint8 state_motor_ramping=7  # Motor ramping state (for fixed wings).
  uint8 state_emergency_landing=8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.
  uint8 state
  ```


**Good practice**:
Specific information is encoded in *custom* (package) messages, general information (like pose) in standard messages.

### Our interface

  1. we [interface with the bebop driver](https://github.com/jeguzzi/drone_arena/blob/master/scripts/fence_control.py#L160), i.e., sending
      - `cmd_vel: geometry_msgs/Twist`
      - `land|takeoff: std_msgs/Empty`
  2. we get poses and velocities from the Optitrack
      - `bebop/mocap_odom: nav_msgs/Odometry` (in a fixed WORLD frame)
      - `head/mocap_odom: nav_msgs/Odometry` (in a fixed WORLD frame)
  3. we catch (and bypass to add safety) some of the driver inputs (ROS allows topic renaming at launch time)
      - `cmd_vel: geometry_msgs/Twist` => `cmd_vel_input: geometry_msgs/Twist` (driver)
      - `takeoff: geometry_msgs/Twist` => `takeoff_input: geometry_msgs/Twist` (driver)
  4. (as a library) we [offer services](https://github.com/jeguzzi/drone_arena/blob/master/scripts/fence_control.py#L242):
      - `target: geometry_msgs/PoseStamped`
      - `des_body_vel: geometry_msgs/Twist`
      - `des_vel: geometry_msgs/TwistStamped`


## Our controller

Every time we receive an odometry, we update the drone state:

```python
def has_received_odometry(self, msg):
       if not self.localization_active:
           return
       # Transform pose to World and twist to world
       odom = odometry_in_frame(self.tf_buffer, msg, self.frame_id, self.frame_id)
       if not odom:
           return
       msg = odom
       self.last_localization = msg.header.stamp
       _p = msg.pose.pose.position
       self.z = _p.z
       _v = msg.twist.twist.linear
       o = msg.pose.pose.orientation
       # Position in world_frame
       self.position = [_p.x, _p.y, _p.z]
       self.inside_fence = self.inside_fence_margin(self.position)
       self.q = [o.x, o.y, o.z, o.w]
       _, _, self.yaw = euler_from_quaternion(self.q) inverse=True)[:2]
       self.velocity = [_v.x, _v.y, _v.z]
       self.acc_bounds = acceleration_bounds(
           self.pos_bounds[:2], self.position[:2], self.velocity[:2],
           self.tau, self.eta, self.delay)
       self.localized = True
       self.publish_location()
```

We compute and send a new command to the drone at 20Hz:

```python
def update(self, evt):
    # check if we still are localized
    self.update_localization_state()
    # transform last input from joystick to a target acceleration
    self.update_hovering_cmd()
    if self.localized:
        if self.target_position is not None:
            self.update_pose_control(self.target_position, self.target_yaw,
                                     self.target_velocity)
        elif self.hovering_cmd:
            msg = Twist()
            msg.linear.z = min(0, 2 * (self.max_height - self.z))
            self.clamp_in_fence_cmd(msg)
            self.pub_cmd.publish(msg)
```

<img src="https://raw.githubusercontent.com/jeguzzi/drone_arena/master/notes/fence_control.png" width="700"/>
