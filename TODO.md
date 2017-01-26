- [x] Fly bebop with joy node

  It flies well. Zero position of control corresponds to 0 filled Twist -> hovering (Separate controller)
  At max speed, it brakes in about 60 cm

- [x] Fix an optitrack marker (had marker) to the frame/battery
- [ ] Fuse optitrack and odom (maily to obtain the twist) using robot_localization
  - [x] record a bag
    - modelled relation cmd_vel -> acceleration
  - [ ] process noise is low -> too much lag, maybe
  - [x] compute directly the Twist (stamped) from the optitrack pose (recovered my old node optitrack2odom)
    velQuater = ((diffQuater * 2) / d_time) * conjBoxQuater; (or reverse if in moving frame i.e 2 q^-1 q')
  - [ ] review node
    - [ ] child_frame_id as parameter
    - [ ] history length should be in time (not number of elements)
  - [x] changed to child_frame = World (so I don't need to transform Twist)
- [ ] Fence control
  - [ ] Fence defined in arbitrary frames (Polygon -> shapely polygon in World frame)
  - [x] Repulsion/stopping in tau
  - [x] Add cmd_vel pass through
  - [x] Return to home (and land) for low battery (maybe move in a separate node)
  - [x] add bounds for cmd_vel (i.e. max/min accelerations)
  - [ ] tune eta/tau/delay
  - [x] add enable/disable
  - [x] add dyn reconfig
- [x] Go to point control
  - [ ] Add action interface
  - [x] Add position bounds on target control (i.e. use fence)
  - [x] Add z bounds to target

- [ ] Assisted joystick control
  - [x] World frame
  - [x] head frame
  - [x] joy in pos control mode
    - [ ] not satisfyng yet
  - [x] modify config of joy_teleop to add new actions for:
    - [ ] mode switching
- [ ] Safety
  - What happen if lose connection??
  - [x] If not tracked -> land or hover!!! (already implemented in opritrack_odom)

- [ ] Multiple bebop_msgs
  - see https://github.com/tnaegeli/multiple_bebops

- [ ] Demo
  - part of old Juan version
    - [ ] gestures (for takeoff, land, follow and hover)

- [x] Head tracker
  - [ ] simple separate node (at first without fence)
