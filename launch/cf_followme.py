<launch>
  <node pkg="crazyflie_driver" type="crazyflie_server" name="crazyflie_server" output="screen"/>
  <arg name="use_mocap_for_state_estimation" default="False"/>
  <arg name="use_world_frame" default="True"/>
  <arg name="max_angular_speed" default="2.0"/>
  <arg name="max_speed" default="1.2"/>
  <arg name="max_vertical_speed" default="0.8"/>
  <arg name="target_altitude" default="-0.3"/>
  <arg name="target_range" default="1.3"/>
  <arg name="takeoff_altitude" default="0.5"/>
  <include file="$(find drone_arena)/launch/test_optitrack_odom.launch"/>
  <include file="$(find crazyflie_description)/launch/model.launch">
    <arg name="name" value="cf"/>
    <arg name="camera" value="true"/>
  </include>
  <node clear_params="true" name='supervisor' pkg='drone_arena' type='cf_supervisor.py' output="screen" ns="cf">
    <param name='name' value='cf'/>
    <rosparam subst_value="true" param="">
      radio:
        id: 0
        channel: 100
        bandwidth : 2M
      params: []
      crazyflies:
        -
          id: 6
          params:
            kalman/thetapix: 3.75
            stabilizer/controller: 1
            stabilizer/estimator: 2
        -
          id: 7
          params:
            kalman/thetapix: 3.75
            stabilizer/controller: 1
            stabilizer/estimator: 2
        -
          id: 3
          params:
            kalman/thetapix: 3.75
            stabilizer/controller: 1
            stabilizer/estimator: 2
            <!-- aideck_HIMAX/fps: $(arg rate) -->
    </rosparam>
  </node>
  <node pkg="drone_arena" type="cf_controller.py" name="fence_controller" output="screen" ns="cf">
    <param name="mocap_pose" value="/optitrack/cf" if="$(arg use_mocap_for_state_estimation)"/>
    <param name="use_mocap_z" value="False"/>
    <param name="frame_id" value="$(eval 'World' if arg('use_world_frame') else 'cf/odom')"/>
    <remap from="odom" to="mocap_odom" if="$(arg use_world_frame)"/>
    <remap from="cf_odom" to="odom"/>
    <param name="enable_fence" value="True"/>
    <param name="publish_target" value="False"/>
    <param name="publish_body_vel" value="True"/>
    <param name="publish_cmd" value="False"/>
    <param name="teleop_mode" value="1"/>
    <param name="max_speed" value="$(arg max_speed)"/>
    <param name="max_vertical_speed" value="$(arg max_vertical_speed)"/>
    <param name="max_angular_speed" value="$(arg max_angular_speed)"/>
    <param name="tau" value="0.5"/>
    <param name="eta" value="1.0"/>
    <param name="delay" value="0"/>
    <param name="joy_set_teleop_mode" value="true"/>
    <param name="max_safe_angle" value="0.75"/>
    <param name="takeoff_altitude" value="$(arg takeoff_altitude)"/>
    <param name="land_at_altitude" value="false"/>
    <param name="track_altitude_as_relative" value="true"/>
    <param name="track_relative_altitude" value="$(arg target_altitude)"/>
    <param name="track_distance" value="$(arg target_range)"/>
    <rosparam param="pos_bounds">
      - [-2.1, 2.5]
      - [-2.1, 2.1]
      - [0.25, 1.8]
    </rosparam>
  </node>
  <include file="$(find drone_arena)/launch/joy_teleop.launch">
    <arg name="teleop_config" default="$(find drone_arena)/config/_log710.yaml"/>
    <arg name="namespace" value="cf"/>
    <arg name="joy_dev" value="/dev/input/js0"/>
  </include>
  <node pkg="drone_arena" type="cf_led.py" name="blinkstick" output="screen" ns="cf">
    <param name="leds" value="8"/>
    <param name="max_intensity" value="100"/>
    <param name="ns" value="cf"/>
  </node>
</launch>
