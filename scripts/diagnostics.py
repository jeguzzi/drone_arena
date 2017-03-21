#!/usr/bin/env python

import rospy
import diagnostic_updater
import diagnostic_msgs
import bebop_msgs.msg as b_msg

OK = diagnostic_msgs.msg.DiagnosticStatus.OK
WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR
STALE = diagnostic_msgs.msg.DiagnosticStatus.STALE


class BebopState(object):

    def __init__(self, updater):
        self.msg = None
        rospy.Subscriber('states/{topic}'.format(topic=self.topic), self.msg_type,
                         self.has_received_msg)
        rospy.set_param('bebop_driver/states/{param}'.format(param=self.param), True)
        updater.add(self.name, self.update)

    def has_received_msg(self, msg):
        # print(self.name, msg)
        self.msg = msg

    def update_stat(self, stat):
        raise NameError('Not implemented')

    def update(self, stat):
        # print(self.name, self.msg)
        if not self.msg:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE,
                         "No {name} status received".format(name=self.name))
            return stat
        d = (rospy.Time.now() - self.msg.header.stamp).to_sec()
        stat = self.update_stat(stat)
        stat.add("last updated", "%.0f seconds ago" % d)
        return stat


def sensor_name(name):
    return getattr(b_msg.CommonCommonStateSensorsStatesListChanged, 'sensorName_{0}'.format(name))


class SensorState(BebopState):
    name = 'Sensors'
    param = 'enable_commonstate_sensorsstateslistchanged'
    topic = 'common/CommonState/SensorsStatesListChanged'
    msg_type = b_msg.CommonCommonStateSensorsStatesListChanged

    def __init__(self, updater):
        super(SensorState, self).__init__(updater)
        self.sensors = {sensor_name(n): {'working': True, 'name': n}
                        for n in ['IMU', 'barometer', 'ultrasound', 'GPS', 'magnetometer',
                                  'vertical_camera']}

    def has_received_msg(self, msg):
        super(SensorState, self).has_received_msg(msg)
        self.sensors[msg.sensorName]['working'] = msg.sensorState == 1

    def update_stat(self, stat):
        if len([s for s, v in self.sensors.items() if v == 0]) > 0:
            stat.summary(WARN, "Not working properly")
        else:
            stat.summary(OK, "Ok")
        for v in self.sensors.values():
            stat.add(v['name'], v['working'])
        return stat


def state_name(name):
    return getattr(b_msg.Ardrone3PilotingStateFlyingStateChanged, 'state_{0}'.format(name), -1)


class AutopilotState(BebopState):
    name = 'Autopilot state'
    param = 'enable_pilotingstate_flyingstatechanged'
    topic = 'ardrone3/PilotingState/FlyingStateChanged'
    msg_type = b_msg.Ardrone3PilotingStateFlyingStateChanged

    def __init__(self, *args):
        super(AutopilotState, self).__init__(*args)
        self.state = {state_name(n): (n, OK)
                      for n in ['landed', 'hovering', 'flying', 'landing', 'usertakeoff',
                                'takingoff']}
        self.state.update({state_name(n): (n, WARN)
                           for n in ['emergency', 'emergency_landing']})

    def update_stat(self, stat):
        s = self.msg.state
        name, diag = self.state[s]
        stat.summary(diag, name)
        return stat


def alert_name(name):
    return getattr(b_msg.Ardrone3PilotingStateAlertStateChanged, 'state_{0}'.format(name), -1)


class AutopilotAlert(BebopState):
    name = 'Autopilot alert'
    param = 'enable_pilotingstate_alertstatechanged'
    topic = 'ardrone3/PilotingState/AlertStateChanged'
    msg_type = b_msg.Ardrone3PilotingStateAlertStateChanged

    def __init__(self, *args):
        super(AutopilotAlert, self).__init__(*args)
        self.alert = {alert_name(n): (n, ERROR)
                      for n in ['user', 'cut_out', 'critical_battery', 'low_battery',
                                'too_much_angle']}
        self.alert.update({alert_name(n): (n, OK) for n in ['none']})

    def update_stat(self, stat):
        s = self.msg.state
        name, diag = self.alert[s]
        stat.summary(diag, name)
        return stat


class BatteryState(BebopState):
    name = 'Battery'
    param = 'enable_commonstate_batterystatechanged'
    topic = 'common/CommonState/BatteryStateChanged'
    msg_type = b_msg.CommonCommonStateBatteryStateChanged

    def update_stat(self, stat):
        battery_msg = self.msg
        p = battery_msg.percent
        if p < 10:
            stat.summary(WARN, "Almost empty")
        else:
            stat.summary(OK, "Ok")
        stat.add("percent", p)
        return stat


class MagnetormeterState(BebopState):
    name = 'Magnetometer'
    param = 'enable_calibrationstate_magnetocalibrationrequiredstate'
    topic = 'common/CalibrationState/MagnetoCalibrationRequiredState'
    msg_type = b_msg.CommonCalibrationStateMagnetoCalibrationRequiredState

    def update_stat(self, stat):
        if self.msg.required == 1:
            stat.summary(WARN, 'Need calibration')
        else:
            stat.summary(OK, 'Calibration ok')
        return stat


class OverheatingState(BebopState):
    name = 'Overheating'
    param = 'enable_overheatstate_overheatchanged'
    topic = 'common/OverHeatState/OverHeatChanged'
    msg_type = b_msg.CommonOverHeatStateOverHeatChanged

    def update_stat(self, stat):
        stat.summary(WARN, 'Overheating')
        return stat


class WifiState(BebopState):
    name = 'Wifi'
    param = 'enable_commonstate_wifisignalchanged'
    topic = 'common/CommonState/WifiSignalChanged'
    msg_type = b_msg.CommonCommonStateWifiSignalChanged

    def update_stat(self, stat):
        wifi_signal_msg = self.msg
        rssi = wifi_signal_msg.rssi
        if rssi > -75:
            stat.summary(OK, "Ok")
        elif rssi > -87:
            stat.summary(WARN, "Medium quality")
        elif rssi > -96:
            stat.summary(WARN, "Low quality")
        else:
            stat.summary(ERROR, "Quality too low")
        stat.add('RSSI', "%d dBm" % rssi)
        return stat


class BebopDiagnostics(object):

    def __init__(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("bebop diagnostic")
        clss = [AutopilotState, AutopilotAlert, SensorState, BatteryState, WifiState,
                OverheatingState, MagnetormeterState]
        self.bebop_states = [cls(self.updater) for cls in clss]
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)

    def update_diagnostics(self, event):
        self.updater.update()


if __name__ == '__main__':
    try:
        rospy.init_node('diagnostics', anonymous=True)
        BebopDiagnostics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
