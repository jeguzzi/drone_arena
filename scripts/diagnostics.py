#!/usr/bin/env python

import rospy
import diagnostic_updater
import diagnostic_msgs
import bebop_msgs.msg as b_msg

OK = diagnostic_msgs.msg.DiagnosticStatus.OK
WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR


class BebopState(object):

    def __init__(self, updater, name, param, topic, msg_type, f):
        self.msgs = []
        self.name = name
        rospy.Subscriber('states/{topic}'.format(topic=topic), msg_type, self.has_received_msg)
        rospy.set_param('~states/{param}'.format(param=param), True)
        updater.add(name, self.updater(f))

    def has_received_msg(self, msg):
        self.msgs.append(msg)

    def updater(self, f):
        def g(stat):
            if not self.msgs:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                             "No {name} status received".format(name=self.name))
                return stat
            msg = self.msgs[-1]
            if msg:
                d = (rospy.Time.now() - self.msgs.header.stamp).to_sec()
            else:
                d = None
            if d is not None and d > 60:
                stat.summary(
                    diagnostic_msgs.msg.DiagnosticStatus.WARN,
                    "Not updated for more than 60 seconds")
            else:
                stat = f(stat, self.msgs)
                stat.add("last updated", "%.0f seconds ago" % d)
            return stat
        return g


class BebopDiagnostics(object):

    def __init__(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("bebop diagnostic")
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        self.sensors = {}
        _m = b_msg.Ardrone3PilotingStateFlyingStateChanged
        self.state = {getattr(_m, 'state_{0}'.format(n), -1): (n, OK)
                      for n in ['landed', 'hovering', 'flying', 'landing', 'usertakeoff',
                                'takingoff']}
        self.state.update({getattr(_m, 'state_{0}'.format(n), -1): (n, WARN)
                           for n in ['emergency', 'emergency_landing']})
        _am = b_msg.Ardrone3PilotingStateAlertStateChanged
        self.alert = {getattr(_am, 'state_{0}'.format(n), -1): (n, ERROR)
                      for n in ['user', 'cut_out', 'critical_battery', 'low_battery',
                                'too_much_angle']}
        self.alert.update({getattr(_am, 'state_{0}'.format(n)): (n, OK) for n in ['none']})

        BebopState(self.updater, 'Battery',
                   'enable_commonstate_batterystatechanged',
                   'common/CommonState/BatteryStateChanged',
                   b_msg.CommonCommonStateBatteryStateChanged,
                   self.battery_diagnostics)
        BebopState(self.updater, 'Wifi',
                   'enable_commonstate_wifisignalchanged',
                   'common/CommonState/WifiSignalChanged',
                   b_msg.CommonCommonStateWifiSignalChanged,
                   self.wifi_diagnostics)
        BebopState(self.updater, 'Wifi',
                   'enable_commonstate_sensorsstateslistchanged',
                   'common/CommonState/SensorsStatesListChanged',
                   b_msg.CommonCommonStateSensorsStatesListChanged,
                   self.sensors_diagnostics)
        BebopState(self.updater, 'Overheating',
                   'enable_overheatstate_overheatchanged',
                   'common/OverHeatState/OverHeatChanged',
                   b_msg.CommonOverHeatStateOverHeatChanged,
                   self.overheat)
        BebopState(self.updater, 'Magnetometer',
                   'enable_calibrationstate_magnetocalibrationrequiredstate',
                   'common/CalibrationState/MagnetoCalibrationRequiredState',
                   b_msg.CommonCalibrationStateMagnetoCalibrationRequiredState,
                   self.mag_calibration)
        BebopState(self.updater, 'Autopilot state',
                   'enable_pilotingstate_flyingstatechanged',
                   'ardrone3/PilotingState/FlyingStateChanged',
                   b_msg.Ardrone3PilotingStateFlyingStateChanged,
                   self.autopilot)
        BebopState(self.updater, 'Autopilot alert',
                   'enable_pilotingstate_alertstatechanged',
                   'ardrone3/PilotingState/AlertStateChanged',
                   b_msg.Ardrone3PilotingStateAlertStateChanged,
                   self.autopilot_alert)

    def update_diagnostics(self, event):
        self.updater.update()

    @staticmethod
    def battery_diagnostics(stat, msgs):
        battery_msg = msgs[-1]
        p = battery_msg.percent
        if p < 10:
            stat.summary(WARN, "Almost empty")
        else:
            stat.summary(OK, "Ok")
        stat.add("percent", p)
        return stat

    @staticmethod
    def wifi_diagnostics(stat, msgs):
        wifi_signal_msg = msgs[-1]
        rssi = wifi_signal_msg.rssi
        if rssi > -75:
            stat.summary(OK, "Ok")
        elif rssi > -87:
            stat.summary(WARN, "Medium quality")
        elif rssi > -96:
            stat.summary(WARN, "Low quality")
        else:
            stat.summary(ERROR, "Quality too low")
        stat.data('RSSI', "%d dBm" % rssi)
        return stat

    def sensors_diagnostics(self, stat, msgs):
        for m in msgs:
            self.sensors[m.sensorName] = m.sensorState
        if len([s for s, v in self.sensors.items() if v == 0]) > 0:
            stat.summary(WARN, "Not working properly")
        else:
            stat.summary(OK, "Ok")
        for n in ['IMU', 'barometer', 'ultrasound', 'GPS', 'magnetometer', 'vertical_camera']:
            i = b_msg.CommonCommonStateSensorsStatesListChanged.__getattribute__(
                'sensorName_{n}'.format(n=n))
            v = self.sensors.get(i, 1)
            stat.data(n, v == 1)
        return stat

    @staticmethod
    def overheat(stat, msgs):
        stat.summary(WARN, 'Overheating')
        return stat

    @staticmethod
    def mag_calibration(stat, msgs):
        if msgs[-1].required == 0:
            stat.summary(WARN, 'Need calibration')
        else:
            stat.summary(OK, 'Calibration ok')
        return stat

    def autopilot(self, stat, msgs):
        state = msgs[-1].state
        name, diag = self.state[state]
        stat.summary(diag, name)
        return stat

    def autopilot_alert(self, stat, msgs):
        alert = msgs[-1].state
        name, diag = self.alert[alert]
        stat.summary(diag, name)
        return stat


if __name__ == '__main__':
    try:
        rospy.init_node('diagnostics', anonymous=True)
        BebopDiagnostics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
