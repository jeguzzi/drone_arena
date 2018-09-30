#!/usr/bin/env python

import rospy

from crazyflie_driver.srv import AddCrazyflie, RemoveCrazyflie, UpdateParams
from crazyflie_driver.srv import AddCrazyflieRequest, RemoveCrazyflieRequest
from crazyflie_driver.msg import LogBlock
from std_msgs.msg import Float32
import diagnostic_msgs
import diagnostic_updater


def uri(radio_id, channel, bandwith, adress):
    return "radio://{radio_id}/{channel}/{bandwith}/E7E7E7E7{adress:02X}".format(**locals())


class CFSupervisor(object):

    def init_diagnostics(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Crazyradio")
        self.updater.add("Connections", self.connections_diagnostics)
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)

    def update_diagnostics(self, event):
        self.updater.update()

    def connections_diagnostics(self, stat):
        if self.connectable == self.connected:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Missing some crazyflies")
        for cf in self.connectable:
            if cf in self.connections:
                connected = self.connections[cf]
                others = list(self.cf[cf] - {connected})
                stat.add(cf, '{connected} ({others})'.format(**locals()))
            else:
                others = self.cf[cf]
                stat.add(cf, '({others})'.format(**locals()))

    def update_rssi(self, msg, name):
        # print(msg, name)
        self.rssi[name] = (rospy.Time.now(), msg.data)

    def check_connections(self):
        for name in list(self.connected):
            if name not in self.rssi or (rospy.Time.now() - self.rssi[name][0]).to_sec() > 5:
                rospy.loginfo("Unconnect %s %s", name, self.connections[name])
                res = self.unconnect(name, self.connections[name])
                if res:
                    del self.connections[name]
                    if name in self.rssi:
                        del self.rssi[name]
                    self.connected.remove(name)
                    self.rssi_sub[name].unregister()
                else:
                    rospy.error("Should never happen %s", name)

    def update(self, evt=None):
        self.check_connections()
        for name in self.connectable - self.connected:
            for id in self.cf[name]:
                rospy.loginfo("Try to connect %s at id %s", name, id)
                if self.connect(name, id):
                    rospy.loginfo("Connected")
                    self.connected.add(name)
                    self.connections[name] = id
                    self.rssi_sub[name] = rospy.Subscriber(
                        '{name}/rssi'.format(**locals()), Float32, self.update_rssi, name)
                    rospy.loginfo("Set params")
                    self.set_params(name)
                    rospy.loginfo("Done")
                    break
                else:
                    rospy.loginfo("Not connected")
                    rospy.sleep(0.1)

    def set_params(self, name):
        service_name = '{name}/update_params'.format(**locals())
        rospy.wait_for_service(service_name)
        update_params = rospy.ServiceProxy(service_name, UpdateParams)
        for param in self.params[name]:
            param_name = param["name"]
            rospy.set_param('{name}/{param_name}'.format(**locals()), param['value'])
        update_params([param['name'] for param in self.params[name]])

    def unconnect(self, name, id):
        req = RemoveCrazyflieRequest()
        req.uri = self.uri(id)
        res = self.remove_crazyflie(req)
        return True

    def connect(self, name, id):
        req = AddCrazyflieRequest()
        req.uri = self.uri(id)
        req.tf_prefix = name
        req.roll_trim = 0
        req.pitch_trim = 0
        req.enable_logging = True
        req.enable_parameters = True
        req.use_ros_time = True
        req.enable_logging_imu = False
        req.enable_logging_temperature = False
        req.enable_logging_magnetic_field = False
        req.enable_logging_pressure = False
        req.enable_logging_battery = True
        req.enable_logging_packets = False
        req.enable_logging_odom = True
        req.enable_logging_state = True
        # Make the destroyer of the CF on the server timeout (and crash)
        # req.log_blocks = [
        #     LogBlock(topic_name='thrust', frequency=100, variables=['stabilizer.thrust'])]
        try:
            res = self.add_crazyflie(req)
        except Exception as e:
            print(e)
            return False
        return res.result

    def __init__(self):
        crazyradio = rospy.get_param('~crazyradio')
        channel = crazyradio['channel']
        bandwidth = crazyradio['bandwidth']
        radio_id = crazyradio['id']
        self.uri = lambda adress: uri(radio_id, channel, bandwidth, adress)
        self.cf = {}
        self.rssi = {}
        self.params = {}
        print(crazyradio['crazyflies'])
        for cf in crazyradio['crazyflies']:
            self.cf.setdefault(cf['name'], set()).add(cf['adress'])
            self.params[cf['name']] = cf['params']
        self.connectable = set(self.cf.keys())
        self.connected = set()
        self.connections = {}
        self.rssi_sub = {}
        rospy.wait_for_service('add_crazyflie')
        self.add_crazyflie = rospy.ServiceProxy('add_crazyflie', AddCrazyflie)
        rospy.wait_for_service('remove_crazyflie')
        self.remove_crazyflie = rospy.ServiceProxy('remove_crazyflie', RemoveCrazyflie)
        rospy.Timer(rospy.Duration(10), self.update)
        self.init_diagnostics()

        self.update()


if __name__ == '__main__':
    try:
        rospy.init_node('cf_supervisor')
        CFSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
