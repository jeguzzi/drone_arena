#!/usr/bin/env python

import rospy

from crazyflie_driver.srv import AddCrazyflie, RemoveCrazyflie, UpdateParams
from crazyflie_driver.srv import AddCrazyflieRequest, RemoveCrazyflieRequest
from crazyflie_driver.msg import FlightState, LogBlock
from std_msgs.msg import Bool
import diagnostic_msgs
import diagnostic_updater


def uri(radio_id, channel, bandwith, id):
    return "radio://{radio_id}/{channel}/{bandwith}/E7E7E7E7{id:02X}".format(**locals())


def connection_request(uri, name, crazyradio_tx_power=0, enable_logging_front_net=False,
                       front_net_frame='base_link', front_net_rate=10, topics={}):
    # frequency is in ms
    log_blocs = [LogBlock(topic_name=topic, frequency=vs['period'], variables=vs['variables'])
                 for topic, vs in topics.items()]
    return AddCrazyflieRequest(
        uri=uri, crazyradio_tx_power=crazyradio_tx_power, tf_prefix=name, enable_logging=True,
        enable_parameters=True, use_ros_time=True,
        enable_logging_battery=True, enable_logging_odom=True, enable_logging_state=True,
        enable_logging_front_net=enable_logging_front_net, front_net_frame=front_net_frame,
        front_net_rate=front_net_rate, log_blocks=log_blocs)


class CFSupervisor(object):

    def init_diagnostics(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Crazyradio")
        self.updater.add("Crazyflie {0}".format(self.name), self.connection_diagnostics)
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)

    def update_diagnostics(self, event):
        self.updater.update()

    def connection_diagnostics(self, stat):
        if self.current_id:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.OK,
                "Connected to {0} {1}".format(self.current_id, list(self.ids - {self.current_id})))
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         "Not connected {0}".format(list(self.ids)))

    def check_connection(self, evt=None):
        if self.current_id is not None and not self.connected:
            rospy.loginfo("Unconnect %s", self.current_id)
            if self.unconnect(self.current_id):
                self.current_id = None
                self.connected_pub.publish(False)
                if self.state_sub:
                    self.state_sub.unregister()
                    self.state_sub = None
                self.try_to_connect()
                return
            else:
                rospy.logerr("Failed to unconnect %s", self.current_id)

    def try_to_connect(self, evt=None):
        if self.current_id is None:
            for id in self.ids:
                if self.connect(id):
                    self.upload_params(id)
                    self.connected_pub.publish(True)
                    self.last_update = rospy.Time.now()
                    # rospy.loginfo("Reset last_update %s", self.last_update)
                    self.current_id = id
                    self.state_sub = rospy.Subscriber('state', FlightState, self.update_state,
                                                      queue_size=1)
                    return
            rospy.loginfo("Will retry")
            rospy.Timer(rospy.Duration(self.retry_interval), self.try_to_connect, oneshot=True)

    def unconnect(self, id):
        req = RemoveCrazyflieRequest()
        req.uri = self.uri(id)
        self.remove_crazyflie(req)
        return True

    def connect(self, id):
        uri = self.uri(id)
        log_front_net = self.front_net.get(id, False)
        front_net_frame = self.front_net_frame.get(id, 'base_link')
        front_net_rate = self.front_net_rate.get(id, 10)
        topics = self.topics.get(id, {})
        rospy.loginfo('Try to connect to CF @ %s with power %d dB', self.uri(id),
                      self.crazyradio_tx_power)
        if log_front_net:
            rospy.loginfo('and front net wrt frame %s' % front_net_frame)
        req = connection_request(uri, self.name, self.crazyradio_tx_power,
                                 enable_logging_front_net=log_front_net,
                                 front_net_frame=front_net_frame,
                                 front_net_rate=front_net_rate, topics=topics)
        try:
            res = self.add_crazyflie(req)
        except Exception as e:
            rospy.logerr('Exception %s while connecting %s to id %s', e, self.name, id)
            return False
        return res.result

    def __init__(self):
        self.name = rospy.get_param('~name')
        channel = rospy.get_param('~radio/channel')
        bandwidth = rospy.get_param('~radio/bandwidth', '2M')
        radio_id = rospy.get_param('~radio/id', 0)
        self.crazyradio_tx_power = rospy.get_param('~radio/power_db', 0)
        self.uri = lambda id: uri(radio_id, channel, bandwidth, id)
        params = {}
        self.ids = set([cf['id'] for cf in rospy.get_param('~crazyflies')])
        self.params = {cf['id']: dict((cf.get('params', {}).items()))
                       for cf in rospy.get_param('~crazyflies')}
        self.params.update(params)
        self.front_net = {cf['id']: cf.get('front_net', False)
                          for cf in rospy.get_param('~crazyflies')}
        self.front_net_frame = {cf['id']: cf.get('front_net_frame', 'base_link')
                                for cf in rospy.get_param('~crazyflies')}
        self.front_net_rate = {cf['id']: cf.get('front_net_rate', 10)
                               for cf in rospy.get_param('~crazyflies')}
        self.topics = {cf['id']: cf.get('topics', {})
                       for cf in rospy.get_param('~crazyflies')}
        self.current_id = None
        self.retry_interval = rospy.get_param('~retry_interval', 10)
        self.state_timeout = rospy.get_param('~connection_timeout', 3)
        self.color_period = rospy.get_param('~color_period', 2)

        self.connected_pub = rospy.Publisher('connected', Bool, queue_size=1, latch=True)
        self.connected_pub.publish(False)
        self.state_sub = None
        rospy.wait_for_service('/add_crazyflie')
        self.add_crazyflie = rospy.ServiceProxy('/add_crazyflie', AddCrazyflie)
        rospy.wait_for_service('/remove_crazyflie')
        self.remove_crazyflie = rospy.ServiceProxy('/remove_crazyflie', RemoveCrazyflie)
        self.init_diagnostics()
        self.try_to_connect()
        rospy.Timer(rospy.Duration(1), self.check_connection)

    def update_state(self, msg):
        self.last_update = msg.header.stamp
        # rospy.loginfo("Has received state at %s", msg.header.stamp)

    @property
    def connected(self):
        # rospy.loginfo("Check connected %s", self.last_update)
        return (rospy.Time.now() - self.last_update).to_sec() < self.state_timeout

    def upload_params(self, id):
        params = self.params[id]
        for name, value in params.items():
            if value is not None:
                rospy.set_param(name, value)
        rospy.wait_for_service('update_params')
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        update_params(list(params.keys()))


if __name__ == '__main__':
    try:
        rospy.init_node('cf_supervisor')
        CFSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
