#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

import subprocess


class PDUController():
    """Routes control messages to a PDU."""
    def __init__(self, community, host):
        self.community = community
        self.host = host

    def handle_outlet_msg(self, msg):
        topic = msg._connection_header['topic']
        port = topic.split('/')[-1]
        self.set_port_state(port, msg.data)

    def set_port_state(self, port, state):
        if state:
            value = "1"
        else:
            value = "2"

        # TODO(mv): use proper SNMP bindings
        subprocess.call(["snmpset","-v","2c","-c", self.community, self.host,
                        "1.3.6.1.4.1.1718.3.2.3.1.11.1.1." + port,
                        "i", value])


def main():
    rospy.init_node('servertech_pdu')

    community = rospy.get_param('~community', 'acme')
    host = rospy.get_param('~host')
    port_count = rospy.get_param('~port_count', 2)

    controller = PDUController(community, host)

    # Subscribe each outlet.
    for i in range(1, port_count + 1):
        rospy.Subscriber(
            '{}/outlet/{}'.format(rospy.get_name(), i),
            Bool,
            controller.handle_outlet_msg
        )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
