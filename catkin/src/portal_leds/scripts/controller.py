#!/usr/bin/env python

import rospy
from std_msgs.msg import Duration, String


class LEDController():
    def __init__(self, led_pub, timeout,
                 init_command, hello_command, goodbye_command):

        self.led_pub = led_pub
        self.timeout = timeout
        self.init_command = init_command
        self.hello_command = hello_command
        self.goodbye_command = goodbye_command
        self.occupied = False

        self.publish_command(init_command)

    def publish_command(self, command):
        self.led_pub.publish(command)

    def set_occupancy(self, occupied):
        if occupied and not self.occupied:
            self.publish_command(self.hello_command)
        elif not occupied and self.occupied:
            self.publish_command(self.goodbye_command)
        self.occupied = occupied

    def handle_occupancy_msg(self, msg):
        duration = msg.data
        self.set_occupancy(duration < self.timeout)


def main():
    rospy.init_node('portal_led_controller', anonymous=True)

    led_pub = rospy.Publisher('/portal_leds/command', String, queue_size=2)

    init_command = rospy.get_param('~init_command', 'c')
    hello_command = rospy.get_param('~hello_command', 'h')
    goodbye_command = rospy.get_param('~goodbye_command', 'g')
    timeout = rospy.Duration.from_sec(rospy.get_param('~timeout', 1.5))

    controller = LEDController(
        led_pub,
        timeout,
        init_command,
        hello_command,
        goodbye_command
    )

    rospy.Subscriber(
        '/portal_occupancy/occupancy/inactive_duration',
        Duration,
        controller.handle_occupancy_msg
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
