#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String


class LEDController():
    def __init__(self, led_pub, init_command, hello_command, goodbye_command):
        self.led_pub = led_pub
        self.init_command = init_command
        self.hello_command = hello_command
        self.goodbye_command = goodbye_command
        self.occupied = False

        self.publish_command(init_command)

    def publish_command(self, command):
        self.led_pub.publish(command)

    def change_occupancy(self, occupied):
        self.occupied = occupied
        if (occupied):
            self.publish_command(self.hello_command)
        else:
            self.publish_command(self.goodbye_command)

    def handle_occupancy_msg(self, msg):
        occupied = msg.data
        if (occupied != self.occupied):
            self.change_occupancy(occupied)


def main():
    rospy.init_node('portal_led_controller', anonymous=True)

    led_pub = rospy.Publisher('/portal_leds/command', String, queue_size=2)

    init_command = rospy.get_param('init_command', 'c')
    hello_command = rospy.get_param('hello_command', 'h')
    goodbye_command = rospy.get_param('goodbye_command', 'g')

    controller = LEDController(
        led_pub,
        init_command,
        hello_command,
        goodbye_command
    )

    rospy.Subscriber(
        '/portal_occupancy/occupancy/is_active',
        Bool,
        controller.handle_occupancy_msg
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
