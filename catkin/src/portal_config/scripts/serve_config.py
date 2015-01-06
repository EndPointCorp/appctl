#!/usr/bin/env python

import rospy
from portal_config.srv import *


class ConfigRequestHandler():
    def __init__(self, url):
        self.url = url

    def get_config(self):
        return '{"foo": "bar"}'

    def handle_request(self, request):
        config = self.get_config()
        return PortalConfigResponse(config)


def main():
    rospy.init_node('portal_config')
    url = rospy.get_param('~url', 'http://lg-head/portal/config.json')

    handler = ConfigRequestHandler(url)

    s = rospy.Service(
        '/portal_config/query',
        PortalConfig,
        handler.handle_request
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
