#!/usr/bin/env python

import rospy
import urllib2
from portal_config.srv import *

# XXX TODO: return an error if the config file isn't valid JSON

class ConfigRequestHandler():
    def __init__(self, url):
        self.url = url

    def get_config(self):
        response = urllib2.urlopen(self.url)
        return response.read()

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

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4 smartindent
