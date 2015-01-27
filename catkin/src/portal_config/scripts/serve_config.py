#!/usr/bin/env python

import rospy
import urllib2
from portal_config.srv import *

NODE_NAME = 'portal_config'
SRV_QUERY = '/'.join(('', NODE_NAME, 'query'))
PARAM_URL = '~url'
DEFAULT_URL = 'http://lg-head/portal/config.json'

# XXX TODO: return an error if the config file isn't valid JSON

class ConfigRequestHandler():
    def get_url(self):
        return rospy.get_param(PARAM_URL, DEFAULT_URL)

    def get_config(self):
        url = self.get_url()
        response = urllib2.urlopen(url)
        return response.read()

    def handle_request(self, request):
        config = self.get_config()
        return PortalConfigResponse(config)


def main():
    rospy.init_node(NODE_NAME)

    #if not rospy.has_param(PARAM_URL):
    #    rospy.set_param(PARAM_URL, DEFAULT_URL)

    handler = ConfigRequestHandler()

    s = rospy.Service(
        SRV_QUERY,
        PortalConfig,
        handler.handle_request
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4 smartindent
