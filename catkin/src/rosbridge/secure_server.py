#!/usr/bin/env python
#
# This app should:
# - check whether there's a generated SSL certificate and key
# - generate the key and certificate
# - distribute the cert + key among all nodes that talk to wss://
# - run rosrun rosbridge_server
#   rosbridge_websocket ~certfile ${HOME}/etc/rosbridge.crt \
#    ~keyfile ${HOME}/etc/rosbridge.key

import os


class SecureBridgeRunner():
    def __init__(self):
        self.catkin_home = "catkin/"
        self.crt_path = "/home/lg/etc/slinky.crt"
        self.key_path = "/home/lg/etc/slinky.key"
        self.rosbridge_cmd = "export ROS_MASTER_URI='http://lg-head:11311' ;\
            cd %s ; bash devel/setup.sh ; cd - ; \
            rosrun rosbridge_server rosbridge_websocket ~certfile %s ~keyfile %s" \
            % (self.catkin_home, self.crt_path, self.key_path)

    def run_rosbridge(self):
        os.system(self.rosbridge_cmd)

    def run(self):
        self.run_rosbridge()

if __name__ == '__main__':
    bridge = SecureBridgeRunner()
    bridge.run()
