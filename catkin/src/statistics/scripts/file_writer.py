#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import sys
import json
import time
from statistics.srv import SessionQuery


class FileWriter:
    """
    - periodically collects session events from /statistics/session service
    - wraps session events with metadata and renders json files to filesystem
    """

    def __init__(self):
        self.node = self._init_node()
        self.path = rospy.get_param('~json_output_path', '/tmp')
        self.interval = rospy.get_param('~aggregation_interval', 300)
        self.country = rospy.get_param('~country', None)
        self.store_id = rospy.get_param('~store_id', None)
        self.retailer = rospy.get_param('~retailer', None)
        if self.country == None or\
           self.store_id == None or\
           self.retailer == None:
               raise GeolocationDataException("You need to specify proper store metadata")
        self._wait_for_service()
        self.json_template = {
                              "report_time": 0,
                              "start_ts": 0,
                              "end_ts": 0,
                              "metadata":
                }
        pass

    def _wait_for_service(self):
        rospy.loginfo("Waiting for the /statistics/session service to become available")
        rospy.wait_for_service('statistics/session')
        rospy.loginfo("Statistics aggretator service has become available")

    def _call_service(self):
        service_call = rospy.ServiceProxy('statistics/session', SessionQuery)
        response = service_call(erase=True)
        rospy.loginfo("Received response from service: %s" % response)
        return response

    def _sleep_between_requests(self):
        rospy.loginfo("Sleeping for %s seconds between /statistics/session call" % self.interval)
        d = rospy.Duration(self.interval, 0)
        rospy.sleep(d)
        pass

    def _init_node(self):
        rospy.init_node('statistics')

    def _compose_json(self, sessions):
        sessions = sessions.replace("'{", "{").replace("}'", "}")
        rospy.loginfo("Deserialized sessions => %s" % sessions)
        for session in json.loads(sessions):
            rospy.loginfo("Deserialized session => %s" % session)

        unix_now =  time.time()

    def _spin(self):
        response = self._call_service()
        json_file = self._compose_json(response.sessions)
        #json_file.write()
        #json_file.close()

    def run(self):
        rospy.loginfo("Starting session statistics service")
        while not rospy.is_shutdown():
            self._spin()
            rospy.loginfo("Sleeping for %s seconds between /statistics/session call" % self.interval)
            rospy.sleep(self.interval)
            pass
        pass

if __name__ == '__main__':
    try:
        FileWriter().run()
    except rospy.ROSInterruptException:
        pass
