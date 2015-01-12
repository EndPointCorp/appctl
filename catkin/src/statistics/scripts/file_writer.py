#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import sys
import json
import time
from statistics.srv import SessionQuery


class GeolocationDataException(Exception):
    pass


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
        if self.country is None or\
           self.store_id is None or\
           self.retailer is None:
               raise GeolocationDataException("You need to specify proper store metadata")
        self._wait_for_service()
        pass

    def _wait_for_service(self):
        rospy.loginfo("Waiting for the /statistics/session service to become available")
        rospy.wait_for_service('statistics/session')
        rospy.loginfo("Statistics aggretator service has become available")
        pass

    def _call_service(self):
        service_call = rospy.ServiceProxy('statistics/session', SessionQuery)
        response = service_call(erase=True)
        rospy.logdebug("Received response from service: %s" % response)
        return response

    def _sleep_between_requests(self):
        rospy.logdebug("Sleeping for %s seconds before writing stats" % self.interval)
        rospy.sleep(self.interval)
        pass

    def _init_node(self):
        rospy.init_node('statistics')
        pass

    def _jsonize_string(self, string):
        sessions = string.replace("'{", "{").replace("}'", "}")
        rospy.logdebug("Deserialized sessions => %s" % sessions)
        return json.loads(sessions)

    def _finalize_report_data(self, report_contents):
        """
        - fill report_time and end_ts with current time
        - fill start_ts with current_time minus self.interval
        """
        unix_now = int(time.time())
        report_contents['report_time'] = unix_now
        report_contents['start_ts'] = unix_now - self.interval
        report_contents['end_ts'] = unix_now
        return report_contents

    def _compose_and_write_json(self, sessions):
        """
        - jsonize the sessions string received from json aggregator
        - convert sessions list to json
        - finalize report and write the file
        """
        sessions = self._jsonize_string(sessions)
        report_contents = {"report_time": 0,
                           "start_ts": 0,
                           "status": "on",
                           "experience_type": "portal",
                           "end_ts": 0,
                           "metadata": {"country": self.country,
                                        "store_id": self.store_id,
                                        "retailer": self.retailer
                                        },
                           "sessions": []
                           }
        for session in sessions:
            report_contents['sessions'].append(session)

        report_contents = self._finalize_report_data(report_contents)
        with open(self.path + "/" +  str(int(time.time())) + ".json", 'w') as json_file:
            rospy.loginfo("Writing file %s" % json_file.name)
            json_file.write(json.dumps(report_contents, sort_keys=True, indent=4, separators=(',', ': ')))
            del report_contents

    def _spin(self):
        try:
            response = self._call_service()
        except rospy.service.ServiceException, e:
            rospy.logfatal("Could not connect to /statistics/session service because: %s" % e)
        self._compose_and_write_json(response.sessions)
        pass

    def run(self):
        rospy.logdebug("Starting session statistics service")
        while not rospy.is_shutdown():
            self._sleep_between_requests()
            self._spin()
            pass
        pass

if __name__ == '__main__':
    try:
        FileWriter().run()
    except rospy.ROSInterruptException:
        pass
    except rospy.service.ServiceException:
        pass
